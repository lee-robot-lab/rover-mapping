# ================================================================
# mission_runtime.py
# 설명: site 좌표 pose로 대표점 방문·등판 중단·후퇴·우회·복귀와 속도 요청을 계산한다.
# 사용법: python -m experiments.handheld_mapping.mission_runtime MISSION INPUT.jsonl OUTPUT
# ================================================================
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from experiments.handheld_mapping.site_mission import SiteMission, dense_path
from experiments.handheld_mapping.storage import atomic_json
from experiments.handheld_mapping.sensor_safety import SensorSafety


def angle_wrap(value: float) -> float:
    return math.atan2(math.sin(value),math.cos(value))


class MissionRuntime:
    """유효한 site 관측에만 반응하고 실제 지나온 경로로 후퇴하는 임무 상태기계."""

    def __init__(self, mission: SiteMission, *, safety: SensorSafety | None=None) -> None:
        if not mission.validate()['accepted']:raise ValueError('valid mission required')
        self.safety=safety;self.safety_status=None
        self.mission,self.settings=mission,mission.settings
        self.state='to_junction';self.route='30';self.direction='up'
        self.visits={'entry30':'pending','entry15':'pending','entry10':'pending',
                     'summit':'pending','return':'pending'}
        self.route_status={name:{'up':'unknown','down':'unknown'} for name in ('10','15','30')}
        self.events: list[dict]=[];self.failed_routes: list[str]=[]
        self.last_stamp=None;self.last_valid_stamp=None;self.last_position=None
        self.pose_epoch=None;self.epoch_changed=False
        self.paused=True;self.started=False;self.paused_samples=0
        self.command={'vx_m_s':0.,'yaw_rate_rad_s':0.}
        self.breadcrumbs: list[list]=[]
        self._set_path(mission.connector())

    def _set_path(self, points: np.ndarray) -> None:
        self.points=dense_path(points);self.index=0
        self.arcs=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(self.points,axis=0),axis=1))]
        self.phase_elapsed=self.effort=self.window_progress=0.

    def _event(self, kind: str, stamp: float, **details) -> None:
        self.events.append({'type':kind,'stamp_s':stamp,'state':self.state,'route':self.route,**details})

    def _output(self, reason: str='active', command: dict | None=None) -> dict:
        self.command=command or {'vx_m_s':0.,'yaw_rate_rad_s':0.}
        result={'state':self.state,'route':self.route,'direction':self.direction,'reason':reason,
                'visits':self.visits.copy(),'route_status':{k:v.copy() for k,v in self.route_status.items()},
                'command':self.command.copy(),'frame':'site','site_identity':self.mission.site.identity,
                'command_stamp_s':self.last_stamp,
                'valid_until_s':None if self.last_stamp is None else self.last_stamp+self.settings['pose_max_age_s'],
                'hardware_commands_enabled':False}
        if self.safety_status is not None:
            result['sensor_safety']=self.safety_status.copy()
            result['valid_until_s']=self.safety_status['valid_until_s']
        return result

    def observe_imu(self, stamp_s: float, now_s: float, valid: bool=True) -> None:
        """비동기 IMU 입력을 수용하고 watchdog에서 pose와 독립적으로 freshness를 검사한다."""
        if self.safety is not None:self.safety.observe_imu(stamp_s,now_s,valid)

    def watchdog(self, now_s: float) -> dict:
        """센서 입력과 독립적으로 주기 호출하며 끊김 시 두 속도를 0으로 만든다."""
        if self.state in ('complete','failed'):return self._output(self.state)
        if self.safety is None:return self._pause('sensor_watchdog_disabled')
        self.safety_status=self.safety.status(now_s)
        if self.safety_status['mode']=='stop':return self._pause(self.safety_status['reason'])
        return self._output('watchdog_ok',command=self.command.copy())

    def _pause(self, reason: str) -> dict:
        if self.safety is not None and reason!='recovering':
            self.safety.inhibit(reason)
            self.safety_status={'mode':'stop','reason':reason,'scale':0.,'valid_until_s':None}
        self.paused=True;self.effort=0.;self.paused_samples+=1
        return self._output(reason)

    def update(self, sample: dict, now_s: float) -> dict:
        """stamp와 receipt가 같은 명시 clock의 관측 한 개를 처리한다."""
        if self.state in ('complete','failed'):return self._output(self.state)
        try:
            stamp=float(sample['stamp_s']);now=float(now_s)
            position=np.asarray(sample['position_m'],float);yaw=float(sample['yaw_rad'])
            if (position.shape not in ((2,),(3,)) or not np.isfinite(np.r_[position,yaw,stamp,now]).all()):
                raise ValueError('finite XY/XYZ/yaw/time required')
            position=position[:2]
        except (KeyError,ValueError,TypeError,OverflowError):return self._pause('invalid_pose')
        if sample.get('frame')!='site' or sample.get('site_identity')!=self.mission.site.identity:
            return self._pause('unaligned_frame')
        if now-stamp>self.settings['pose_max_age_s'] or now<stamp-.02:
            return self._pause('stale_pose')
        if self.last_stamp is not None and stamp<=self.last_stamp:return self._pause('nonmonotonic_pose')
        self.last_stamp=stamp
        if self.safety is not None:
            if 'imu_stamp_s' in sample:self.observe_imu(sample['imu_stamp_s'],now,sample.get('imu_valid',True))
            self.safety.observe_pose(stamp,now,sample.get('tracking_valid'),sample.get('tracking_quality','good'))
            self.safety_status=self.safety.status(now)
            if self.safety_status['mode']=='stop':return self._pause(self.safety_status['reason'])
        if sample.get('tracking_valid') is not True:return self._pause('tracking_lost')
        epoch=sample.get('pose_epoch',0)
        if type(epoch) not in (int,str):return self._pause('invalid_pose_epoch')
        if self.pose_epoch is None:self.pose_epoch=epoch
        if self.pose_epoch!=epoch:self.epoch_changed=True
        if self.epoch_changed:return self._pause('pose_epoch_changed')
        if not self.started and np.linalg.norm(position-self.mission.start[:2])>self.settings['arrival_tolerance_m']*2:
            return self._pause('initial_pose_away_from_start')
        if self.last_position is not None and np.linalg.norm(position-self.last_position)>self.settings['max_pose_jump_m']:
            return self._pause('pose_jump')
        if self.last_position is not None:
            limit=self.settings['trial_slope_deg' if self.route=='30' and self.state in ('trial_entry','ascending','retreating') else 'normal_slope_deg']
            # Observed pose may drift by the arrival tolerance that the planned-path radius already budgets.
            if not self.mission.site.validate_path([self.last_position,position],limit,self.settings['arrival_tolerance_m'])['accepted']:
                return self._pause('unsafe_observed_motion')
        dt=0. if self.paused or self.last_valid_stamp is None else stamp-self.last_valid_stamp
        if dt>self.settings['pose_max_age_s']:
            dt=0.;self.effort=0.
        self.started=True;self.paused=False;self.last_valid_stamp=stamp;self.last_position=position.copy()
        self.phase_elapsed+=dt
        if self.state in ('trial_entry','ascending'):
            if not self.breadcrumbs or np.linalg.norm(position-self.breadcrumbs[-1])>=.025:
                self.breadcrumbs.append(position.tolist())
        if len(self.breadcrumbs)>20_000:return self._fail(stamp,'breadcrumb_limit')
        entry=self.mission.routes[self.route][1]
        if self.visits['entry'+self.route]!='reached' and np.linalg.norm(position-entry)<=self.settings['arrival_tolerance_m']:
            self.visits['entry'+self.route]='reached';self._event('entry_reached',stamp)
        if np.linalg.norm(position-self.points[-1])<=self.settings['arrival_tolerance_m']:
            handoff_safe=True
            if self.state=='ascending':
                descent=self.mission.descent_path();normal=self.settings['normal_slope_deg']
                # Stay in ascent until the actual pose can enter the normal descent corridor.
                if self.mission.site.validate_path(descent,normal)['accepted']:
                    handoff_safe=self.mission.site.validate_path(np.vstack((position,descent)),normal)['accepted']
            if handoff_safe:return self._arrived(position,stamp)
        while self.index<len(self.points)-1 and np.linalg.norm(position-self.points[self.index])<=self.settings['arrival_tolerance_m']:
            self.index+=1
        progress=float(self.arcs[self.index])
        if self.paused or progress-self.window_progress>=self.settings['min_progress_m']:
            self.window_progress=progress;self.effort=0.
        elif abs(self.command['vx_m_s'])>.01:
            self.effort+=dt
        budget=(self.settings['max_attempt_s'] if self.state=='ascending' else
                max(self.settings['max_attempt_s'],self.arcs[-1]/self.settings['max_speed_m_s']*4))
        if self.effort>=self.settings['stuck_window_s'] or self.phase_elapsed>budget:
            if self.state in ('ascending','trial_entry'):return self._ascent_failed(position,stamp)
            return self._fail(stamp,'no_progress_'+self.state)
        reverse=self.state=='retreating'
        target=self.points[self.index]
        limit=self.settings['trial_slope_deg' if self.route=='30' and self.state in ('ascending','trial_entry','retreating')
                            else 'normal_slope_deg']
        if not self.mission.site.validate_path([position,target],limit,self.settings['arrival_tolerance_m'])['accepted']:
            return self._pause('unsafe_local_segment')
        heading=math.atan2(target[1]-position[1],target[0]-position[0])+(math.pi if reverse else 0.)
        error=angle_wrap(heading-yaw)
        yaw_rate=float(np.clip(error*2.,-self.settings['max_yaw_rate_rad_s'],self.settings['max_yaw_rate_rad_s']))
        speed=self.settings['max_speed_m_s']*max(0.,math.cos(error)) if abs(error)<1. else 0.
        scale=1. if self.safety_status is None else self.safety_status['scale']
        return self._output(command={'vx_m_s':float((-speed if reverse else speed)*scale),'yaw_rate_rad_s':yaw_rate*scale})

    def _arrived(self, position: np.ndarray, stamp: float) -> dict:
        if self.state=='to_junction':
            self.state='trial_entry';self.route='30';self.breadcrumbs=[position.tolist()]
            self.visits['entry30']='active';self._set_path(self.mission.routes['30'][:2])
        elif self.state=='trial_entry':
            self.visits['entry30']='reached';self.state='ascending'
            self._set_path(np.vstack((position,self.mission.routes['30'][1:])))
        elif self.state=='ascending':
            self.visits['summit']='reached';self.route_status[self.route]['up']='passed'
            self._event('summit_reached',stamp)
            if not self.mission.site.validate_path(self.mission.descent_path(),self.settings['normal_slope_deg'])['accepted']:
                self.route_status['10']['down']='failed'
                return self._fail(stamp,'descent_route_unavailable')
            self.state='descending';self.route='10';self.direction='down';self.visits['return']='active'
            self._set_path(np.vstack((position,self.mission.descent_path())))
        elif self.state=='retreating':
            remaining=[name for name in ('30','15','10') if name not in self.failed_routes]
            if not remaining:return self._fail(stamp,'all_ascent_routes_failed')
            self.route=remaining[0];self.direction='up';self.state='ascending'
            self.breadcrumbs=[position.tolist()];self.visits['entry'+self.route]='active'
            self._set_path(np.vstack((position,self.mission.path(self.route))))
            self._event('fallback_selected',stamp)
        elif self.state=='descending':
            self.route_status['10']['down']='passed'
            if np.linalg.norm(position-self.mission.start[:2])<=self.settings['arrival_tolerance_m']:
                self.visits['return']='reached';self.state='complete';self._event('return_reached',stamp)
            else:
                self.state='returning'
                self._set_path(np.vstack((position,self.mission.connector()[::-1])))
        elif self.state=='returning':
            self.visits['return']='reached';self.state='complete';self._event('return_reached',stamp)
        self._event('phase_started',stamp,direction=self.direction)
        return self._output('phase_transition')

    def _ascent_failed(self, position: np.ndarray, stamp: float) -> dict:
        self.failed_routes.append(self.route);self.route_status[self.route]['up']='failed'
        if self.visits['entry'+self.route]!='reached':self.visits['entry'+self.route]='failed'
        self._event('ascent_failed',stamp,reason='observed_no_progress_not_wheel_slip')
        points=np.asarray(self.breadcrumbs)
        if len(points)<2:points=np.vstack((position,self.mission.junction))
        self.state='retreating';self.direction='down'
        self._set_path(np.vstack((position,points[::-1])))
        self._event('retreat_started',stamp,path=self.points.tolist())
        return self._output('ascent_failed')

    def _fail(self, stamp: float, reason: str) -> dict:
        self.state='failed'
        if self.visits['summit']!='reached':self.visits['summit']='failed'
        if self.visits['return']!='reached':self.visits['return']='failed'
        self._event('mission_failed',stamp,reason=reason)
        return self._output(reason)


def run_jsonl(mission: SiteMission, input_path: Path, output: Path, *, safety: SensorSafety | None=None) -> dict:
    """명시한 receipt time의 입력을 읽고 파일로만 속도 요청을 기록한다. 송신은 없다."""
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    runtime=MissionRuntime(mission,safety=safety);last=None
    with (output/'commands.jsonl').open('w') as dest:
        try:
            with Path(input_path).open() as source:
                for line in source:
                    if not line.strip():continue
                    packet=json.loads(line)
                    if 'now_s' not in packet:raise ValueError('explicit same-clock now_s receipt required')
                    if packet.get('sensor_event')=='imu':
                        runtime.observe_imu(packet['imu_stamp_s'],packet['now_s'],packet.get('imu_valid',True))
                        last=runtime.watchdog(packet['now_s'])
                    elif packet.get('watchdog') is True:last=runtime.watchdog(packet['now_s'])
                    else:last=runtime.update(packet,packet['now_s'])
                    dest.write(json.dumps({**last,'stamp_s':packet.get('stamp_s')},allow_nan=False)+'\n')
        finally:
            dest.write(json.dumps({'state':runtime.state,'reason':'input_end_stop',
                                  'command':{'vx_m_s':0.,'yaw_rate_rad_s':0.}})+'\n')
    with (output/'events.jsonl').open('w') as stream:
        for event in runtime.events:stream.write(json.dumps(event)+'\n')
    summary={'state':runtime.state,'visits':runtime.visits,'input_exhausted':True,
             'hardware_commands_enabled':False,'last_result':last}
    atomic_json(output/'summary.json',summary)
    return summary


def main() -> None:
    parser=argparse.ArgumentParser(description='파일 기반 site pose 임무 실행; 실제 모터 송신 없음')
    parser.add_argument('mission',type=Path);parser.add_argument('input',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--sensor-safety',action='store_true')
    parser.add_argument('--require-imu',action='store_true')
    args=parser.parse_args()
    safety=SensorSafety(require_imu=args.require_imu) if args.sensor_safety or args.require_imu else None
    print(json.dumps(run_jsonl(SiteMission.load(args.mission),args.input,args.output,safety=safety),ensure_ascii=False))


if __name__=='__main__':main()
