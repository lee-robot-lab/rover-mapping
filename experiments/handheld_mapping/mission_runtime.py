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


def angle_wrap(value: float) -> float:
    return math.atan2(math.sin(value),math.cos(value))


class MissionRuntime:
    """유효한 site 관측에만 반응하고 실제 지나온 경로로 후퇴하는 임무 상태기계."""

    def __init__(self, mission: SiteMission) -> None:
        if not mission.validate()['accepted']:raise ValueError('valid mission required')
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
        return {'state':self.state,'route':self.route,'direction':self.direction,'reason':reason,
                'visits':self.visits.copy(),'route_status':{k:v.copy() for k,v in self.route_status.items()},
                'command':self.command.copy(),'frame':'site','site_identity':self.mission.site.identity,
                'command_stamp_s':self.last_stamp,
                'valid_until_s':None if self.last_stamp is None else self.last_stamp+self.settings['pose_max_age_s'],
                'hardware_commands_enabled':False}

    def _pause(self, reason: str) -> dict:
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
        if self.last_position is not None and not self.paused:
            limit=self.settings['trial_slope_deg' if self.route=='30' and self.state in ('trial_entry','ascending','retreating') else 'normal_slope_deg']
            if not self.mission.site.validate_path([self.last_position,position],limit)['accepted']:
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
            return self._arrived(position,stamp)
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
        if not self.mission.site.validate_path([position,target],limit)['accepted']:
            return self._pause('unsafe_local_segment')
        heading=math.atan2(target[1]-position[1],target[0]-position[0])+(math.pi if reverse else 0.)
        error=angle_wrap(heading-yaw)
        yaw_rate=float(np.clip(error*2.,-self.settings['max_yaw_rate_rad_s'],self.settings['max_yaw_rate_rad_s']))
        speed=self.settings['max_speed_m_s']*max(0.,math.cos(error)) if abs(error)<1. else 0.
        return self._output(command={'vx_m_s':float(-speed if reverse else speed),'yaw_rate_rad_s':yaw_rate})

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
            if not self.mission.site.validate_path(self.mission.path('10','down'),self.settings['normal_slope_deg'])['accepted']:
                self.route_status['10']['down']='failed'
                return self._fail(stamp,'descent_route_unavailable')
            self.state='descending';self.route='10';self.direction='down';self.visits['return']='active'
            self._set_path(np.vstack((position,self.mission.path('10','down'))))
        elif self.state=='retreating':
            remaining=[name for name in ('30','15','10') if name not in self.failed_routes]
            if not remaining:return self._fail(stamp,'all_ascent_routes_failed')
            self.route=remaining[0];self.direction='up';self.state='ascending'
            self.breadcrumbs=[position.tolist()];self.visits['entry'+self.route]='active'
            self._set_path(np.vstack((position,self.mission.path(self.route))))
            self._event('fallback_selected',stamp)
        elif self.state=='descending':
            self.route_status['10']['down']='passed';self.state='returning'
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


def run_jsonl(mission: SiteMission, input_path: Path, output: Path) -> dict:
    """명시한 receipt time의 입력을 읽고 파일로만 속도 요청을 기록한다. 송신은 없다."""
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    runtime=MissionRuntime(mission);last=None
    with (output/'commands.jsonl').open('w') as dest:
        try:
            with Path(input_path).open() as source:
                for line in source:
                    if not line.strip():continue
                    packet=json.loads(line)
                    if 'now_s' not in packet:raise ValueError('explicit same-clock now_s receipt required')
                    last=runtime.update(packet,packet['now_s'])
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
    args=parser.parse_args();print(json.dumps(run_jsonl(SiteMission.load(args.mission),args.input,args.output),ensure_ascii=False))


if __name__=='__main__':main()
