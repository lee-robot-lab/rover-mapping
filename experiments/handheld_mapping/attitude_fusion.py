# ================================================================
# attitude_fusion.py
# 설명: 저장한 Stereo ORB 위치를 보존하며 IMU·ORB 회전을 외부 상보 필터로 합친다.
# 사용법: python -m experiments.handheld_mapping.attitude_fusion RUN --correction-s -0.04
# ================================================================
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from experiments.handheld_mapping.inertial import camera_from_urdf, validate_imu_provenance
from experiments.handheld_mapping.inertial_timing import aligned_inputs
from experiments.handheld_mapping.storage import atomic_json
from experiments.handheld_mapping.time_alignment import file_sha256


class AttitudeFilter:
    """body 회전을 예측·보정하고 시각 공백과 map 좌표 변경을 명시적으로 분리한다."""

    def __init__(self, times: np.ndarray, gyro: np.ndarray, acc: np.ndarray,
                 visual_tau_s: float = .5, gravity_tau_s: float = 2.) -> None:
        self.times=np.asarray(times,dtype=float)
        self.gyro,self.acc=np.asarray(gyro,dtype=float),np.asarray(acc,dtype=float)
        if (self.times.ndim!=1 or len(self.times)<2 or np.any(np.diff(self.times)<=0)
                or self.gyro.shape!=(len(self.times),3) or self.acc.shape!=self.gyro.shape
                or not all(np.isfinite(x).all() for x in (self.times,self.gyro,self.acc))):
            raise ValueError('invalid IMU timestamp or vectors')
        if not all(math.isfinite(x) and x>0 for x in (visual_tau_s,gravity_tau_s)):
            raise ValueError('positive filter time constants required')
        self.visual_tau_s,self.gravity_tau_s=visual_tau_s,gravity_tau_s
        self.rotation=None
        self.gravity=None
        self.time=None
        self.last_visual_time=None
        self.map_id=None

    def update(self, stamp_s: float, map_id: int, visual: np.ndarray | None,
               stationary_gravity_body: np.ndarray | None) -> tuple[np.ndarray | None,str]:
        """유효 ORB로 기준을 잡고 최대0.3초 손실 구간에서만 회전을 예측한다."""
        t=float(stamp_s)
        if not math.isfinite(t) or (self.time is not None and t<=self.time):
            raise ValueError('nonmonotonic frame timestamp')
        if visual is not None:
            visual=np.asarray(visual,dtype=float)
            if (visual.shape!=(3,3) or not np.isfinite(visual).all()
                    or not np.allclose(visual.T@visual,np.eye(3),atol=.003)
                    or not np.isclose(np.linalg.det(visual),1.,atol=.003)):
                raise ValueError('invalid visual rotation')
            visual=Rotation.from_matrix(visual).as_matrix()
        previous=self.time
        self.time=t
        if visual is None and map_id>=0 and map_id!=self.map_id:
            self.rotation=None
            self.gravity=None
            return None,'unavailable_map_change'
        if visual is not None and (self.rotation is None or map_id!=self.map_id
                                    or t-self.last_visual_time>.3):
            return self._anchor(t,map_id,visual,stationary_gravity_body,'visual_reanchor')
        if self.rotation is None or previous is None:
            return None,'unavailable'
        first=int(np.searchsorted(self.times,previous,side='right'))-1
        last=int(np.searchsorted(self.times,t,side='left'))
        valid=(0<=first and last<len(self.times)
               and np.max(np.diff(self.times[first:last+1]),initial=0.)<=.100000001
               and t-previous<=.3)
        if not valid:
            self.gravity=None
            if visual is not None:
                return self._anchor(t,map_id,visual,stationary_gravity_body,'visual_fallback_imu_gap')
            self.rotation=None
            return None,'unavailable'
        if visual is None and t-self.last_visual_time>.3:
            self.rotation=None
            return None,'unavailable'
        for i in range(first,last):
            dt=min(t,self.times[i+1])-max(previous,self.times[i])
            if dt>0:
                self.rotation=self.rotation@Rotation.from_rotvec(self.gyro[i]*dt).as_matrix()
        if visual is None:
            return self.rotation.copy(),'gyro_only_position_invalid'
        innovation=Rotation.from_matrix(self.rotation.T@visual).as_rotvec()
        if np.linalg.norm(innovation)>np.deg2rad(30):
            return self._anchor(t,map_id,visual,stationary_gravity_body,'visual_reanchor_jump')
        dt=t-previous
        self.rotation=self.rotation@Rotation.from_rotvec(innovation*(-math.expm1(-dt/self.visual_tau_s))).as_matrix()
        if self.gravity is None and stationary_gravity_body is not None:
            self.gravity=visual@stationary_gravity_body
            self.gravity/=np.linalg.norm(self.gravity)
        sample=self.acc[max(first,last-1)]
        # norm gate도 선형 가속도를 완전히 구분하지는 못한다. 실험용 피드백이다.
        if self.gravity is not None and abs(np.linalg.norm(sample)-9.80665)<.4:
            predicted=self.rotation@(sample/np.linalg.norm(sample))
            correction=np.cross(predicted,self.gravity)*(-math.expm1(-dt/self.gravity_tau_s))
            self.rotation=Rotation.from_rotvec(correction).as_matrix()@self.rotation
        self.last_visual_time=t
        return self.rotation.copy(),'fused_attitude'

    def _anchor(self, t: float, map_id: int, visual: np.ndarray,
                gravity_body: np.ndarray | None, status: str) -> tuple[np.ndarray,str]:
        self.rotation=visual.copy()
        self.last_visual_time,self.map_id=t,map_id
        self.gravity=None if gravity_body is None else visual@gravity_body
        if self.gravity is not None:
            self.gravity/=np.linalg.norm(self.gravity)
        return self.rotation.copy(),status


def stationary_gravity(data: dict, t: float) -> np.ndarray | None:
    """현재까지0.5초의 낮은 회전·가속도 변화 구간에서만 중력 기준 후보를 만든다."""
    times=data['imu_times_s']
    start,end=np.searchsorted(times,[t-.5,t],side='right')
    gyro,acc=data['gyro'][start:end],data['acc'][start:end]
    if (end-start<15 or times[end-1]-times[start]<.4
            or np.max(np.linalg.norm(gyro,axis=1))>.04
            or np.max(np.abs(np.linalg.norm(acc,axis=1)-9.80665))>.3
            or np.max(np.std(acc,axis=0))>.15):
        return None
    mean=np.mean(acc,axis=0)
    return mean/np.linalg.norm(mean)


def tilt(rotation: np.ndarray, gravity: np.ndarray | None) -> tuple[float,float]:
    """map별 초기 중력 기준에 대한 body roll·pitch를 반환한다."""
    if gravity is None:
        return float('nan'),float('nan')
    x,y,z=rotation.T@gravity
    return math.degrees(math.atan2(y,z)),math.degrees(math.atan2(-x,math.hypot(y,z)))


def run(run_dir: Path, output: Path | None = None, correction_s: float | None = None,
        segment: int | None = None) -> dict:
    """원본 기록의 위치를 보존하고 비교 CSV·그림을 새 디렉터리에 저장한다."""
    source=Path(run_dir).resolve()
    calibration=validate_imu_provenance(source)
    if calibration['frame']!='rover_body':
        raise ValueError('body-axis corrected IMU required')
    data=aligned_inputs(source,correction_s=correction_s,segment=segment)
    stereo=json.loads((source/'capture_calibration.json').read_text())['stereo']
    camera_from_body=camera_from_urdf(source/'robot.urdf',stereo)[:3,:3]
    folder=Path(output).resolve() if output else source/'attitude_fusion'
    if folder==source or folder in source.parents:
        raise ValueError('output must not overlap source directory')
    folder.mkdir(parents=True,exist_ok=False)
    atomic_json(folder/'alignment.json',data['report'])
    filt=AttitudeFilter(data['imu_times_s'],data['gyro'],data['acc'])
    with (source/'orb_live/live_poses.csv').open() as stream:
        original=list(csv.DictReader(stream))
    native_timings=source/'orb_inertial/timings.csv'
    native_rows={}
    if native_timings.is_file():
        with native_timings.open() as stream:
            native_rows={int(row['stamp_ns']):row for row in csv.DictReader(stream)}
    records=[]
    previous_valid=None
    for row in original:
        stamp,state,map_id=int(row['stamp_ns']),int(row['state']),int(row['map_id'])
        if stamp not in data['camera_times_s']:
            continue
        t=data['camera_times_s'][stamp]
        if not data['imu_times_s'][0]<=t<=data['imu_times_s'][-1]:
            continue
        started=time.perf_counter_ns()
        visual_valid=state==2 and map_id>=0
        pose=np.array([[float(row[f'm{i}{j}']) for j in range(4)] for i in range(4)]) if visual_valid else None
        visual=pose[:3,:3]@camera_from_body if visual_valid else None
        if (visual_valid and previous_valid is not None and map_id==previous_valid[0]
                and np.linalg.norm(pose[:3,3]-previous_valid[1])>.75):
            # map ID가 같은 loop correction·재시작 좌표 점프도 별도 재기준 설정한다.
            filt.rotation=None
        rotation,status=filt.update(t,map_id,visual,stationary_gravity(data,t))
        if visual_valid:
            previous_valid=(map_id,pose[:3,3].copy())
        fused=np.full((4,4),np.nan)
        if rotation is not None:
            fused[:3,:3]=rotation@camera_from_body.T
            fused[3]=[0.,0.,0.,1.]
            if visual_valid:
                fused[:3,3]=pose[:3,3]
        correction=float(np.degrees(Rotation.from_matrix(visual.T@rotation).magnitude())) if visual_valid and rotation is not None else float('nan')
        raw_tilt=tilt(visual,filt.gravity) if visual_valid else (float('nan'),)*2
        fused_tilt=tilt(rotation,filt.gravity) if rotation is not None else (float('nan'),)*2
        record={'stamp_ns':stamp,'time_s':t,'map_id':map_id,
                'orientation_map_id':filt.map_id if rotation is not None else -1,'orb_state':state,
                'position_valid':int(visual_valid),'orientation_valid':int(rotation is not None),'status':status,
                'gravity_reference_available':int(filt.gravity is not None),
                'orb_roll_deg':raw_tilt[0],'orb_pitch_deg':raw_tilt[1],
                'fused_roll_deg':fused_tilt[0],'fused_pitch_deg':fused_tilt[1],
                'orientation_correction_deg':correction,'filter_process_ms':(time.perf_counter_ns()-started)/1e6}
        native_row=native_rows.get(stamp,{})
        record['native_inertial_state']=int(native_row.get('state',-1))
        record['native_inertial_initialized']=int(native_row.get('imu_initialized',-1))
        record.update({f'm{i}{j}':fused[i,j] for i in range(4) for j in range(4)})
        records.append(record)
    if not records:
        raise ValueError('no overlapping ORB and IMU frames')
    with (folder/'poses.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]))
        writer.writeheader();writer.writerows(records)
    elapsed=np.asarray([r['filter_process_ms'] for r in records])
    summary={'status':'experimental_complete','source':str(source),'raw_preserved':True,
             'algorithm':'SO(3) complementary attitude filter; ORB position unchanged',
             'orb_internal_imu_enabled':False,'imu_clock_verified':False,'extrinsic_verified':False,
             'accuracy_evaluated':False,'position_integrated_from_imu':False,'frames':len(records),
             'visual_tau_s':.5,'gravity_tau_s':2.,'gyro_only_max_s':.3,
             'gravity_reference':'per-map stationary IMU mapped through ORB body rotation; no absolute world yaw',
             'status_counts':dict(Counter(r['status'] for r in records)),
             'position_valid_fraction':sum(r['position_valid'] for r in records)/len(records),
             'orientation_valid_fraction':sum(r['orientation_valid'] for r in records)/len(records),
             'filter_process_ms_p50_p95':np.percentile(elapsed,[50,95]).tolist(),
             'input_sha256':{**data['report']['input_sha256'],
                             'robot.urdf':file_sha256(source/'robot.urdf'),
                             'imu_manifest.json':file_sha256(source/'imu_manifest.json')},
             'comparison':{'stereo_live_input':{'frames':len(records),'position_valid_fraction':sum(r['position_valid'] for r in records)/len(records)},
                           'external_attitude':{'position_valid_fraction':sum(r['position_valid'] for r in records)/len(records)},
                           'scope':'different native runs/map frames cannot establish accuracy gains'}}
    native=source/'orb_inertial/summary.json'
    if native.is_file():
        prior=json.loads(native.read_text())
        summary['comparison']['orb_inertial_existing_run']={key:prior.get(key) for key in (
            'frames','tracking_ok_fraction','imu_initialized_fraction','last_tracked_frame_imu_initialized',
            'imu_clock_verified','extrinsic_verified')}
        summary['input_sha256']['orb_inertial/summary.json']=file_sha256(native)
    if native_timings.is_file():
        summary['input_sha256']['orb_inertial/timings.csv']=file_sha256(native_timings)
    atomic_json(folder/'summary.json',summary)
    plot(records,folder/'comparison.png')
    return summary


def plot(records: list[dict], path: Path) -> None:
    """map 경계를 끊어 표시하며 자세 차이를 정확도 향상으로 해석하지 않는다."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    x=np.asarray([r['time_s'] for r in records])
    cuts=np.r_[False,np.diff([r['map_id'] for r in records])!=0]
    cuts|=np.asarray([r['status'].startswith('visual_reanchor') for r in records])
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
    for ax,axis in zip(axes[:2],('roll','pitch')):
        for prefix in ('orb','fused'):
            y=np.asarray([r[f'{prefix}_{axis}_deg'] for r in records]);y[cuts]=np.nan
            ax.plot(x,y,label=prefix,linewidth=.9)
        ax.set_ylabel(f'{axis} (deg)');ax.legend();ax.grid(alpha=.25)
    axes[2].plot(x,[r['orientation_correction_deg'] for r in records],label='orientation correction (not error)')
    axes[2].set_ylabel('deg');axes[2].set_xlabel('aligned camera time (s)');axes[2].legend();axes[2].grid(alpha=.25)
    availability=[('Recorded live ORB position',[r['position_valid'] for r in records]),
                  ('External attitude',[r['orientation_valid'] for r in records]),
                  ('Existing offline IMU initialized',[r['native_inertial_initialized'] for r in records])]
    for index,(_,values) in enumerate(availability):
        y=np.asarray(values,dtype=float);y[y<0]=np.nan
        axes[3].fill_between(x,index,index+.8*y,step='post',alpha=.65)
    axes[3].set_yticks(np.arange(3)+.4)
    axes[3].set_yticklabels([label for label,_ in availability],fontsize=8)
    axes[3].set_xlabel('aligned camera time (s)');axes[3].grid(alpha=.25)
    fig.suptitle('Experimental external attitude fusion: ORB position preserved; clocks/extrinsics unverified')
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)


def main() -> None:
    parser=argparse.ArgumentParser(description='Experimental offline external ORB/IMU attitude fusion')
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--correction-s',type=float)
    parser.add_argument('--segment',type=int)
    args=parser.parse_args()
    print(json.dumps(run(args.run,args.output,args.correction_s,args.segment),indent=2))


if __name__=='__main__':
    main()
