# ================================================================
# sensor_safety.py
# 설명: 같은 clock의 pose·IMU 상태로 감속·정지·복구 대기를 결정하는 모터 없는 안전 정책.
# 사용법: SensorSafety(require_imu=True).status(now_s)
# ================================================================
from __future__ import annotations
import math


class SensorSafety:
    """센서 callback과 별개로 주기 검사할 수 있는 freshness·tracking motion gate."""

    def __init__(self, *, require_imu: bool=False, pose_timeout_s: float=.5,
                 imu_timeout_s: float=.1, recovery_s: float=.5, slow_scale: float=.3) -> None:
        if (not all(math.isfinite(v) and v>0 for v in (pose_timeout_s,imu_timeout_s,recovery_s,slow_scale))
                or slow_scale>1):raise ValueError('positive finite limits and scale <=1 required')
        self.require_imu=require_imu;self.pose_timeout_s=pose_timeout_s;self.imu_timeout_s=imu_timeout_s
        self.recovery_s=recovery_s;self.slow_scale=slow_scale
        self.pose_stamp=None;self.imu_stamp=None;self.pose_fault=None;self.imu_fault=None
        self.quality='good';self.good_since=None;self.good_count=0;self.armed=False;self.last_check=None

    def _stop(self, reason: str) -> dict:
        self.armed=False;self.good_since=None;self.good_count=0
        return {'mode':'stop','reason':reason,'scale':0.,'valid_until_s':None}

    def inhibit(self, reason: str) -> None:
        """상위 pose/frame 검사 실패에도 재개 안정 시간을 다시 요구한다."""
        self._stop(reason)

    def observe_pose(self, stamp_s: float, now_s: float, valid: bool, quality: str='good') -> None:
        """추적 상태와 실제 새 pose만 수용한다. 중복 pose는 복구 근거로 세지 않는다."""
        try:
            stamp,now=float(stamp_s),float(now_s)
            if not math.isfinite(stamp) or not math.isfinite(now) or stamp<0 or stamp>now+.02:raise ValueError()
            if self.pose_stamp is not None and stamp<=self.pose_stamp:raise ValueError()
        except (ValueError,TypeError,OverflowError):self.pose_fault='invalid_pose_time';self._stop(self.pose_fault);return
        if self.pose_stamp is not None and stamp-self.pose_stamp>self.pose_timeout_s:self._stop('pose_gap')
        self.pose_stamp=stamp;self.quality=quality
        self.pose_fault='tracking_lost' if valid is not True else ('invalid_tracking_quality' if quality not in ('good','degraded') else None)
        if self.pose_fault:self._stop(self.pose_fault)
        elif quality=='good':
            if self.good_since is None:self.good_since=stamp
            self.good_count+=1
        elif not self.armed:self.good_since=None;self.good_count=0

    def observe_imu(self, stamp_s: float, now_s: float, valid: bool=True) -> None:
        """IMU callback을 독립 수용한다. 같은 stamp 재전달은 freshness를 연장하지 않는다."""
        try:
            stamp,now=float(stamp_s),float(now_s)
            if (not math.isfinite(stamp) or not math.isfinite(now) or stamp<0 or stamp>now+.02
                    or (self.imu_stamp is not None and stamp<self.imu_stamp)):raise ValueError()
        except (ValueError,TypeError,OverflowError):
            self.imu_fault='invalid_imu_time'
            if self.require_imu:self._stop(self.imu_fault)
            return
        if self.require_imu and self.imu_stamp is not None and stamp-self.imu_stamp>self.imu_timeout_s+1e-9:self._stop('imu_gap')
        self.imu_stamp=stamp;self.imu_fault=None if valid is True else 'imu_invalid'
        if self.imu_fault and self.require_imu:self._stop(self.imu_fault)

    def status(self, now_s: float) -> dict:
        """새 관측이 없어도 호출한다. stale/손실은 정지, 복구는 정상 pose 3개와 안정 시간을 요구한다."""
        try:
            now=float(now_s)
            if not math.isfinite(now) or (self.last_check is not None and now<self.last_check):raise ValueError()
        except (ValueError,TypeError,OverflowError):return self._stop('invalid_clock')
        self.last_check=now
        if self.pose_stamp is None:return self._stop('pose_missing')
        if self.pose_fault:return self._stop(self.pose_fault)
        if now<self.pose_stamp-.02 or now-self.pose_stamp>self.pose_timeout_s+1e-9:return self._stop('pose_stale')
        if self.require_imu:
            if self.imu_stamp is None:return self._stop('imu_missing')
            if self.imu_fault:return self._stop(self.imu_fault)
            if now<self.imu_stamp-.02 or now-self.imu_stamp>self.imu_timeout_s+1e-9:return self._stop('imu_stale')
        deadline=self.pose_stamp+self.pose_timeout_s
        if self.require_imu:deadline=min(deadline,self.imu_stamp+self.imu_timeout_s)
        if not self.armed:
            if (self.quality!='good' or self.good_since is None or self.good_count<3
                    or self.pose_stamp-self.good_since<self.recovery_s-1e-9):
                return {'mode':'stop','reason':'recovering','scale':0.,'valid_until_s':deadline}
            self.armed=True
        degraded=self.quality=='degraded'
        return {'mode':'slow' if degraded else 'normal','reason':'tracking_degraded' if degraded else 'sensors_ok',
                'scale':self.slow_scale if degraded else 1.,'valid_until_s':deadline}
