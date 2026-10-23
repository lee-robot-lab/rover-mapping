# ================================================================
# core.py
# 설명: 중계용 UDP 상태 검증, 출발 좌표 정렬, RUN 기록과 사전 예측 조회를 담당한다.
# 사용법: from experiments.broadcast_demo.core import Session, Lookup
# ================================================================
from __future__ import annotations

import json
import math
import threading
import time
from typing import Any
import numpy as np

FIELDS = ('timestamp', 'x', 'y', 'yaw', 'v', 'w')


def decode_packet(data: bytes) -> dict:
    try:
        value = json.loads(data)
        if not isinstance(value, dict) or type(value['seq']) is not int or value['seq'] < 0:
            raise ValueError('seq must be a nonnegative integer')
        result = {key: float(value[key]) for key in FIELDS}
        if not all(math.isfinite(x) for x in result.values()):
            raise ValueError('nonfinite telemetry')
        result['seq'] = value['seq']
        if 'vy_m_s' in value:
            lateral=float(value['vy_m_s'])
            if not math.isfinite(lateral):raise ValueError('invalid lateral speed')
            result['vy_m_s']=lateral
        if 'speed_m_s' in value:
            speed=float(value['speed_m_s'])
            if not math.isfinite(speed) or speed<0:raise ValueError('invalid ground speed')
            result['speed_m_s']=speed
        if value.get('site_identity') is not None:
            identity=value['site_identity']
            if not isinstance(identity,str) or len(identity)!=64 or any(c not in '0123456789abcdef' for c in identity):
                raise ValueError('invalid site identity')
            result['site_identity']=identity
        if 'operator' in value:
            op=value['operator']
            if (not isinstance(op,dict) or type(op.get('active')) is not bool
                    or type(op.get('run')) is not int or op['run'] not in (0,1,2,3,4)
                    or type(op.get('anchor_count')) is not int or op['anchor_count']<0):
                raise ValueError('invalid operator state')
            result['operator']={key:op[key] for key in ('active','run','anchor_count')}
            if 'anchor_id' in op:
                if type(op['anchor_id']) is not int or op['anchor_id'] not in (1,2,3,4):raise ValueError('invalid anchor preset')
                result['operator']['anchor_id']=op['anchor_id']
            if 'stage' in op:
                if not isinstance(op['stage'],str) or len(op['stage'])>64:raise ValueError('invalid stage')
                result['operator']['stage']=op['stage']
            result['operator']['done']=bool(op.get('done'))
        if 'session_id' in value:
            if not isinstance(value['session_id'], str) or not 1 <= len(value['session_id']) <= 128:
                raise ValueError('session_id must be a short nonempty string')
            result['session_id'] = value['session_id']
        return result
    except (KeyError, TypeError, OverflowError, json.JSONDecodeError) as exc:
        raise ValueError('invalid telemetry packet') from exc


def wrap(angle: float | np.ndarray) -> Any:
    return np.arctan2(np.sin(angle), np.cos(angle))


class Lookup:
    """위치·방향·속도·회전율이 저장 범위 안인 경우에만 사전 예측 인덱스를 반환한다."""

    def __init__(self, states: np.ndarray, runs: np.ndarray, stages: np.ndarray | None = None) -> None:
        self.states, self.runs = states, runs
        self.stages = stages

    def find(self, state: list[float], run: int, stage: str | None = None) -> int | None:
        delta = self.states - np.asarray(state)
        delta[:, 2] = wrap(delta[:, 2])
        distance = np.linalg.norm(delta[:, :2], axis=1)
        valid = ((self.runs == run) & (distance <= .04) & (np.abs(delta[:, 2]) <= np.deg2rad(5))
                 & (np.abs(delta[:, 3]) <= .08) & (np.abs(delta[:, 4]) <= .3))
        if self.stages is not None:
            valid &= self.stages == stage
        if not valid.any():
            return None
        score = (delta / [.2, .2, np.deg2rad(25), .08, .3])**2
        return int(np.argmin(np.where(valid, score.sum(axis=1), np.inf)))


class Session:
    """최신 pose를 정렬하고 네 RUN 및 좌표 단절을 구분한 TELEOP 궤적을 기록한다."""

    def __init__(self, start: tuple[float, float, float]) -> None:
        self.start_pose = start
        self.lock = threading.RLock()
        self.raw = None
        self.received = None
        self.run = 0
        self.rotation = 0.
        self.offset = np.zeros(2)
        self.tracks = [[], [], [], []]
        self.distances = [0., 0., 0., 0.]
        self.started = None
        self.stopped = None
        self.active = False
        self.accepted = self.rejected = 0
        self.retired_sessions = set()
        self.teleop_tracks=[]
        self.teleop_distance_m=0.
        self.teleop_continuous=False

    def break_teleop(self) -> None:
        """다음 수동 위치를 새 구간으로 시작해 좌표 변경·AUTO 구간을 연결하지 않는다."""
        self.teleop_continuous=False

    def record_teleop(self) -> None:
        """수동 이동만 누적하고 정지 중 중복 위치는 생략한다."""
        with self.lock:
            p=self.pose()
            if p is None:return
            point=[p['x'],p['y'],p.get('speed_m_s',math.hypot(p['v'],p.get('vy_m_s',0.))),p['timestamp']]
            if not self.teleop_continuous:
                self.teleop_tracks.append([point]);self.teleop_continuous=True
                return
            segment=self.teleop_tracks[-1]
            distance=math.hypot(point[0]-segment[-1][0],point[1]-segment[-1][1])
            if distance<.001:return
            self.teleop_distance_m+=distance
            segment.append(point)

    def align_to_start(self, align: bool = True) -> None:
        """기록 모드를 바꾸지 않고 현재 pose를 출발 기준에 정렬한다."""
        self.rotation = float(wrap(self.start_pose[2]-self.raw['yaw'])) if align else 0.
        c,s=math.cos(self.rotation),math.sin(self.rotation)
        self.offset = (np.asarray(self.start_pose[:2])-np.array([[c,-s],[s,c]]) @ [self.raw['x'],self.raw['y']]
                       if align else np.zeros(2))

    def pose(self) -> dict | None:
        if self.raw is None:
            return None
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        xy = np.array([[c, -s], [s, c]]) @ [self.raw['x'], self.raw['y']] + self.offset
        return dict(self.raw, x=float(xy[0]), y=float(xy[1]), yaw=float(wrap(self.raw['yaw']+self.rotation)))

    def ingest(self, packet: dict, now: float | None = None, record: bool = True) -> bool:
        with self.lock:
            now = time.monotonic() if now is None else now
            source = packet.get('session_id', '')
            old_source = None if self.raw is None else self.raw.get('session_id', '')
            if source in self.retired_sessions:
                self.rejected += 1
                return False
            changed = self.raw is not None and source != old_source
            if self.raw is not None and not changed and packet['seq'] <= self.raw['seq']:
                self.rejected += 1
                return False
            if changed:
                self.retired_sessions.add(old_source)
                self.stop(now)
            if self.received is not None and now-self.received>.5:self.break_teleop()
            self.raw, self.received = packet, now
            self.accepted += 1
            if self.active and record:
                p = self.pose()
                point = [p['x'], p['y'], p['v'], p['timestamp']]
                track = self.tracks[self.run-1]
                if track:
                    self.distances[self.run-1] += math.hypot(point[0]-track[-1][0], point[1]-track[-1][1])
                track.append(point)
            return True

    def start(self, run: int, now: float | None = None, require_history: bool = True, align: bool = True) -> None:
        with self.lock:
            now = time.monotonic() if now is None else now
            if run not in (1,2,3,4) or self.received is None or now-self.received > .5:
                raise ValueError('fresh pose required to start RUN 1..4')
            if run == 2 and require_history and not self.tracks[0]:
                raise ValueError('RUN 1 record required')
            self.align_to_start(align)
            self.break_teleop()
            if run == 1:
                self.tracks, self.distances = [[], [], [], []], [0., 0., 0., 0.]
            else:
                self.tracks[run-1], self.distances[run-1] = [], 0.
            self.run, self.started, self.active = run, now, True
            self.stopped = None
            p = self.pose()
            self.tracks[run-1].append([p['x'], p['y'], p['v'], p['timestamp']])

    def stop(self, now: float | None = None) -> None:
        with self.lock:
            self.break_teleop()
            if self.active:
                self.stopped = time.monotonic() if now is None else now
            self.active = False

    def snapshot(self, now: float | None = None) -> dict:
        with self.lock:
            now = time.monotonic() if now is None else now
            age = None if self.received is None else max(0., now-self.received)
            return dict(pose=self.pose(), run=self.run, active=self.active, age_s=age,
                        stale=age is None or age > .5, tracks=[list(t) for t in self.tracks],
                        distances=list(self.distances), elapsed_s=0. if self.started is None else
                        (now if self.stopped is None else self.stopped)-self.started,
                        accepted=self.accepted, rejected=self.rejected,
                        teleop_tracks=[list(segment) for segment in self.teleop_tracks],
                        teleop_distance_m=self.teleop_distance_m)
