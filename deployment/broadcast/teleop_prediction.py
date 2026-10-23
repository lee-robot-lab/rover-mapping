# ================================================================
# teleop_prediction.py
# 설명: 현재 수동주행 방향을 임시 목표로 삼아 지도 MPPI와 미교정 slip을 표시한다.
# 사용법: from deployment.broadcast.teleop_prediction import TeleopPrediction
# ================================================================
from __future__ import annotations

import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

import numpy as np
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ksrc/runtime'))
from ksrc_runtime.handheld_mapping.terrain_mppi import TerrainMppi


class TeleopPrediction:
    """제어 출력 없이 배포 MPPI의 안전한 후보와 방향별 지형 slip을 반환한다."""

    def __init__(self, site: Any) -> None:
        self.site = site
        self.planner = TerrainMppi(site, max_speed=.15, max_yaw_rate=.3)
        self.allowed = site.observed.copy()
        height = site.height_m.copy()
        nearest = distance_transform_edt(~np.isfinite(height), return_distances=False,
                                         return_indices=True)
        self.gy, self.gx = np.gradient(height[tuple(nearest)], site.resolution)
        self.lock = threading.Lock()
        self.last_key: tuple | None = None
        self.last_result: dict | None = None

    def predict(self, pose: dict) -> dict | None:
        """수신 pose를 변경하지 않고 현재 위치에서 새로 검증된 표시 궤적만 반환한다."""
        values = tuple(float(pose[key]) for key in ('x', 'y', 'yaw', 'v', 'w'))
        x, y, yaw, v, w = values
        lateral = float(pose.get('vy_m_s', 0.))
        translation = math.hypot(v, lateral)
        speed = float(pose.get('speed_m_s', translation))
        if not all(math.isfinite(value) for value in (*values, lateral, speed)) or translation < .01:
            return None
        key = (*values, lateral, speed)
        with self.lock:
            if key == self.last_key:
                return self.last_result
            started = time.perf_counter()
            xy = np.array([x, y])
            # 횡이동은 실제 이동 방향의 등가 공간 예측으로 바꾸며 수신 차체 yaw는 유지한다.
            heading = yaw + math.atan2(lateral, v) if lateral else yaw
            reverse = v < 0. and lateral == 0.
            # 최대 30cm/3초의 현재 주행 원호이며 임무 경로·waypoint는 참조하지 않는다.
            velocity = float(np.clip(translation if lateral else v, -.15, .15))
            rate = float(np.clip(w, -.3, .3))
            duration = min(3.,.3 / abs(velocity))
            if abs(rate) < 1e-8:
                target = xy + velocity*duration * np.array([math.cos(heading), math.sin(heading)])
            else:
                angle = heading + rate * duration
                target = xy + velocity/rate * np.array([math.sin(angle)-math.sin(heading),
                                                        math.cos(heading)-math.cos(angle)])
            self.planner.controls[:] = [velocity, rate, 0.]
            self.planner._reverse = reverse
            result = self.planner.plan(xy, heading, target, self.allowed, 35., reverse=reverse)
            prediction = None
            if result['valid'] and result['candidate_trajectories']:
                paths = np.asarray(result['candidate_trajectories'], float)
                selected = np.asarray(result['selected_trajectory'], float)
                cells = self.site.cells(paths.reshape(-1, 2)).reshape(paths.shape)
                # 색상은 지도 경사 평균이며 planner의 MPPI 목적함수 값이 아니다.
                costs = self.site.slope_deg[cells[..., 1], cells[..., 0]].mean(axis=1)
                cx, cy = self.site.cells([xy])[0]
                angle = heading + (math.pi if reverse else 0.)
                grade = float(self.gx[cy, cx]*math.cos(angle) + self.gy[cy, cx]*math.sin(angle))
                # build_mission_cache.slip_estimate와 동일한 미교정 방향별 견인 모델.
                demand = np.clip((max(grade, 0.)+.03)/.6, 0., .995)
                slip = float(np.clip(-.15*np.log1p(-demand), 0., .95))
                if all(np.isfinite(array).all() for array in (paths, selected, costs, [slip])):
                    prediction = dict(candidates=paths.tolist(), selected=selected.tolist(),
                                      costs=costs.tolist(), slip=slip,
                                      simulated_compute_ms=(time.perf_counter()-started)*1000.,
                                      lookup_distance_m=0., source='teleop_model')
            self.last_key, self.last_result = key, prediction
            return prediction
