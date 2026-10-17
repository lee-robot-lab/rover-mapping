# ================================================================
# carrot_follower.py
# 설명: 조밀 경로 위 lookahead 당근 점을 고르고, 급한 꼭짓점은 넘지 않으며 꼭짓점·끝 근처 속도 배율을 준다.
# 사용법: p=CarrotPath(points); p.target(index, .35); p.speed_factor(index, .3, .35)
# ================================================================
from __future__ import annotations

import numpy as np


class CarrotPath:
    """경로 점열에서 진행 위치 앞의 당근 점과 꼭짓점 감속 배율을 계산한다."""

    def __init__(self, points: np.ndarray, corner_deg: float=35., window_m: float=.15) -> None:
        self.points=np.asarray(points,float)
        self.arcs=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(self.points,axis=0),axis=1))]
        self.corners=self._find_corners(np.radians(corner_deg),window_m)

    def _find_corners(self, threshold: float, window: float) -> list[int]:
        """window 길이 현의 방향 변화가 threshold를 넘는 연속 구간마다 최대 지점 하나를 꼭짓점으로 쓴다.
        격자 양자화 지그재그는 현 평균에서 사라져 꼭짓점으로 잡히지 않는다."""
        n=len(self.points);turn=np.zeros(n)
        for i in range(1,n-1):
            back=self.points[int(np.searchsorted(self.arcs,self.arcs[i]-window))]
            ahead=self.points[min(int(np.searchsorted(self.arcs,self.arcs[i]+window)),n-1)]
            a,b=self.points[i]-back,ahead-self.points[i]
            if np.linalg.norm(a)<window/2 or np.linalg.norm(b)<window/2:continue
            turn[i]=abs(np.arctan2(a[0]*b[1]-a[1]*b[0],a@b))
        corners,i=[],1
        while i<n-1:
            if turn[i]<=threshold:i+=1;continue
            j=i
            while j<n-1 and turn[j]>threshold:j+=1
            corners.append(i+int(np.argmax(turn[i:j])));i=j
        return corners

    def _next_stop_arc(self, arc: float) -> float:
        ahead=[self.arcs[i] for i in self.corners if self.arcs[i]>arc+1e-9]
        return min(ahead[0] if ahead else self.arcs[-1],self.arcs[-1])

    def target(self, index: int, lookahead: float) -> np.ndarray:
        """index 점의 호 길이에서 lookahead 앞, 다만 다음 급한 꼭짓점과 경로 끝을 넘지 않는 점."""
        arc=self.arcs[min(index,len(self.points)-1)]
        goal=min(arc+lookahead,self._next_stop_arc(arc))
        return np.array([np.interp(goal,self.arcs,self.points[:,0]),np.interp(goal,self.arcs,self.points[:,1])])

    def speed_factor(self, index: int, brake_m: float, floor: float) -> float:
        """다음 꼭짓점·끝까지 남은 거리가 brake_m 안이면 floor까지 선형 감속한 배율."""
        arc=self.arcs[min(index,len(self.points)-1)]
        remaining=self._next_stop_arc(arc)-arc
        return float(np.clip(floor+(1.-floor)*remaining/brake_m,floor,1.))
