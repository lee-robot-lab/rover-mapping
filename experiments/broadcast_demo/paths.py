# ================================================================
# paths.py
# 설명: 배포용 중계 데이터와 저장소 루트의 위치를 정의한다.
# 사용법: from experiments.broadcast_demo.paths import ROOT, DEFAULT_OUT
# ================================================================
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT/'deployment/broadcast/data'
