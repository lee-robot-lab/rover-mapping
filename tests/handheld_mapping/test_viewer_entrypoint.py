# ================================================================
# test_viewer_entrypoint.py
# 설명: viewer.py가 스크립트로 실행될 때 main() 호출 전에 필요한 함수가 모두 정의되는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_viewer_entrypoint.py
# ================================================================
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_viewer_runs_as_script_and_reaches_argument_parsing():
    result = subprocess.run([sys.executable, '-m', 'experiments.handheld_mapping.viewer', '--help'],
                            cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr


def test_display_geometry_is_defined_before_main_guard():
    source = (ROOT/'experiments/handheld_mapping/viewer.py').read_text()
    assert source.index('def display_geometry') < source.index("if __name__ == '__main__'")
