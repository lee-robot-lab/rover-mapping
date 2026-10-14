# ================================================================
# test_compare.py
# 설명: 점선 비교에서 추적 실패·지도 분리·stereo 누락을 숨기지 않는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_compare.py -q
# ================================================================
import json
import pytest


def test_tracking_failures_count_in_success_fraction(tmp_path):
    from experiments.handheld_mapping.compare import summarize_timings
    path = tmp_path / "timings.csv"
    path.write_text("stamp_ns,state,map_id,process_ms\n100,2,0,10\n200,3,0,30\n300,2,7,20\n")
    summary = summarize_timings(path)
    assert summary["tracking_ok_fraction"] == pytest.approx(2/3)
    assert summary["map_ids"] == [0, 7]
    assert summary["process_ms_median"] == 20
    assert summary["gt_accuracy_evaluated"] is False


def test_pairing_requires_exact_stereo_stamp(tmp_path):
    from experiments.handheld_mapping.compare import stereo_pairs
    rows = [{"stream": "left", "stamp_ns": 100, "path": "raw/left/100.png"},
            {"stream": "right", "stamp_ns": 101, "path": "raw/right/101.png"},
            {"stream": "left", "stamp_ns": 200, "path": "raw/left/200.png"},
            {"stream": "right", "stamp_ns": 200, "path": "raw/right/200.png"}]
    (tmp_path / "frames.jsonl").write_text("\n".join(json.dumps(row) for row in rows))
    pairs = stereo_pairs(tmp_path)
    assert len(pairs) == 1
    assert pairs[0][0] == 200


def test_runner_loads_without_inherited_library_path():
    import subprocess
    from experiments.handheld_mapping.compare import ROOT, runner_environment
    binary = ROOT / "outputs/handheld_mapping/orbline_build/rover_orbline"
    if not binary.exists():
        pytest.skip("optional native runner is not built")
    result = subprocess.run([str(binary)], env=runner_environment(False), capture_output=True, text=True)
    assert result.returncode == 2, result.stderr
    assert "ORBvoc" in result.stderr


def test_timing_summary_exact_quantiles_without_materializing_csv_rows(tmp_path, monkeypatch):
    import numpy as np
    from experiments.handheld_mapping import compare
    path = tmp_path/'timings.csv'
    values = [float(i % 31) for i in range(1001)]
    path.write_text('stamp_ns,state,map_id,process_ms\n'+''.join(
        f'{i},2,0,{v}\n' for i, v in enumerate(values)))
    expected = np.percentile(values, [50, 95])
    def refuse(*args, **kwargs):
        raise AssertionError('full timing materialization')
    monkeypatch.setattr(compare.np, 'percentile', refuse)
    result = compare.summarize_timings(path)
    assert result['process_ms_median'] == pytest.approx(expected[0])
    assert result['process_ms_p95'] == pytest.approx(expected[1])
