# ================================================================
# test_session.py
# 설명: 실제 subprocess 기반으로 촬영·종료·TSDF와 비정상 전송 처리를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_session.py -q
# ================================================================
import json
import sys
import time

import pytest


def wait_state(session, states, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = session.snapshot()
        if status["state"] in states:
            return status
        time.sleep(.03)
    raise AssertionError(session.snapshot())


def test_start_stop_records_originals_and_builds_mesh(tmp_path):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    session.start({"output_root": str(tmp_path), "demo": True})
    wait_state(session, {"recording"})
    with pytest.raises(RuntimeError):
        session.start({"output_root": str(tmp_path), "demo": True})
    time.sleep(.5)
    assert 10 < session.snapshot()["left_receipt_fps"] < 20
    session.stop()
    result = wait_state(session, {"complete", "partial", "failed"})
    assert result["state"] == "complete", result
    assert result["left_receipt_fps"] == 0
    from pathlib import Path
    run = Path(result["run_dir"])
    assert list((run / "raw/left").glob("*.png"))
    assert list((run / "raw/right").glob("*.png"))
    assert list((run / "raw/depth").glob("*.png"))
    assert (run / "dense/map_0_mesh.ply").stat().st_size > 100
    assert json.loads((run / "session.json").read_text())["source"] == "synthetic_demo"


def test_early_stop_does_not_hang_startup(tmp_path):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    session.start({"output_root": str(tmp_path), "demo": True})
    session.stop()
    result = wait_state(session, {"complete", "partial", "failed"})
    assert result["state"] == "failed"
    assert "error" in result


def test_partial_line_map_is_reported_without_losing_baseline(tmp_path, monkeypatch):
    from pathlib import Path
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import compare
    monkeypatch.setattr(compare, "run_orbline", lambda run: {"dense": {"status": "partial"}})
    session = Session()
    session.start({"output_root": str(tmp_path), "demo": True, "compare_lines": True})
    try:
        wait_state(session, {"recording"})
        time.sleep(.2)
        session.stop()
        result = wait_state(session, {"complete", "partial", "failed"})
        session.thread.join(timeout=5)
        assert result["state"] == "partial", result
        assert (Path(result["run_dir"]) / "dense/map_0_mesh.ply").is_file()
    finally:
        session.stop()
        session.thread.join(timeout=15)


def test_abrupt_worker_eof_is_failure(tmp_path):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    session.start({"output_root": str(tmp_path), "worker_command": [sys.executable, "-c", "pass"]})
    status = wait_state(session, {"failed", "complete"})
    assert status["state"] == "failed"
    assert "종료" in status["error"] or "finished" in status["error"]


def test_finished_elapsed_time_does_not_keep_increasing(tmp_path):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    session.start({"output_root": str(tmp_path), "demo": True})
    wait_state(session, {"recording"})
    time.sleep(.2)
    session.stop()
    result = wait_state(session, {"complete", "failed"})
    time.sleep(.15)
    assert session.snapshot()["elapsed_s"] == result["elapsed_s"]


def test_write_error_stops_owned_worker_but_leaves_other_process(tmp_path, monkeypatch):
    import subprocess
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping.storage import RunWriter
    original = RunWriter.accept
    def fail_on_image(self, header, payload):
        if header["kind"] == "image":
            raise OSError("disk failure fixture")
        return original(self, header, payload)
    monkeypatch.setattr(RunWriter, "accept", fail_on_image)
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        session = Session()
        session.start({"output_root": str(tmp_path), "demo": True})
        result = wait_state(session, {"failed"})
        session.thread.join(timeout=10)
        assert not session.thread.is_alive()
        assert session.process.poll() is not None
        assert other.poll() is None
        assert "disk failure fixture" in result["error"]
    finally:
        other.terminate()
        other.wait()


def test_previous_stop_watchdog_cannot_terminate_new_session(tmp_path):
    from experiments.handheld_mapping.controller import Session
    session = Session()
    session.start({"output_root": str(tmp_path), "demo": True})
    wait_state(session, {"recording"})
    previous = getattr(session, "generation", 0)
    time.sleep(.2)
    session.stop()
    wait_state(session, {"complete", "failed"})
    session.thread.join(timeout=5)
    session.start({"output_root": str(tmp_path), "demo": True})
    wait_state(session, {"recording"})
    try:
        session._stop_timeout(previous, time.monotonic()-1)
        time.sleep(.1)
        assert session.process.poll() is None
    finally:
        session.stop()
        session.thread.join(timeout=15)
