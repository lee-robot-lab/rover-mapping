# ================================================================
# controller.py
# 설명: 노트북 촬영 세션과 소유 subprocess의 정상 종료·저장·TSDF를 관리한다.
# 사용법: from experiments.handheld_mapping.controller import Session
# ================================================================
from datetime import datetime
from collections import deque
import io
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import threading
import time
import uuid

from experiments.handheld_mapping.protocol import read_packet
from experiments.handheld_mapping.storage import RunWriter, atomic_json

ROOT = Path(__file__).resolve().parents[2]
ACTIVE = {"starting", "recording", "stopping", "processing", "reconstructing", "comparing"}


def remote_command(host: str, run_id: str, sensors_only: bool = False, queue_mib: float = 4.) -> list[str]:
    if not re.fullmatch(r"[A-Za-z0-9_.@:-]+", host) or host.startswith("-"):
        raise ValueError("잘못된 SSH 연결 대상")
    if not math.isfinite(queue_mib) or not .25 <= queue_mib <= 64:
        raise ValueError("invalid queue MiB limit")
    remote = f"/dev/shm/rover_mapping_{run_id}"
    code = (
        "set -e; source /opt/ros/jazzy/setup.bash >&2; "
        "export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=67 RMW_IMPLEMENTATION=rmw_fastrtps_cpp RCUTILS_LOGGING_USE_STDOUT=0; "
        f"export ROS_LOG_DIR={shlex.quote(remote+'/logs')}; "
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/handheld_mapping/live/fastdds_stereo_local.xml; "
        "export LD_LIBRARY_PATH=/home/pi/handheld_mapping/runtime/install/lib:/home/pi/handheld_mapping/runtime/prefix/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}; "
        f"cd {shlex.quote(remote+'/code')}; "
        f"exec /usr/bin/python3 -m experiments.handheld_mapping.pi_worker --run-dir {shlex.quote(remote+'/run')}"
        f" --queue-mib {queue_mib}"+(" --sensors-only" if sensors_only else "")
    )
    return ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=3", host, shlex.join(["bash", "-c", code])]


def deploy(host: str, run_id: str) -> None:
    # 명령의 host 검증을 공유하며 사용자 셸 문자열을 실행하지 않는다.
    remote_command(host, run_id)
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w:gz") as tar:
        for path in (ROOT / "experiments/handheld_mapping").glob("*.py"):
            tar.add(path, arcname=f"experiments/handheld_mapping/{path.name}")
        for name in ("prepare_mapping_run.py",):
            tar.add(ROOT / "ros2_ws/src/orbslam3_ros2/scripts" / name, arcname=f"helpers/{name}")
    remote = f"/dev/shm/rover_mapping_{run_id}/code"
    command = f"mkdir -p {shlex.quote(remote)} && tar -xzf - -C {shlex.quote(remote)}"
    subprocess.run(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, command],
                   input=archive.getvalue(), check=True, timeout=20, stdout=subprocess.DEVNULL,
                   stderr=subprocess.PIPE)


class Session:
    """촬영·파일 기록·최종 지도 생성의 상태를 한 worker thread에서 관리한다."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.status = {"state": "idle", "message": "준비", "counts": {}, "run_dir": "", "dropped": 0}
        self.process: subprocess.Popen | None = None
        self.thread: threading.Thread | None = None
        self.stop_requested = threading.Event()
        self.preview = None
        self.started_at = 0.
        self.ended_at = 0.
        self.generation = 0
        self.left_receipts = deque(maxlen=30)
        self.live = None

    def live_snapshot(self) -> dict | None:
        with self.lock:
            mapper = self.live
            error = self.status.get('live_error')
        return mapper.snapshot() if mapper is not None else ({'error':error} if error else None)

    def snapshot(self) -> dict:
        with self.lock:
            result = {**self.status, "counts": dict(self.status.get("counts", {}))}
            result["elapsed_s"] = (self.ended_at or time.monotonic())-self.started_at if self.started_at else 0.
            receipts = self.left_receipts
            result["left_receipt_fps"] = ((len(receipts)-1)/(receipts[-1]-receipts[0])
                if len(receipts) >= 2 and result["state"] == "recording" and time.monotonic()-receipts[-1] < 1 else 0.)
            return result

    def update(self, **values) -> None:
        with self.lock:
            if values.get("state") in {"complete", "partial", "failed"} and not self.ended_at:
                self.ended_at = time.monotonic()
            self.status.update(values)

    def start(self, config: dict) -> None:
        with self.lock:
            if self.status["state"] in ACTIVE or (self.thread is not None and self.thread.is_alive()):
                raise RuntimeError("이미 실행 중입니다")
            self.stop_requested.clear()
            self.generation += 1
            self.process = None
            self.preview = None
            self.live = None
            self.left_receipts.clear()
            self.started_at = time.monotonic()
            self.ended_at = 0.
            self.status = {"state": "starting", "message": "연결·카메라 준비 중", "counts": {}, "run_dir": "", "dropped": 0}
            self.thread = threading.Thread(target=self._run, args=(dict(config),), daemon=False)
            self.thread.start()

    def stop(self) -> None:
        with self.lock:
            if self.status["state"] not in {"starting", "recording", "stopping"}:
                return
            first = not self.stop_requested.is_set()
            self.stop_requested.set()
            self.status.update(state="stopping", message="촬영 종료·최종 경로 저장 중")
            process = self.process
            if process is not None and process.poll() is None:
                try:
                    process.stdin.write(b"stop\n")
                    process.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
            if first:
                threading.Thread(target=self._stop_timeout, args=(self.generation, time.monotonic()+75), daemon=True).start()

    def _stop_timeout(self, generation: int, deadline: float) -> None:
        while time.monotonic() < deadline:
            with self.lock:
                if generation != self.generation or not self.stop_requested.is_set():
                    return
                process = self.process
                state = self.status["state"]
            if state not in {"starting", "recording", "stopping"}:
                return
            if process is not None and process.poll() is not None:
                return
            time.sleep(.2)
        with self.lock:
            if generation != self.generation or not self.stop_requested.is_set():
                return
            process = self.process
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)

    def _run(self, config: dict) -> None:
        writer = None
        run = None
        process = None
        log = None
        live = None
        try:
            root = Path(config["output_root"]).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(root).free < 2_000_000_000:
                raise OSError("노트북 여유 공간이 2GB 미만입니다")
            run_id = datetime.now().strftime("%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:6]
            run = root / run_id
            writer = RunWriter(run)
            self.update(run_dir=str(run), source="synthetic_demo" if config.get("demo") else "D435_Pi")
            atomic_json(run / "session.json", self.snapshot())
            if config.get("demo"):
                command = [sys.executable, "-u", "-m", "experiments.handheld_mapping.demo"]
            elif config.get("worker_command"):
                command = config["worker_command"]
            else:
                deploy(config.get("host", "pi@raspberrypi.local"), run_id)
                command = remote_command(config.get("host", "pi@raspberrypi.local"), run_id,
                                         sensors_only=config.get("sensors_only", False),
                                         queue_mib=config.get("queue_mib", 4.))
            if self.stop_requested.is_set():
                raise RuntimeError("시작 중 종료 요청: 촬영하지 않았습니다")
            log = (run / "worker.log").open("wb")
            process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=log, start_new_session=True, bufsize=0)
            with self.lock:
                self.process = process
                if self.stop_requested.is_set():
                    process.stdin.write(b"stop\n")
                    process.stdin.flush()
            finished = None
            received = 0
            laptop_capture = {"packets": 0, "read_wait_ms_total": 0., "read_wait_ms_max": 0.,
                              "writer_accept_ms_total": 0., "writer_accept_ms_max": 0.,
                              "packet_process_ms_max": 0., "packet_process_ms_max_kind": None,
                              "packet_process_ms_max_stamp_ns": None}
            while True:
                read_start = time.monotonic_ns()
                packet = read_packet(process.stdout)
                receipt_ns = time.monotonic_ns()
                if packet is None:
                    break
                header, payload = packet
                read_ms = (receipt_ns-read_start)/1e6
                # Pi와 노트북 monotonic은 서로 다른 clock이다. read 대기는 latency가 아니다.
                header = {**header, "laptop_receipt_monotonic_ns": receipt_ns,
                          "laptop_read_wait_ms": read_ms}
                accept_start = time.monotonic_ns()
                image = writer.accept(header, payload)
                accept_ms = (time.monotonic_ns()-accept_start)/1e6
                if header['kind'] == 'calibration':
                    self.update(capture_mode=header.get('capture_mode', 'pi_orb'),
                                source=header.get('source', self.snapshot()['source']))
                    if config.get('live_mapping'):
                        try:
                            from experiments.handheld_mapping.live import LiveMapper
                            live = LiveMapper(run, demo=bool(config.get('demo')))
                            with self.lock:
                                self.live = live
                        except Exception as error:
                            self.update(live_error=str(error))
                if header['kind'] == 'pose' and live is not None:
                    try:
                        live.offer_pose(header)
                    except Exception as error:
                        self.update(live_error=str(error))
                received += 1
                if received % 45 == 0 and shutil.disk_usage(run).free < 256_000_000:
                    raise OSError("노트북 공간 부족으로 촬영을 종료합니다")
                if header["kind"] == "ready" and not self.stop_requested.is_set():
                    self.update(state="recording", message=header.get("message", "촬영 중"))
                elif header["kind"] == "status":
                    self.update(tracking_state=header.get("tracking_state", -1), dropped=header.get("dropped", 0),
                                source_frames=header.get("source_frames", 0), transport_queue=header.get('queue', {}))
                    if 'capture_metrics' in header:
                        self.update(capture_metrics=header['capture_metrics'])
                elif header["kind"] == "error":
                    raise RuntimeError(header["message"])
                elif header["kind"] == "finished":
                    finished = header
                    self.update(**{target: header[source] for source, target in (
                        ('queue', 'transport_queue'), ('dropped', 'dropped'), ('source_frames', 'source_frames'),
                        ('capture_metrics', 'capture_metrics')) if source in header})
                if image is not None:
                    if live is not None:
                        live.offer(header)
                    with self.lock:
                        self.status["counts"] = dict(writer.counts)
                        if header["stream"] == "left":
                            self.preview = image
                            self.left_receipts.append(time.monotonic())
                laptop_capture['packets'] = received
                for key, value in (("read_wait_ms", read_ms), ("writer_accept_ms", accept_ms)):
                    laptop_capture[key+'_total'] += value
                    laptop_capture[key+'_max'] = max(laptop_capture[key+'_max'], value)
                processing_ms = (time.monotonic_ns()-receipt_ns)/1e6
                if processing_ms > laptop_capture['packet_process_ms_max']:
                    laptop_capture.update(packet_process_ms_max=processing_ms,
                        packet_process_ms_max_kind=header['kind'],
                        packet_process_ms_max_stamp_ns=header.get('stamp_ns'))
                self.update(laptop_capture={**laptop_capture,
                    'images': {side: dict(stats) for side, stats in writer.image_timings.items()}})
            exitcode = process.wait(timeout=10)
            writer.close()
            writer = None
            if live is not None:
                try:
                    live.finish()
                    details = live.snapshot()
                    self.update(live={key:details[key] for key in ('processed','dropped_pairs','no_depth',
                                'error','map_id','lag_ms','map_changes','synthetic')})
                except Exception as error:
                    self.update(live_error=str(error))
                live = None
            if exitcode != 0 or finished is None or finished.get("status") != "saved":
                raise RuntimeError(f"정상 종료·저장 확인 실패 (exit={exitcode}, finished={finished}) — worker.log 확인")
            backend = 'orb'
            if finished.get('capture_mode') == 'sensors_only':
                self.update(state='processing', message='노트북에서 원본으로 ORB 계산 중')
                from experiments.handheld_mapping.offline import run_point
                self.update(offline=run_point(run))
                backend = 'orb_offline'
            self.update(state="reconstructing", message="최종 keyframe으로 TSDF 지도 생성 중")
            from experiments.handheld_mapping.reconstruct import reconstruct
            dense = (reconstruct(run, progress=lambda message: self.update(message=message), backend=backend)
                     if backend != 'orb' else reconstruct(run, progress=lambda message: self.update(message=message)))
            line_partial = False
            if config.get("compare_lines"):
                self.update(state="comparing", message="동일 원본으로 점+선 SLAM 비교 중")
                from experiments.handheld_mapping.compare import run_orbline
                comparison = run_orbline(run)
                self.update(comparison=comparison)
                line_partial = comparison.get("dense", {}).get("status", "complete") != "complete"
            partial = dense["status"] != "complete" or finished.get("dropped", 0) > 0 or line_partial
            self.update(state="partial" if partial else "complete", message="일부 누락 있음 · 지도 저장 완료" if partial else "지도 저장 완료",
                        dense=dense, dropped=finished.get("dropped", 0))
        except Exception as error:
            self.update(state="failed", error=str(error), message=str(error))
        finally:
            if live is not None:
                try:
                    live.finish()
                except Exception as error:
                    self.update(live_error=str(error))
            if process is not None:
                if process.poll() is None:
                    try:
                        process.stdin.close()
                        process.wait(timeout=3)
                    except (OSError, subprocess.TimeoutExpired):
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                process.stdout.close()
                if not process.stdin.closed:
                    process.stdin.close()
            if writer is not None:
                writer.close()
            if log is not None:
                log.close()
            with self.lock:
                if not self.ended_at:
                    self.ended_at = time.monotonic()
            if run is not None:
                try:
                    atomic_json(run / "session.json", self.snapshot())
                except OSError as error:
                    self.update(state="failed", error=str(error), message="상태 파일 저장 실패: "+str(error))

    def rebuild(self, run_dir: Path) -> None:
        with self.lock:
            if self.status["state"] in ACTIVE or (self.thread is not None and self.thread.is_alive()):
                raise RuntimeError("촬영·저장 중에는 다시 생성할 수 없습니다")
            self.started_at = time.monotonic()
            self.ended_at = 0.
            self.status.update(state="reconstructing", run_dir=str(run_dir), message="원본에서 TSDF 다시 생성 중")
            self.thread = threading.Thread(target=self._rebuild, args=(Path(run_dir),), daemon=False)
            self.thread.start()

    def _rebuild(self, run_dir: Path) -> None:
        try:
            from experiments.handheld_mapping.reconstruct import reconstruct
            backend = 'orb' if any((run_dir/'orb').glob('map_*_keyframes.csv')) else 'orb_offline'
            result = reconstruct(run_dir, progress=lambda message: self.update(message=message), backend=backend)
            self.update(state="complete" if result["status"] == "complete" else "partial", message="지도 다시 생성 완료", dense=result)
        except Exception as error:
            self.update(state="failed", message=str(error), error=str(error))
        finally:
            with self.lock:
                if not self.ended_at:
                    self.ended_at = time.monotonic()
