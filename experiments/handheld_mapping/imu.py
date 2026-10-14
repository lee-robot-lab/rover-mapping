# ================================================================
# imu.py
# 설명: Pi 브리지의 IMU telemetry를 보정하여 노트북 run에 별도 기록한다.
# 사용법: from experiments.handheld_mapping.imu import ImuCapture
# ================================================================
from collections import deque
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import threading
import time


MAX_LINE = 65536


def finite_vector(value: object, name: str) -> list[float]:
    """3축 유한 수치만 허용한다."""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{name}: finite vector length 3 required')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f'{name}: finite vector required')
    return [float(x) for x in value]


def strict_json(value: str) -> dict:
    """비표준 NaN/Infinity와 JSON 객체 외 입력을 거부한다."""
    def invalid(token: str) -> None:
        raise ValueError(f'invalid JSON finite number: {token}')
    result = json.loads(value, parse_constant=invalid)
    if not isinstance(result, dict):
        raise ValueError('JSON object required')
    return result


def load_calibration(path: Path) -> dict:
    """버전·센서·body frame·단위·축 매핑이 명시된 보정을 읽는다."""
    calibration = strict_json(Path(path).read_text())
    for name, expected in [('version', 1), ('sensor', 'ISM330DHCX'), ('frame', 'rover_body'),
                           ('acc_unit', 'g'), ('gyro_unit', 'deg/s')]:
        if calibration.get(name) != expected:
            raise ValueError(f'calibration {name} must be {expected}')
    axes = calibration.get('chip_to_body')
    if (not isinstance(axes, list) or len(axes) != 3 or
            any(not isinstance(row, list) or len(row) != 2 or row[0] not in (0, 1, 2) or
                row[1] not in (-1, 1) for row in axes) or len({row[0] for row in axes}) != 3):
        raise ValueError('calibration chip_to_body must be signed axis permutation')
    for name in ('gyro_bias_dps', 'acc_offset_g', 'acc_scale'):
        calibration[name] = finite_vector(calibration.get(name), name)
    if any(x <= 0 for x in calibration['acc_scale']):
        raise ValueError('acc_scale must be positive')
    return calibration


def correct_measurement(calibration: dict, gyro_dps: object, acc_g: object) -> dict:
    """이미 body 좌표계인 디코딩 값에 bias/offset/scale만 적용한다."""
    gyro = finite_vector(gyro_dps, 'gyro_dps')
    acc = finite_vector(acc_g, 'acc_g')
    gyro = [gyro[i] - calibration['gyro_bias_dps'][i] for i in range(3)]
    acc = [(acc[i] - calibration['acc_offset_g'][i]) * calibration['acc_scale'][i] for i in range(3)]
    if not all(math.isfinite(x) for x in gyro + acc):
        raise ValueError('corrected values must be finite')
    return {'gyro_dps': gyro, 'acc_g': acc, 'acc_norm_g': math.hypot(*acc),
            'roll_deg': math.degrees(math.atan2(acc[1], acc[2])),
            'pitch_deg': math.degrees(math.atan2(-acc[0], math.hypot(acc[1], acc[2]))),
            'tilt_method': 'accelerometer_only_unreliable_during_motion'}


class ImuCapture:
    """소유 SSH subprocess를 관리하며 원본·보정·시계 출처를 bounded 상태로 기록한다."""

    def __init__(self, host: str, run: Path, calibration_path: Path, urdf_path: Path,
                 socket_path: str = '/dev/shm/rover-mapping-imu.sock',
                 command: list[str] | None = None) -> None:
        if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', host) or host.startswith('-'):
            raise ValueError('invalid SSH host')
        if not socket_path.startswith('/') or '\x00' in socket_path:
            raise ValueError('absolute socket path required')
        self.host, self.run = host, Path(run)
        self.calibration_path, self.urdf_path = Path(calibration_path), Path(urdf_path)
        remote = f'/dev/shm/rover_mapping_{self.run.name}/code'
        script = (f'set -e; cd {shlex.quote(remote)}; '
                  'export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1; '
                  'exec /usr/bin/python3 -u -m experiments.handheld_mapping.imu_subscriber '
                  f'--socket {shlex.quote(socket_path)}')
        self.command = list(command) if command is not None else [
            'ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3',
            host, shlex.join(['bash', '-c', script])]
        self.process: subprocess.Popen | None = None
        self._lock = threading.RLock()
        self._fresh, self._stopping = threading.Event(), threading.Event()
        self._reader: threading.Thread | None = None
        self._stderr_reader: threading.Thread | None = None
        self._stderr: deque[str] = deque(maxlen=16)
        self._recent: deque[tuple[int, int]] = deque(maxlen=256)
        self._last_t: int | None = None
        self._last_pi_ns: int | None = None
        self._last_fresh_ns: int | None = None
        self._tee_baseline: int | None = None
        self._ready = False
        self._log = None
        self._final: dict | None = None
        self._manifest: dict = {}
        self._status = {'state': 'starting', 'recorded': 0, 'valid': 0, 'unique': 0,
                        'duplicates': 0, 'startup_discarded': 0, 'resets': 0,
                        'clock_jumps': 0, 'malformed': 0, 'tee_drops': 0,
                        'imu_clock_verified': False, 'orb_fused': False,
                        'extrinsic_provisional': True, 'run_id': self.run.name,
                        'hz_basis': 'pi_receipt_monotonic_unique_packets'}

    def _json_file(self, name: str, data: dict) -> None:
        with (self.run / name).open('x') as output:
            json.dump(data, output, indent=2, allow_nan=False)
            output.write('\n')

    def _prepare(self) -> None:
        self.calibration = load_calibration(self.calibration_path)
        cal_bytes, urdf_bytes = self.calibration_path.read_bytes(), self.urdf_path.read_bytes()
        self.run.mkdir(parents=True, exist_ok=True)
        for name in ('imu.jsonl', 'imu_calibration.json', 'robot.urdf', 'imu_manifest.json', 'imu_summary.json'):
            if (self.run / name).exists():
                raise FileExistsError(self.run / name)
        for name, data in [('imu_calibration.json', cal_bytes), ('robot.urdf', urdf_bytes)]:
            with (self.run / name).open('xb') as output:
                output.write(data)
        self._manifest = {
            'schema': 1, 'run_id': self.run.name, 'sensor': 'ISM330DHCX', 'frame': 'rover_body',
            'calibration_sha256': hashlib.sha256(cal_bytes).hexdigest(),
            'urdf_sha256': hashlib.sha256(urdf_bytes).hexdigest(),
            'chip_to_body_expected': self.calibration['chip_to_body'],
            'imu_clock_verified': False, 'orb_fused': False, 'extrinsic_provisional': True,
            'imu_source': 'existing_spi_owner_unix_tee',
            'rate_note': 'unique MCU control packets; imu_n may average multiple 208 Hz sensor samples',
            'clock_note': 'MCU t_ms, Pi realtime/monotonic and laptop receipts are distinct clocks; no camera alignment verified',
            'tilt_note': 'accelerometer-only diagnostic; unreliable during motion',
        }
        self._log = (self.run / 'imu.jsonl').open('x', buffering=1)

    def start(self, timeout: float = 8.) -> None:
        """실제 축 매핑과 첫 fresh unique IMU를 확인할 때까지 기다린다."""
        if self.process is not None or self._final is not None:
            raise RuntimeError('IMU capture already started or stopped')
        try:
            self._prepare()
            self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, text=True, bufsize=1,
                                            start_new_session=True)
            self._reader = threading.Thread(target=self._read, name='mapping-imu', daemon=True)
            self._stderr_reader = threading.Thread(target=self._read_stderr, name='mapping-imu-stderr', daemon=True)
            self._stderr_reader.start()
            self._reader.start()
            if not self._fresh.wait(timeout):
                raise TimeoutError('IMU fresh sample timeout')
            state = self.snapshot()
            if state['state'] != 'recording':
                raise RuntimeError(state.get('error', 'IMU failed before fresh sample'))
        except Exception as error:
            self._fail(str(error))
            self.stop()
            raise

    def _fail(self, error: str) -> None:
        with self._lock:
            self._status.update(state='failed', error=error[:2000])
        self._fresh.set()

    def _read_stderr(self) -> None:
        assert self.process is not None and self.process.stderr is not None
        while True:
            line = self.process.stderr.readline(MAX_LINE + 1)
            if not line:
                return
            with self._lock:
                self._stderr.append(line[-2000:].rstrip())

    def _read(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        try:
            while True:
                line = self.process.stdout.readline(MAX_LINE + 1)
                if not line:
                    if not self._stopping.is_set():
                        raise RuntimeError('IMU subscriber exited unexpectedly')
                    return
                if len(line) > MAX_LINE:
                    raise ValueError('IMU JSON line exceeds bound')
                try:
                    event = strict_json(line)
                except ValueError:
                    self._log.write(json.dumps({'acceptance': 'invalid_json', 'raw_line': line.rstrip()}) + '\n')
                    raise
                with self._lock:
                    self._event(event)
        except Exception as error:
            self._fail(str(error))
            if self.process.poll() is None:
                self.process.terminate()

    def _event(self, event: dict) -> None:
        if not self._ready:
            if event.get('kind') != 'ready':
                raise ValueError('IMU ready metadata required before telemetry')
            if event.get('chip_to_body') != self.calibration['chip_to_body']:
                raise ValueError('IMU chip_to_body differs from calibration')
            self._ready = True
            self._manifest['chip_to_body_actual'] = event['chip_to_body']
            self._json_file('imu_manifest.json', self._manifest)
            return
        if event.get('kind') == 'malformed':
            self._status['malformed'] += 1
            raise ValueError('invalid JSON datagram reported by subscriber')
        if event.get('kind') != 'telemetry' or event.get('schema') != 1:
            raise ValueError('IMU schema 1 telemetry required')
        now = time.monotonic_ns()
        row = {'schema': 1, 'run_id': self.run.name, 'raw': event, 'calibrated': None,
               'acceptance': 'invalid_flag', 'clock': {
                   'mcu_t_ms': None, 'pi_receipt_realtime_ns': event.get('host_receipt_ns'),
                   'pi_receipt_monotonic_ns': event.get('host_monotonic_ns'),
                   'laptop_receipt_realtime_ns': time.time_ns(), 'laptop_receipt_monotonic_ns': now,
                   'imu_clock_verified': False, 'camera_clock_alignment': 'unverified',
                   'segment': self._status['resets']}}
        self._status['recorded'] += 1
        try:
            drops = event.get('tee_drops', 0)
            if type(drops) is not int or drops < 0:
                raise ValueError('invalid tee_drops')
            if self._tee_baseline is None:
                self._tee_baseline = drops
            if drops < self._status.get('tee_drops_total', self._tee_baseline):
                raise ValueError('tee_drops counter reset: bridge restarted')
            self._status.update(tee_drops=drops - self._tee_baseline,
                                tee_drops_baseline=self._tee_baseline, tee_drops_total=drops)
            telemetry = event.get('telemetry')
            if not isinstance(telemetry, dict) or not (int(telemetry.get('flags', 0)) & 16):
                return
            self._status['valid'] += 1
            corrected = correct_measurement(self.calibration, telemetry.get('gyro_dps'), telemetry.get('acc_g'))
            row['calibrated'] = corrected
            t_ms, tick = telemetry.get('t_ms'), telemetry.get('tick')
            pi_ns, pi_realtime = event.get('host_monotonic_ns'), event.get('host_receipt_ns')
            for value, limit, name in [(t_ms, 2**32, 't_ms'), (tick, 256, 'tick'),
                                       (pi_ns, 2**64, 'host_monotonic_ns'), (pi_realtime, 2**64, 'host_receipt_ns')]:
                if type(value) is not int or not 0 <= value < limit:
                    raise ValueError(f'invalid {name}')
            row['clock']['mcu_t_ms'] = t_ms
            if self._last_t is None:
                self._status['startup_discarded'] += 1
                row['acceptance'] = 'startup_discard'
            elif t_ms == self._last_t:
                self._status['duplicates'] += 1
                row['acceptance'] = 'duplicate'
                return
            elif self._last_pi_ns is not None and pi_ns <= self._last_pi_ns:
                row['acceptance'] = 'pi_clock_nonmonotonic'
                self._status['clock_jumps'] += 1
                return
            elif t_ms < self._last_t and not (self._last_t > 2**32 - 10000 and t_ms < 10000):
                # 시작 첫 패킷만 오래된 이전 MCU 응답일 수 있다. 이후 감소는 새 segment로 분리한다.
                if self._status['unique']:
                    self._status['resets'] += 1
                    row['acceptance'] = 'reset_boundary_discard'
                    row['clock']['segment'] = self._status['resets']
                    self._last_fresh_ns = None
                    self._recent.clear()
                else:
                    row['acceptance'] = 'accepted'
            else:
                elapsed_ms = (pi_ns - self._last_pi_ns) / 1e6
                if (t_ms - self._last_t) % 2**32 > max(1000, elapsed_ms + 1000):
                    self._status['clock_jumps'] += 1
                    row['acceptance'] = 'mcu_clock_jump_discard'
                    self._last_fresh_ns = None
                    self._recent.clear()
                else:
                    row['acceptance'] = 'accepted'
            self._last_t, self._last_pi_ns = t_ms, pi_ns
            if row['acceptance'] == 'accepted':
                self._status['unique'] += 1
                self._status.update(corrected)
                self._status.update(state='recording', t_ms=t_ms, tick=tick, imu_n=telemetry.get('imu_n'))
                self._last_fresh_ns = now
                self._recent.append((pi_ns, now))
                self._fresh.set()
        except Exception as error:
            row['acceptance'], row['error'] = 'invalid_measurement', str(error)
            raise
        finally:
            self._log.write(json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n')

    def snapshot(self) -> dict:
        """UI용 detached snapshot을 반환하며 수신 정지는 unique 표본으로 판단한다."""
        with self._lock:
            if self._final is not None:
                return copy.deepcopy(self._final)
            state = copy.deepcopy(self._status)
            now = time.monotonic_ns()
            recent = [stamp for stamp in self._recent if now - stamp[1] < 2_000_000_000]
            state['hz'] = (len(recent) - 1) * 1e9 / (recent[-1][0] - recent[0][0]) if len(recent) > 1 else 0.
            state['stale'] = self._last_fresh_ns is None or now - self._last_fresh_ns > 500_000_000
            state['stderr_tail'] = list(self._stderr)
            return state

    def stop(self) -> dict:
        """소유 process만 정상 종료하고 파일을 닫은 후 최종 상태를 저장한다."""
        if self._final is not None:
            return self.snapshot()
        self._stopping.set()
        if self.process is not None:
            if self.process.poll() is None:
                try:
                    self.process.stdin.write('stop\n')
                    self.process.stdin.flush()
                except (BrokenPipeError, OSError, ValueError):
                    pass
                try:
                    self.process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        self.process.wait(timeout=1.)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self.process.wait(timeout=1.)
            for thread in (self._reader, self._stderr_reader):
                if thread is not None:
                    thread.join(timeout=2.)
            for handle in (self.process.stdin, self.process.stdout, self.process.stderr):
                if handle is not None:
                    handle.close()
        if self._log is not None:
            self._log.flush()
            self._log.close()
            if not (self.run / 'imu_manifest.json').exists():
                self._json_file('imu_manifest.json', self._manifest)
        with self._lock:
            if self._status['state'] != 'failed':
                self._status['state'] = 'stopped'
            self._final = self.snapshot()
        if self._log is not None:
            self._json_file('imu_summary.json', self._final)
        return self.snapshot()
