#!/usr/bin/env python3
# ================================================================
# ksrc_telemetry_capture.py
# 설명: 비차단 분기된 전체 telemetry를 기록하고 receipt 시각의 body IMU만 진단용으로 발행한다.
# 사용법: ksrc_telemetry_capture.py --run-dir RUN --socket /tmp/ksrc-mapping.sock
# ================================================================
import argparse
import json
import math
import signal
import socket
from pathlib import Path
from prepare_mapping_run import atomic_json

class ImuClockAudit:
    """uint8 tick과 uint32 MCU milliseconds의 중복·gap·역행을 acquisition 시각과 구분한다."""
    def __init__(self) -> None:
        self.previous: tuple[int, int] | None = None

    def observe(self, tick: int, milliseconds: int) -> str:
        if not 0 <= tick <= 255 or not 0 <= milliseconds < 2**32:
            raise ValueError('invalid MCU tick/t_ms range')
        previous, self.previous = self.previous, (tick, milliseconds)
        if previous is None: return 'first'
        dtick, dt = (tick-previous[0]) % 256, (milliseconds-previous[1]) % 2**32
        if dtick == 0 and dt == 0: return 'duplicate'
        if dt == 0 or dt >= 2**31: return 'reset_or_reorder'
        return 'ok' if dtick == 1 else 'gap'


def imu_values(telemetry: dict) -> tuple[list[float], list[float]] | None:
    if not int(telemetry['flags']) & (1 << 4): return None
    gyro, acc = telemetry['gyro_dps'], telemetry['acc_g']
    if len(gyro) != 3 or len(acc) != 3 or not all(math.isfinite(v) for v in [*gyro,*acc]):
        raise ValueError('invalid IMU measurement')
    return [math.radians(v) for v in gyro], [v * 9.80665 for v in acc]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    args = parser.parse_args()
    if args.socket.exists(): parser.error('socket exists: do not replace another collector')
    import rclpy
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    rclpy.init()
    node = rclpy.create_node('ksrc_telemetry_capture')
    from rclpy.qos import qos_profile_sensor_data
    publisher = node.create_publisher(Imu, '/rover/imu/data_raw_receipt', qos_profile_sensor_data)
    diagnostics = node.create_publisher(DiagnosticArray, '/rover/telemetry_diagnostics', 10)
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.bind(str(args.socket)); receiver.settimeout(.1)
    audit = ImuClockAudit(); counts: dict[str, int] = {}
    running = True
    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, stop)
    failed = False
    try:
        with (args.run_dir/'telemetry.jsonl').open('x') as output:
            while running and rclpy.ok():
                try: data = receiver.recv(65536)
                except socket.timeout: continue
                try:
                    payload = json.loads(data)
                    if payload.get('schema') != 1 or payload.get('run_id') != args.run_dir.name:
                        raise ValueError('schema/run_id mismatch')
                    payload['collector_receipt_ns'] = node.get_clock().now().nanoseconds
                    telemetry = payload.get('telemetry')
                    if telemetry is not None:
                        state = audit.observe(telemetry['tick'], telemetry['t_ms'])
                        counts[state] = counts.get(state,0) + 1
                        payload['clock_audit'] = state
                        values = imu_values(telemetry)
                        if values and state not in ('duplicate','reset_or_reorder'):
                            message = Imu()
                            receipt = int(payload['host_receipt_ns'])
                            message.header.stamp.sec, message.header.stamp.nanosec = divmod(receipt,10**9)
                            message.header.frame_id = 'base_link'; message.orientation_covariance[0] = -1.
                            gyro,acc = values
                            message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z = gyro
                            message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z = acc
                            publisher.publish(message)
                    output.write(json.dumps(payload, allow_nan=False)+'\n')
                    counts['packets'] = counts.get('packets',0)+1
                    if counts['packets'] % 50 == 0:
                        output.flush()
                        diag = DiagnosticArray(); diag.header.stamp = node.get_clock().now().to_msg()
                        status = DiagnosticStatus(name='ksrc/receipt_imu', level=1, message='MCU clock unverified; recording only')
                        status.values = [KeyValue(key=k,value=str(v)) for k,v in counts.items()]
                        diag.status = [status]; diagnostics.publish(diag)
                except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                    counts['invalid'] = counts.get('invalid',0)+1
    except OSError:
        failed = True
        raise
    finally:
        receiver.close(); args.socket.unlink(missing_ok=True)
        atomic_json(args.run_dir/'telemetry.json', {'status':'failed' if failed else 'stopped', 'counts':counts,
                    'imu_clock_verified':False,'stamp_source':'bridge_host_receipt','frame':'base_link', 'spi_owner':False})
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
