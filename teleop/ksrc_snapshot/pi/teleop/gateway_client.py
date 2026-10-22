# ================================================================
# gateway_client.py
# 설명: 조이스틱 화면이 Pi의 control_gateway(텔레옵↔자동 중계)에 제어 메시지(하트비트·비상·시작)를 보내고
#       중계가 5 Hz로 돌려주는 상태를 받아 두는 클라이언트. pygame과 무관해서 화면 없이 시험할 수 있다.
# 사용법: gw = GatewayClient(udp_socket, (host, 5006)); gw.tick(now); gw.start(1); gw.lines(now)
# ================================================================
import math
import struct

CTL = struct.Struct("<4sIB3f")   # control_gateway의 KCTL 형식: 순서번호, 동작, 값 3개
HEARTBEAT, START, ESTOP = 1, 3, 4   # 번호는 control_gateway와 같다(2는 쓰지 않는다)
ESTOP_REPEAT = 3                 # UDP 손실에 대비해 비상 정지를 연달아 보낸다
STATUS_FRESH_S = 1.5


class GatewayClient:
    """중계로 제어 메시지를 보내고 마지막 상태 방송을 들고 있다."""

    def __init__(self, sock, dest, heartbeat_s=0.2):
        self.sock, self.dest, self.heartbeat_s = sock, dest, heartbeat_s
        self.seq = 0
        self.status, self.status_at = {}, -math.inf
        self.next_heartbeat = 0.0

    def _send(self, op, a=0.0, b=0.0, c=0.0):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        try:
            self.sock.sendto(CTL.pack(b"KCTL", self.seq, op, a, b, c), self.dest)
        except OSError:
            pass

    def tick(self, now):
        """하트비트를 heartbeat_s마다 보낸다."""
        if now >= self.next_heartbeat:
            self._send(HEARTBEAT)
            self.next_heartbeat = now + self.heartbeat_s

    def estop(self):
        for _ in range(ESTOP_REPEAT):
            self._send(ESTOP)

    def start(self, run):
        self._send(START, float(run))

    def consume(self, message, now):
        """중계의 상태 방송이면 저장하고 True, 아니면(예: STM 텔레메트리) False."""
        if not isinstance(message, dict) or "mode" not in message:
            return False
        self.status, self.status_at = message, now
        return True

    def lines(self, now):
        """화면에 찍을 (문장, 색 이름) 목록. 색 이름은 ok/warn/bad."""
        if now - self.status_at > STATUS_FRESH_S:
            return [("GATEWAY: no status (check Pi gateway)", "bad")]
        s = self.status
        mode = s.get("mode", "?")
        out = [(f"MODE {mode}" + (f"  RUN {s.get('run_id')}  (joystick ignored)" if mode == "AUTO" else ""),
                "ok" if mode == "TELEOP" else "warn")]
        out.append(("laptop link OK" if s.get("link_ok") else "laptop link LOST (AUTO keeps running)", "ok" if s.get("link_ok") else "warn"))
        if mode == "TELEOP" and s.get("missing"):
            out.append(("not ready: " + ", ".join(s["missing"]), "warn"))
        if s.get("last_reject"):
            out.append(("last start rejected: " + s["last_reject"], "bad"))
        return out
