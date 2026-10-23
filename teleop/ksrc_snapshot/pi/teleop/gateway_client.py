# ================================================================
# gateway_client.py
# 설명: 조이스틱 화면이 Pi의 control_gateway(텔레옵↔자동 중계)에 제어 메시지(하트비트·비상·시작)를 보내고
#       중계가 5 Hz로 돌려주는 상태를 받아 두는 클라이언트. pygame과 무관해서 화면 없이 시험할 수 있다.
# 사용법: gw = GatewayClient(udp_socket, (host, 5006)); gw.tick(now); gw.start(1); gw.anchor(now); gw.orb_reset(now); gw.lines(now)
# ================================================================
import math
import struct

CTL = struct.Struct("<4sIB3f")   # control_gateway의 KCTL 형식: 순서번호, 동작, 값 3개
HEARTBEAT, START, ESTOP, ANCHOR, ORB_RESET, ANCHOR2, ANCHOR3, ANCHOR4 = 1, 3, 4, 5, 6, 7, 8, 9
ANCHOR_OPS = {1: ANCHOR, 2: ANCHOR2, 3: ANCHOR3, 4: ANCHOR4}
ANCHOR_LABELS = {1: 'ANCHOR1 -Y', 2: 'ANCHOR2 -X', 3: 'ANCHOR3 +Y', 4: 'ANCHOR4 +X'}
ESTOP_REPEAT = 3                 # UDP 손실에 대비해 비상 정지를 연달아 보낸다
STATUS_FRESH_S = 1.5
ANCHOR_REPLY_S = 1.5             # 이 시간 안에 anchor_count가 늘지 않으면 거절된 것으로 본다
STATIONARY_NEED_S = 3.0          # control_gateway의 ready_stationary_s와 같다
ORB_RESET_NOTE_S = 3.0           # ORB RESET을 보낸 뒤 이 시간 동안 안내 줄을 보인다


class GatewayClient:
    """중계로 제어 메시지를 보내고 마지막 상태 방송을 들고 있다."""

    def __init__(self, sock, dest, heartbeat_s=0.2):
        self.sock, self.dest, self.heartbeat_s = sock, dest, heartbeat_s
        self.seq = 0
        self.status, self.status_at = {}, -math.inf
        self.next_heartbeat = 0.0
        self.orb_reset_at = -math.inf
        self.anchor_wait, self.anchor_result = None, None   # (보낸 시각, 그때의 anchor_count), applied/rejected/sent

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
        if run not in (1, 2, 3, 4):
            raise ValueError("run must be 1, 2, 3, or 4")
        self._send(START, float(run))

    def anchor(self, now: float, preset: int = 1) -> None:
        if preset not in (1, 2, 3, 4):
            raise ValueError('preset must be 1, 2, 3, or 4')
        self._send(ANCHOR_OPS[preset])
        self.anchor_wait, self.anchor_result = (now, self.status.get("anchor_count")), None

    def orb_reset(self, now):
        self._send(ORB_RESET)
        self.orb_reset_at = now

    def orb_reset_allowed(self, now):
        """ORB RESET이 받아들여질 조건(TELEOP·상태 최신). 정지 여부는 묻지 않는다."""
        return now - self.status_at <= STATUS_FRESH_S and self.status.get("mode") == "TELEOP"

    def anchor_allowed(self, now):
        """ANCHOR가 받아들여질 조건(TELEOP·정지 3초·ORB 정상·상태 최신)이 화면 상태로 충족되는가. 출발 구역은 anchor의 조건이 아니다."""
        if now - self.status_at > STATUS_FRESH_S or self.status.get("mode") != "TELEOP":
            return False
        return not {"status", "stationary_s", "orb_ok"} & set(self.status.get("missing") or [])

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
        mode, run = s.get("mode", "?"), s.get("run", s.get("run_id"))
        stage = f"  {s['stage']}" if s.get("stage") else ""
        anchor = ANCHOR_LABELS.get(s.get('anchor_id'))
        out = [(f"MODE {mode}" + (f"  RUN {run}{stage}  (joystick ignored)" if mode == "AUTO" else ""),
                "ok" if mode == "TELEOP" else "warn")]
        if anchor:
            out[0] = (out[0][0] + f'  {anchor}', out[0][1])
        out.append(("laptop link OK" if s.get("link_ok") else "laptop link LOST (AUTO keeps running)", "ok" if s.get("link_ok") else "warn"))
        if s.get("recovering"):
            out.append(("ORB recovering - robot holds still", "warn"))
        if mode == "TELEOP":
            out += self._ready_lines(s, now)
        else:
            self.anchor_wait = self.anchor_result = None   # 다음 정지 때 이전 anchor 결과가 남아 있지 않게 한다
        if s.get("last_reject"):
            out.append(("last start rejected: " + s["last_reject"], "bad"))
        return out

    def _ready_lines(self, s, now):
        """TELEOP 때 보여 줄 준비 상태·anchor 결과·직전 회차 결과."""
        out, missing = [], s.get("missing") or []
        if now - self.orb_reset_at <= ORB_RESET_NOTE_S:
            out.append(("ORB reset requested - hold still until it recovers", "warn"))
        if "status" in missing:
            out.append(("NOT READY: no runner status (stack starting?)", "warn"))
        elif missing:
            why = {"stationary_s": f"hold still {s.get('stationary_s', 0.):.1f}/{STATIONARY_NEED_S:.1f} s",
                   "orb_ok": "ORB not tracking",
                   "in_region1": "not in start zone (align, hold still, press ANCHOR)"}
            out.append(("NOT READY: " + "; ".join(why.get(name, name) for name in missing), "warn"))
        else:
            out.append(("READY: START allowed", "ok"))
        if self.anchor_wait:
            sent, before = self.anchor_wait
            count = s.get("anchor_count")
            if before is None or count is None:
                self.anchor_wait, self.anchor_result = None, "sent"
            elif count > before:
                self.anchor_wait, self.anchor_result = None, "applied"
            elif now - sent > ANCHOR_REPLY_S:
                self.anchor_wait, self.anchor_result = None, "rejected"
        if self.anchor_wait:
            out.append(("ANCHOR sent...", "warn"))
        elif self.anchor_result == "applied":
            label = ANCHOR_LABELS.get(s.get('anchor_id'), 'ANCHOR')
            out.append((f"{label} applied (current pose = start preset)", "ok"))
        elif self.anchor_result == "rejected":
            out.append(("ANCHOR rejected: hold still 3 s with ORB tracking, then press again", "bad"))
        elif self.anchor_result == "sent":
            out.append(("ANCHOR sent (this stack cannot confirm; check operator_anchor event)", "warn"))
        run, reason = s.get("run"), s.get("reason")
        if s.get("done") and run:
            out.append((f"RUN {run} COMPLETE - back to TELEOP", "ok") if reason == "complete"
                       else (f"RUN {run} FAILED ({reason}) - back to TELEOP", "bad"))
        elif reason == "operator" and run:
            out.append((f"RUN {run} stopped by operator", "warn"))
        return out
