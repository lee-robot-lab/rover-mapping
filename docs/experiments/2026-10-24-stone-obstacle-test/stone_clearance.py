# ================================================================
# stone_clearance.py
# 설명: run의 telemetry.jsonl(GT 위치·자세)에서 로버 바깥 윤곽(바퀴 pivot ± 바퀴 반지름 근사 직사각형)과
#       각 돌 사이의 최소 거리를 구한다. 서버에서 python3로 실행한다(읽기 전용).
# 사용법: python3 stone_clearance.py RUN_DIR... (RUN_DIR/output/telemetry.jsonl 사용)
# ================================================================
import json, math, sys
# 로버 윤곽 근사: pivot x 0.084/-0.121, y ±0.1325, 바퀴 반지름 0.0474·폭 0.0354 → x [-0.168, 0.131], y ±0.151
X0, X1, Y = -0.168, 0.131, 0.151
SETS = {'8': (0.04, [(3.955, .055), (2.45, .85), (.655, .05), (.655, 3.10)]),
        '10': (0.05, [(3.945, .065), (2.458, .85), (.665, .06), (.665, 3.09)]),
        '15': (0.075, [(3.92, .09), (2.483, .85), (.69, .085), (.69, 3.065)])}

def yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

for run in sys.argv[1:]:
    size = '15' if 'rocks15' in run else '10' if 'rocks10' in run else '8'
    R, S = SETS[size]
    best = [(9., 0.)] * 4
    for line in open(run + '/output/telemetry.jsonl'):
        d = json.loads(line)
        t = d['stamp_s']; x, y = d['position_m'][:2]; c, s = math.cos(yaw(d['quaternion_wxyz'])), math.sin(yaw(d['quaternion_wxyz']))
        for i, (sx, sy) in enumerate(S):
            dx, dy = sx - x, sy - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy          # 돌 중심을 로버 좌표로
            ex = max(X0 - lx, 0, lx - X1); ey = max(abs(ly) - Y, 0)
            dist = math.hypot(ex, ey)
            if dist < best[i][0]:
                best[i] = (dist, t)
    # 2번 돌 위도(y 0.80~0.90)를 지나는 통과별 로버 중심 x(돌에서 멀수록 큼): 시간 간격 5 s 이상이면 다른 통과
    passes, last = [], -99
    for line in open(run + '/output/telemetry.jsonl'):
        d = json.loads(line); x, y = d['position_m'][:2]
        if 0.80 <= y <= 0.90 and 2.5 <= x <= 3.1:
            if d['stamp_s'] - last > 5: passes.append([])
            passes[-1].append(x); last = d['stamp_s']
    print('   2번 돌 위도 통과별 평균 x:', [round(sum(p) / len(p), 3) for p in passes])
    comp = json.load(open(run + '/output/completion.json'))
    print(run.split('/')[-1], comp['status'], round(comp['duration_sim_s']),
          '| 윤곽-돌 최소 거리 m (돌 표면까지=값−반지름):', {str(i + 1): (round(b[0], 3), round(b[1])) for i, b in enumerate(best)}, '반지름', R)
