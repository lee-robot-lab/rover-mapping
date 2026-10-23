# ================================================================
# plot_stones.py
# 설명: 보정한 돌 4개 좌표를 미션 지도 위에 전체 그림과 돌별 확대(실제 8 cm 크기)로 그린다.
# 사용법: python3 plot_stones.py
# ================================================================
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

D = '/home/su/rover-mapping/outputs/site_sim/ksrc_final/mission_fillet25/'
OUT = '/home/su/rover-mapping/outputs/site_sim/rocks_1023/stone_layout_final.png'
h = np.load(D + 'site/grid.npz')['height_m']; res = .025
m = json.load(open(D + 'mission.json'))
r15, r10 = np.array(m['routes']['15']), np.array(m['routes']['10'])
r30 = np.array(m['routes']['30'])
# 이름: (x, y, 붙인 가장자리 설명, 가장자리 x 또는 y 선)
S = {'1': (3.955, 0.055, 'east edge x=4.000, south edge y=0.010'),
     '2': (2.45, 0.85, 'plateau cliff edge x=2.403 (drop to 0.29 m pit)'),
     '3': (0.655, 0.05, 'west edge x=0.610, south edge y=0.005'),
     '4': (0.655, 3.10, 'west edge x=0.610, north edge y=3.145')}
ext = [0, h.shape[1] * res, 0, h.shape[0] * res]
fig = plt.figure(figsize=(13, 9.5))
ax = fig.add_axes([0.04, 0.30, 0.55, 0.66])
ax.imshow(h, origin='lower', extent=ext, cmap='terrain', alpha=.75)
ax.plot(*r15.T, color='tab:orange', lw=2, label='15 deg route'); ax.plot(*r10.T, color='tab:green', lw=2, label='10 deg route')
ax.plot(*r30.T, color='tab:blue', lw=1.5, label='30 deg route'); ax.plot(*m['start'][:2], 'ks', label='start')
for k, (x, y, _) in S.items():
    ax.add_patch(Circle((x, y), .06, color='red', zorder=5))
    ax.annotate(f'{k}  ({x:.3f}, {y:.3f})', (x, y), xytext=(x + (-.12 if x > 3 else .12), y + (.12 if y < 1 else -.12)),
                ha='right' if x > 3 else 'left', fontsize=10, weight='bold', color='darkred')
ax.set_xlim(0.4, 4.1); ax.set_ylim(-0.1, 3.35); ax.set_aspect('equal'); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('Final stone positions (red dot drawn 1.5x; zoom panels are true size)'); ax.legend(loc='center', fontsize=8)
# 확대 4장 (실제 크기)
pos = {'1': [0.62, 0.62, 0.17, 0.30], '2': [0.80, 0.62, 0.17, 0.30], '3': [0.62, 0.30, 0.17, 0.30], '4': [0.80, 0.30, 0.17, 0.30]}
hz = 0.16
for k, (x, y, note) in S.items():
    a = fig.add_axes(pos[k]); a.set_aspect('equal')
    a.imshow(h, origin='lower', extent=ext, cmap='terrain', alpha=.75, vmin=np.nanmin(h), vmax=np.nanmax(h))
    a.add_patch(Circle((x, y), .04, fc='red', ec='k', lw=.8, zorder=5))
    a.add_patch(Circle((x, y), .21, fill=False, ls=':', color='k', lw=.8))
    a.set_xlim(x - hz, x + hz); a.set_ylim(y - hz, y + hz); a.set_title(f'stone {k}: ({x:.3f}, {y:.3f})', fontsize=9)
    a.tick_params(labelsize=7)
fig.text(0.04, 0.22, 'Zoom panels: red = 8 cm stone (true size), dotted = 0.21 m rover radius. Map surface ends at the edge (no wall mesh in the Isaac scene).\n'
         + '\n'.join(f'  stone {k}: {v[2]}' for k, v in S.items()), fontsize=9, va='top')
fig.savefig(OUT, dpi=105); print(OUT)
