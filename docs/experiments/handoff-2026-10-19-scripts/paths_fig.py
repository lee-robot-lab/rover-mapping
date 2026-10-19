import sys; sys.path.insert(0,'.')
import json,numpy as np
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in font_manager.fontManager.ttflist:
    if 'Noto Sans CJK JP' in f.name: plt.rcParams['font.family']=f.name; break
plt.rcParams['axes.unicode_minus']=False
from scipy.ndimage import distance_transform_edt
from experiments.handheld_mapping.site_mission import SiteMission, astar
m=SiteMission.load(Path('outputs/site_sim/abD1018/mission_bundle'),load_mesh=False); s=m.site
slope=m.settings['normal_slope_deg']
clear=s.observed&~s.blocked&(s.slope_deg<=slope+1e-4)
D=distance_transform_edt(np.pad(clear,1,constant_values=False))[1:-1,1:-1]*s.resolution
rows,cols=D.shape
x0,y0=s.origin; r=s.resolution
extent=(x0,x0+cols*r,y0,y0+rows*r)
direct=np.asarray([m.start[:2],m.junction])
safe=s.safe_mask(slope); cells=s.cells(direct); a,b=(tuple(c[::-1]) for c in cells)
raw=astar(safe,a,b); xy=s.origin+(np.asarray(raw)[:,::-1]+.5)*s.resolution
old=np.vstack((direct[0],xy,direct[-1]))
pulled=m.connector()
c370=np.array(json.load(open('outputs/site_sim/cx1019_corner370/mission_bundle/mission.json'))['connector_waypoints'])
c375=np.array(json.load(open('outputs/site_sim/cx1019_corner375/mission_bundle/mission.json'))['connector_waypoints'])
def gt(l,tmax=None):
    R=[json.loads(x) for x in open(f'outputs/site_sim/{l}/telemetry.jsonl')]
    g=np.array([r_['position_m'][:2] for r_ in R]);t=np.array([r_['stamp_s'] for r_ in R])
    return g if tmax is None else g[t<=tmax]
fig,ax=plt.subplots(1,2,figsize=(17,8.5))
def base(a_,title):
    im=a_.imshow(D,origin='lower',extent=extent,cmap='viridis',vmin=0,vmax=0.8)
    a_.contour(np.linspace(x0+r/2,x0+cols*r-r/2,cols),np.linspace(y0+r/2,y0+rows*r-r/2,rows),D,levels=[0.2854],colors='red',linewidths=1.6)
    a_.contour(np.linspace(x0+r/2,x0+cols*r-r/2,cols),np.linspace(y0+r/2,y0+rows*r-r/2,rows),D,levels=[0.3354],colors='orange',linewidths=1.6,linestyles='--')
    a_.set_xlim(2.7,4.7);a_.set_ylim(0.1,1.5);a_.set_aspect('equal');a_.set_xlabel('x [m]');a_.set_ylabel('y [m]');a_.set_title(title)
    a_.plot(*m.start[:2],'w*',ms=14);a_.annotate('시작',m.start[:2],color='w',xytext=(5,8),textcoords='offset points')
    a_.plot(*m.junction,'wD',ms=9);a_.annotate('갈림길',m.junction,color='w',xytext=(6,-14),textcoords='offset points')
    return im
im=base(ax[0],'① 계획 경로 4종 (배경: 벽까지 거리 [m], 빨강=정지 검사 문턱 0.2854, 주황 점선=계획 문턱 0.3354)')
ax[0].plot(old[:,0],old[:,1],'-',color='cyan',lw=2.2,label=f'A* 원본(커밋 전, 57점) — repeat1018a가 쓴 경로')
ax[0].plot(pulled[:,0],pulled[:,1],'o-',color='magenta',lw=2.2,ms=6,label='단축 경로(494431e 이후, 5점) — gx_*, id_*, rt1018*')
ax[0].plot(c370[:,0],c370[:,1],'s-',color='white',lw=2.2,ms=6,label='corner370 직각(3점) — cx1019_corner370, cy/cz/cu/cg/cb')
ax[0].plot(c375[:,0],c375[:,1],'^-',color='yellow',lw=2.0,ms=6,label='corner375 직각(4점)')
ax[0].legend(loc='upper right',fontsize=8,framealpha=.85)
base(ax[1],'② 실제 주행 궤적(GT) — 추정이 아니라 로버의 실제 위치')
ax[1].plot(old[:,0],old[:,1],'-',color='cyan',lw=1,alpha=.7)
ax[1].plot(pulled[:,0],pulled[:,1],'o-',color='magenta',lw=1,ms=3,alpha=.7)
tracks=[('repeat1018a (완주 253 s, 처음 24 s)','repeat1018a','lime',24),('id_norl_1 (11 s 정지, RL 없음·이상적 IMU)','id_norl_1','red',None),('id_rl_1 (12 s 정지, RL ON·이상적 IMU)','id_rl_1','orange',None),('cx1019_corner370 (26 s)','cx1019_corner370','white',None),('cz_b370 (blend, 63 s)','cz_b370','deepskyblue',None)]
for lab,l,c,tm in tracks:
    g=gt(l,tm); ax[1].plot(g[:,0],g[:,1],'-',color=c,lw=2,label=lab); ax[1].plot(*g[-1],'o',color=c,ms=8,mec='k')
ax[1].legend(loc='upper right',fontsize=8,framealpha=.85)
cb=fig.colorbar(im,ax=ax,shrink=.7,pad=.02);cb.set_label('벽(비통과 칸)까지 거리 [m]')
plt.savefig('outputs/site_sim/paths_overview.png',dpi=110,bbox_inches='tight')
print('saved')
