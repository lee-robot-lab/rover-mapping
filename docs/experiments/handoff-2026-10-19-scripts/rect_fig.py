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
from experiments.handheld_mapping.site_mission import SiteMission
m=SiteMission.load(Path('outputs/site_sim/cy_missions/rect_mission'),load_mesh=False); s=m.site
r=s.resolution;x0,y0=s.origin
mask=s.observed&~s.blocked&(s.slope_deg<=m.settings['trial_slope_deg']+1e-4)
D=distance_transform_edt(np.pad(mask,1,constant_values=False))[1:-1,1:-1]*r
rows,cols=D.shape;ext=(x0,x0+cols*r,y0,y0+rows*r)
conn=m.connector();R=m.routes
def dense(P):
    out=[]
    for u,v in zip(P[:-1],P[1:]):
        for t in np.linspace(0,1,max(2,int(np.linalg.norm(v-u)/.01))):
            p=u+(v-u)*t;c=s.cells(p);out.append(D[c[1],c[0]])
    return np.array(out)
fig,ax=plt.subplots(1,2,figsize=(17,8),gridspec_kw={'width_ratios':[1.5,1]})
a=ax[0]
im=a.imshow(np.where(mask,D,np.nan),origin='lower',extent=ext,cmap='viridis',vmin=0,vmax=.8)
a.imshow(np.where(~mask,1.,np.nan),origin='lower',extent=ext,cmap='gray_r',vmin=0,vmax=2,alpha=.55)
styles={'30':('red','-',3.4),'15':('orange','-',2.6),'10':('magenta','-',2.0)}
a.plot(conn[:,0],conn[:,1],'-',color='cyan',lw=3.8,label='연결 경로 (시작→갈림길)')
for k in ('10','15','30'):
    c,ls,lw=styles[k]; P=R[k]; a.plot(P[:,0],P[:,1],ls,color=c,lw=lw,label=f'{k}° 램프 경로')
    for u,v in zip(P[:-1],P[1:]):
        if np.linalg.norm(v-u)>.3:
            mid=(u+v)/2;d=(v-u)/np.linalg.norm(v-u);a.annotate('',xy=mid+d*.05,xytext=mid-d*.05,arrowprops=dict(arrowstyle='->',color=c,lw=1.8))
allp=np.vstack([conn]+[R[k] for k in R])
a.plot(*m.start[:2],'w*',ms=16,mec='k');a.plot(*m.junction,'wD',ms=10,mec='k');a.plot(*m.summit,'wP',ms=12,mec='k')
for lab,p in (('시작',m.start[:2]),('갈림길',m.junction),('정상(5.35, 0.4)',m.summit)): a.annotate(lab,p,xytext=(7,7),textcoords='offset points',color='w',fontsize=10)
a.set_aspect('equal');a.set_xlabel('x [m]');a.set_ylabel('y [m]');a.set_title('직각 직선 경로 (모든 구간이 x 또는 y 축에 평행, 코너는 90°)');a.legend(loc='lower left',fontsize=8,framealpha=.9)
fig.colorbar(im,ax=a,shrink=.6,pad=.01).set_label('벽까지 거리 [m]')
b=ax[1]
for name,P,c in (('연결 경로',conn,'cyan'),('30°',np.vstack([conn,R['30'][1:]]),'red'),('15°',np.vstack([conn,R['15'][1:]]),'orange'),('10°',np.vstack([conn,R['10'][1:]]),'magenta')):
    d=dense(P); x=np.linspace(0,np.linalg.norm(np.diff(P,axis=0),axis=1).sum(),len(d)); b.plot(x,d,color=c,label=f'{name}: 최소 {d.min():.3f}, 평균 {d.mean():.3f} m')
b.axhline(.2854,color='red',lw=1.2,label='정지 검사 문턱 0.2854');b.axhline(.3354,color='gray',ls='--',lw=1.2,label='계획 문턱 0.3354')
b.set_ylim(0,.55);b.set_xlabel('경로 길이 [m]');b.set_ylabel('벽까지 거리 [m]');b.set_title('경로를 따라 본 벽까지 거리 (0.025 m 격자 단위로 계단)');b.legend(fontsize=8,loc='lower right');b.grid(alpha=.3)
plt.savefig('outputs/site_sim/rect_paths.png',dpi=105,bbox_inches='tight');print('saved')
for name,P in (('connector',conn),('30',R['30']),('15',R['15']),('10',R['10'])): d=dense(P);print(name,'min',d.min().round(3),'mean',d.mean().round(3))
