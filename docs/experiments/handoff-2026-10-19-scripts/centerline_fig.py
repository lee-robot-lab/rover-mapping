import sys; sys.path.insert(0,'.')
import json,numpy as np,cv2
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in font_manager.fontManager.ttflist:
    if 'Noto Sans CJK JP' in f.name: plt.rcParams['font.family']=f.name; break
plt.rcParams['axes.unicode_minus']=False
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from experiments.handheld_mapping.site_mission import SiteMission
m=SiteMission.load(Path('outputs/site_sim/abD1018/mission_bundle'),load_mesh=False); s=m.site
r=s.resolution; x0,y0=s.origin
mask=s.observed&~s.blocked&(s.slope_deg<=m.settings['trial_slope_deg']+1e-4)
D=distance_transform_edt(np.pad(mask,1,constant_values=False))[1:-1,1:-1]*r
rows,cols=D.shape
sk=cv2.ximgproc.thinning((mask&(D>=.12)).astype(np.uint8)*255)>0
def cell(p): c=s.cells(np.asarray(p,float)); return int(c[1]),int(c[0])
def xy(rc): return np.array([x0+(rc[1]+.5)*r,y0+(rc[0]+.5)*r])
idx=-np.ones(D.shape,int); ok=mask&(D>=.10); idx[ok]=np.arange(ok.sum())
I=[];J=[];W=[]
for dr in(-1,0,1):
    for dc in(-1,0,1):
        if not(dr or dc): continue
        a=np.argwhere(ok); b=a+[dr,dc]
        inb=(b[:,0]>=0)&(b[:,0]<rows)&(b[:,1]>=0)&(b[:,1]<cols); a,b=a[inb],b[inb]
        good=ok[b[:,0],b[:,1]]; a,b=a[good],b[good]
        step=np.hypot(dr,dc)*r; dm=np.minimum(D[a[:,0],a[:,1]],D[b[:,0],b[:,1]])
        I+=idx[a[:,0],a[:,1]].tolist();J+=idx[b[:,0],b[:,1]].tolist();W+=(step*(1+(0.4/dm)**2)).tolist()   # 벽에 가까울수록 비싸게
G=coo_matrix((W,(I,J)),shape=(ok.sum(),)*2).tocsr()
pos=np.argwhere(ok)
def center_route(pts):
    out=[]
    for p,q in zip(pts[:-1],pts[1:]):
        a,b=cell(p),cell(q); a=tuple(pos[np.argmin(((pos-a)**2).sum(1))]); b=tuple(pos[np.argmin(((pos-b)**2).sum(1))])
        d,pre=dijkstra(G,indices=idx[a],return_predecessors=True)
        path=[idx[b]]
        while path[-1]!=idx[a]: path.append(pre[path[-1]])
        out+= [pos[k] for k in path[::-1]]
    return np.array([xy(p) for p in out])
conn_old=m.connector()
R30=m.routes['30'];R15=m.routes['15'];R10=m.routes['10']
paths={
 '30° 램프 임무': (np.vstack([conn_old,R30[1:]]), center_route([m.start[:2],m.junction,R30[-1]])),
 '15° 램프': (np.vstack([conn_old,R15[1:]]), center_route([m.start[:2],m.junction]+[p for p in R15[1:]])),
 '10° 램프': (np.vstack([conn_old,R10[1:]]), center_route([m.start[:2],m.junction]+[p for p in R10[1:]])),
}
def prof(path):
    seg=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    c=s.cells(path); return seg, D[np.clip(c[:,1],0,rows-1),np.clip(c[:,0],0,cols-1)]
fig=plt.figure(figsize=(17,9.5)); gs=fig.add_gridspec(2,2,width_ratios=[1.35,1],height_ratios=[1,1])
ax=fig.add_subplot(gs[:,0]); ext=(x0,x0+cols*r,y0,y0+rows*r)
im=ax.imshow(np.where(mask,D,np.nan),origin='lower',extent=ext,cmap='viridis',vmin=0,vmax=.8)
ax.imshow(np.where(~mask,1.,np.nan),origin='lower',extent=ext,cmap='gray_r',vmin=0,vmax=2,alpha=.55)
ys,xs=np.nonzero(sk); ax.plot(x0+(xs+.5)*r,y0+(ys+.5)*r,'.',color='white',ms=2.5,alpha=.8,label='중앙선 망(스켈레톤, 벽까지 거리 ≥0.12 m)')
cols_=['red','orange','magenta']
for (name,(old,cen)),c in zip(paths.items(),cols_):
    ax.plot(old[:,0],old[:,1],'--',color=c,lw=1.6,alpha=.9)
    ax.plot(cen[:,0],cen[:,1],'-',color=c,lw=2.6,label=f'{name}: 실선=중앙 유지 경로 / 점선=기존 경로')
ax.plot(*m.start[:2],'w*',ms=15,mec='k');ax.plot(*m.junction,'wD',ms=9,mec='k');ax.plot(*R30[-1],'wP',ms=11,mec='k')
ax.annotate('시작',m.start[:2],xytext=(6,8),textcoords='offset points',color='w');ax.annotate('갈림길',m.junction,xytext=(6,-14),textcoords='offset points',color='w');ax.annotate('30° 정상',R30[-1],xytext=(6,6),textcoords='offset points',color='w')
ax.set_aspect('equal');ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.set_title('전체 지도의 중앙선(능선)과 중앙 유지 경로 — 통과 가능 영역(경사≤35°), 색=벽까지 거리 [m], 회색=통과 불가');ax.legend(loc='lower left',fontsize=7.5,framealpha=.9)
fig.colorbar(im,ax=ax,shrink=.6,pad=.01).set_label('벽까지 거리 [m]')
for k,(name,(old,cen)) in enumerate(list(paths.items())[:2]):
    a=fig.add_subplot(gs[k,1]); so,do=prof(old); sc,dc=prof(cen)
    a.plot(so,do,'--',color='gray',lw=1.8,label='기존 경로');a.plot(sc,dc,'-',color='tab:blue',lw=2.2,label='중앙 유지 경로')
    a.axhline(.2854,color='red',lw=1.3,label='정지 검사 문턱 0.2854');a.axhline(.3354,color='orange',ls='--',lw=1.3,label='계획 문턱 0.3354')
    a.set_xlabel('경로 길이 [m]');a.set_ylabel('벽까지 거리 [m]');a.set_title(f'{name}: 경로를 따라 본 벽까지 거리');a.set_ylim(0,.55);a.legend(fontsize=8,loc='lower right');a.grid(alpha=.3)
    print(name,'기존 min/평균',do.min().round(3),do.mean().round(3),'| 중앙 min/평균',dc.min().round(3),dc.mean().round(3),'| 길이',so[-1].round(2),'->',sc[-1].round(2))
plt.savefig('outputs/site_sim/centerline_overview.png',dpi=105,bbox_inches='tight'); print('saved')
