exec(open('/tmp/claude-1000/-home-su-rover/2f374bec-eda0-450c-87e7-cb87e1d65215/scratchpad/fillet_lib.py').read())
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in font_manager.fontManager.ttflist:
    if 'Noto Sans CJK JP' in f.name: plt.rcParams['font.family']=f.name; break
plt.rcParams['axes.unicode_minus']=False
m=load_rect(); s=m.site; r=s.resolution; x0,y0=s.origin
D=make_D(s,35.); rows,cols=D.shape; ext=(x0,x0+cols*r,y0,y0+rows*r)
mask=s.observed&~s.blocked&(s.slope_deg<=35.0001)
def get(tag):
    mm=SiteMission.load(Path('outputs/site_sim/cy_missions')/tag,load_mesh=False); return mm.connector(),mm.routes
sharp=(m.connector(),m.routes); f25=get('rect_fillet25'); f30=get('rect_fillet30')
fig,ax=plt.subplots(1,2,figsize=(17,8),gridspec_kw={'width_ratios':[1.45,1]})
a=ax[0]
a.imshow(np.where(mask,D,np.nan),origin='lower',extent=ext,cmap='viridis',vmin=0,vmax=.8)
a.imshow(np.where(~mask,1.,np.nan),origin='lower',extent=ext,cmap='gray_r',vmin=0,vmax=2,alpha=.55)
c,Rr=f25
a.plot(c[:,0],c[:,1],'-',color='cyan',lw=3.4,label='연결 경로(R=0.25)')
for k,col,lw in (('10','magenta',2.0),('15','orange',2.6),('30','red',3.2)): a.plot(Rr[k][:,0],Rr[k][:,1],'-',color=col,lw=lw,label=f'{k}° 경로(R=0.25)')
a.plot(*m.start[:2],'w*',ms=15,mec='k');a.plot(*m.junction,'wD',ms=9,mec='k');a.plot(*m.summit,'wP',ms=11,mec='k')
a.set_aspect('equal');a.set_xlabel('x [m]');a.set_ylabel('y [m]');a.set_title('직각을 필렛(반지름 0.25 m)으로 다듬은 전체 경로');a.legend(loc='lower left',fontsize=8)
b=ax[1]
b.imshow(np.where(mask,D,np.nan),origin='lower',extent=ext,cmap='viridis',vmin=0,vmax=.8)
b.imshow(np.where(~mask,1.,np.nan),origin='lower',extent=ext,cmap='gray_r',vmin=0,vmax=2,alpha=.55)
for (cc,_),col,lab,lw in ((sharp,'white','직각(필렛 없음)',3),(f25,'cyan','R=0.25',3),(f30,'yellow','R=0.30',2)):
    b.plot(cc[:,0],cc[:,1],'-o',color=col,lw=lw,ms=3,label=lab)
b.set_xlim(3.3,4.1);b.set_ylim(0.2,1.5);b.set_aspect('equal');b.set_title('시작 코너 확대 (연결 경로: (3.7, 1.2) 코너)');b.legend(fontsize=9,loc='upper left');b.set_xlabel('x [m]')
plt.savefig('outputs/site_sim/rect_fillet.png',dpi=105,bbox_inches='tight');print('saved')
