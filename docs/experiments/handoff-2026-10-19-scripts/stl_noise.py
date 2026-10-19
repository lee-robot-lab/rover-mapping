import sys; sys.path.insert(0,'.')
import numpy as np
from pathlib import Path
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
for f in font_manager.fontManager.ttflist:
    if 'Noto Sans CJK JP' in f.name: plt.rcParams['font.family']=f.name; break
plt.rcParams['axes.unicode_minus']=False
from experiments.handheld_mapping.site_mission import SiteMission
m=SiteMission.load(Path('outputs/site_sim/abD1018/mission_bundle'),load_mesh=False); s=m.site
r=s.resolution;x0,y0=s.origin;H=s.height_m;S=s.slope_deg
rows,cols=H.shape;X=x0+(np.arange(cols)+.5)*r;Y=y0+(np.arange(rows)+.5)*r
def region(xa,xb,ya,yb):
    xs=(X>=xa)&(X<=xb);ys=(Y>=ya)&(Y<=yb);return H[np.ix_(ys,xs)],S[np.ix_(ys,xs)]
# 평평해야 하는 구간들
for name,(xa,xb,ya,yb) in {'연결 통로 바닥(x2.9~3.6,y1.0~1.4)':(2.9,3.6,1.0,1.4),'갈림길 앞 바닥(x3.2~3.6,y0.3~0.9)':(3.2,3.6,0.3,0.9),'30° 램프 차선 바닥(x3.3~4.0,y0.2~0.6)':(3.3,4.0,0.2,0.6)}.items():
    h,sl=region(xa,xb,ya,yb); ok=np.isfinite(h)
    print(f"{name}: 높이 평균 {np.nanmean(h):.3f} 표준편차 {np.nanstd(h)*1000:.1f} mm, 범위 {np.nanmin(h):.3f}~{np.nanmax(h):.3f}, 경사 중앙 {np.nanmedian(sl):.1f}° p90 {np.nanpercentile(sl,90):.1f}° 최대 {np.nanmax(sl):.1f}°, 칸 {ok.sum()}")
# 램프 위: 이론 경사 대비 측정 경사 산포
for name,(xa,xb,ya,yb,ang) in {'30° 램프(x4.2~5.2,y0.15~0.65) 이론 30°':(4.2,5.2,0.15,0.65,30),'15° 램프(x4.2~5.2,y0.95~1.45) 이론 15°':(4.2,5.2,0.95,1.45,15)}.items():
    h,sl=region(xa,xb,ya,yb); print(f"{name}: 측정 경사 중앙 {np.nanmedian(sl):.1f}° 표준편차 {np.nanstd(sl):.1f}° p5~p95 {np.nanpercentile(sl,5):.1f}~{np.nanpercentile(sl,95):.1f}°")
# 높이 이웃 칸 차이(국소 거칠기) 분포 - 평지에서
dx=np.abs(np.diff(H,axis=1));print('이웃 칸 높이차 [mm] p50/p90/p99/max',(np.nanpercentile(dx,[50,90,99])*1000).round(1),np.nanmax(dx)*1000)
fig,ax=plt.subplots(1,3,figsize=(18,5.6))
ext=(x0,x0+cols*r,y0,y0+rows*r)
a=ax[0];im=a.imshow(H,origin='lower',extent=ext,cmap='terrain');a.set_title('높이 지도 [m]');fig.colorbar(im,ax=a,shrink=.8)
a=ax[1];im=a.imshow(S,origin='lower',extent=ext,cmap='magma',vmin=0,vmax=35);a.set_title('경사 지도 [°] (평지가 얼룩덜룩하면 노이즈)');fig.colorbar(im,ax=a,shrink=.8)
# 국소 거칠기: 이웃 높이차(평지에서만 의미)
rough=np.zeros_like(H);rough[:,1:]=np.abs(np.diff(H,axis=1));rough[1:,:]=np.maximum(rough[1:,:],np.abs(np.diff(H,axis=0)))
a=ax[2];im=a.imshow(rough*1000,origin='lower',extent=ext,cmap='viridis',vmin=0,vmax=15);a.set_title('이웃 칸 높이차 [mm] (거칠기)');fig.colorbar(im,ax=a,shrink=.8)
for a in ax: a.set_xlabel('x [m]');a.set_ylabel('y [m]')
plt.savefig('outputs/site_sim/stl_noise.png',dpi=100,bbox_inches='tight');print('saved')
