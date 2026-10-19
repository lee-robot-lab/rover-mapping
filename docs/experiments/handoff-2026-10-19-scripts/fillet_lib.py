import sys; sys.path.insert(0,'.')
import json,numpy as np
from pathlib import Path
from scipy.ndimage import distance_transform_edt
from experiments.handheld_mapping.site_mission import SiteMission
def load_rect():
    return SiteMission.load(Path('outputs/site_sim/cy_missions/rect_mission'),load_mesh=False)
def decollinear(P):
    P=[np.asarray(p,float) for p in P]; out=[P[0]]
    for a,b,c in zip(P[:-2],P[1:-1],P[2:]):
        u,v=b-a,c-b
        if abs(u[0]*v[1]-u[1]*v[0])>1e-9 or np.dot(u,v)<0: out.append(b)
    out.append(P[-1]); return np.array(out)
def fillet(P,R,step_deg=15.):
    """90° 코너(U턴 제외)를 반지름 R 원호로 바꾼다. 인접 구간이 모자라면 ValueError."""
    P=decollinear(P); out=[P[0]]
    for i in range(1,len(P)-1):
        a,v,c=P[i-1],P[i],P[i+1]; d1=(v-a)/np.linalg.norm(v-a); d2=(c-v)/np.linalg.norm(c-v)
        cross=d1[0]*d2[1]-d1[1]*d2[0]; dot=float(np.dot(d1,d2))
        if abs(cross)<1e-9 or dot<-0.5:      # 직진 또는 U턴(≥120°)은 그대로
            out.append(v); continue
        th=np.arccos(np.clip(dot,-1,1)); t=R*np.tan(th/2)
        if t>np.linalg.norm(v-a)/(2 if i>1 else 1)+1e-9 and t>np.linalg.norm(v-a)-1e-9 or t>np.linalg.norm(c-v)-1e-9:
            raise ValueError(f'segment too short for R={R} at {v.tolist()}')
        A=v-d1*t; B=v+d2*t; s=np.sign(cross); n=s*np.array([-d1[1],d1[0]]); C=A+n*R
        a0=np.arctan2(*(A-C)[::-1]); N=max(2,int(np.ceil(np.degrees(th)/step_deg)))
        for k in range(N+1):
            ang=a0+s*th*k/N; out.append(C+R*np.array([np.cos(ang),np.sin(ang)]))
    out.append(P[-1]); out=np.array(out)
    keep=np.r_[True,np.linalg.norm(np.diff(out,axis=0),axis=1)>1e-6]; return out[keep]
def make_D(site,slope):
    r=site.resolution;clear=site.observed&~site.blocked&(site.slope_deg<=slope+1e-4)
    return distance_transform_edt(np.pad(clear,1,constant_values=False))[1:-1,1:-1]*r
def dense_min(site,D,P):
    out=[]
    for u,v in zip(P[:-1],P[1:]):
        for t in np.linspace(0,1,max(2,int(np.linalg.norm(v-u)/.01))):
            p=u+(v-u)*t;c=site.cells(p);out.append(D[c[1],c[0]])
    return np.array(out)
