exec(open('/tmp/claude-1000/-home-su-rover/2f374bec-eda0-450c-87e7-cb87e1d65215/scratchpad/fillet_lib.py').read())
import shutil
from experiments.handheld_mapping.site_mission import path_with_heading
base=Path('outputs/site_sim/cy_missions/rect_mission'); m=load_rect(); s=m.site; D=make_D(s,35.)
for rad,tag in ((0.25,'rect_fillet25'),(0.30,'rect_fillet30')):
    out=Path('outputs/site_sim/cy_missions')/tag
    if out.exists(): shutil.rmtree(out)
    shutil.copytree(base,out)
    conn=fillet(m.connector(),rad)
    routes={'30':m.routes['30'],'15':fillet(m.routes['15'],rad),'10':fillet(m.routes['10'],rad)}
    new=SiteMission(s,m.start,m.junction,m.summit,{k:np.asarray(v) for k,v in routes.items()},m.settings,m.site_name,connector_waypoints=conn)
    rep=new.validate(); assert rep['accepted'],rep
    mj=json.load(open(out/'mission.json'))
    mj['routes']={k:np.asarray(v).tolist() for k,v in routes.items()}; mj['connector_waypoints']=conn.tolist(); mj['validation']=rep
    json.dump(mj,open(out/'mission.json','w'),indent=2,ensure_ascii=False)
    for name in ('10','15','30'):
        for direction in ('up','down'):
            np.savetxt(out/'routes'/f'{name}_{direction}.csv',path_with_heading(new.path(name,direction)),delimiter=',',header='x_m,y_m,yaw_rad',comments='')
    chk=SiteMission.load(out,load_mesh=False)
    def curv(P):
        P=np.asarray(P); h=np.unwrap(np.arctan2(*np.diff(P,axis=0).T[::-1])); L=np.linalg.norm(np.diff(P,axis=0),axis=1)
        return np.abs(np.diff(h)/((L[:-1]+L[1:])/2)).max()
    print(f"{tag}: R={rad} 연결 {len(conn)}점 15° {len(routes['15'])}점 10° {len(routes['10'])}점 | 최대 곡률 {max(curv(conn),curv(routes['15']),curv(routes['10'])):.2f} 1/m (=1/R {1/rad:.2f}) | validate {rep['accepted']} | reload ok")
    for k,P in (('연결',conn),('15°',routes['15']),('10°',routes['10'])): d=dense_min(s,D,np.asarray(P)); print(f"   {k}: 벽까지 거리 min {d.min():.3f} 평균 {d.mean():.3f}")
