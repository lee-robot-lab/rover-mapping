# ================================================================
# update_planning_terrain.py
# 설명: 선택한 배포 장애물 격자를 중계 코스트에 투영하며 배경 지도는 보존한다.
# 사용법: python deployment/broadcast/update_planning_terrain.py
# ================================================================
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def update(data: Path = ROOT/'deployment/broadcast/data') -> None:
    """바구니 구역은 보존하고 본 주행 구역의 코스트를 선택한 격자로 교체한다."""
    mission=ROOT/'deployment/ksrc/mission'
    planning=json.loads((mission/'planning_map.json').read_text())
    site=json.loads((mission/'site/site.json').read_text())
    grid_path=mission/'site/grid.npz'
    assert hashlib.sha256(grid_path.read_bytes()).hexdigest()==planning['grid_sha256']
    assert planning['site_identity']==site['identity']
    with np.load(data/'terrain.npz') as stored:
        terrain={key:stored[key] for key in stored.files}
    with np.load(grid_path) as grid:
        slope=grid['slope_deg'];observed=grid['observed'];blocked=grid['blocked']
    xy=terrain['origin']+np.indices(terrain['cost'].shape).transpose(1,2,0)*float(terrain['cell'])
    indices=np.floor((xy-np.asarray(site['origin']))/site['resolution']).astype(int)
    inside=((indices>=0).all(axis=-1)&(indices[...,0]<slope.shape[1])&(indices[...,1]<slope.shape[0]))
    x,y=indices[inside].T
    terrain['cost'][inside]=np.where(blocked[y,x]|~observed[y,x],1.,np.clip(np.nan_to_num(slope[y,x],nan=35.)/35.,0.,1.))
    terrain['valid'][inside]=observed[y,x]
    # 작은 돌이 표시 격자 사이로 빠지지 않도록 모든 차단 셀을 보수적으로 투영한다.
    yy,xx=np.nonzero(blocked)
    centers=np.asarray(site['origin'])+(np.column_stack((xx,yy))+.5)*site['resolution']
    cells=np.rint((centers-terrain['origin'])/float(terrain['cell'])).astype(int)
    good=(cells>=0).all(axis=1)&(cells<terrain['cost'].shape).all(axis=1)
    terrain['cost'][tuple(cells[good].T)]=1.
    np.savez_compressed(data/'terrain.npz',**terrain)
    meta=json.loads((data/'manifest.json').read_text())
    meta['planning_map']={key:planning[key] for key in ('name','site_identity','grid_sha256','source_sha256','stone_diameter_m','stone_count','stone_centers_xy_m')}
    meta['planning_map']['display_cost']='clip(slope_deg/35,0,1); blocked or unknown=1; display only, not MPPI objective'
    (data/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta['planning_map'],indent=2))


if __name__=='__main__':update()
