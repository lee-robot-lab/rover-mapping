# ================================================================
# test_vo_evaluation.py
# 설명: VO 평가가 하나의 초기 강체 정합만 적용하고 scale/drift를 숨기지 않는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_vo_evaluation.py
# ================================================================
import numpy as np
from experiments.site_sim.vo_evaluation import align_trajectory


def test_single_initial_alignment_preserves_scale_error():
    gt=np.repeat(np.eye(4)[None],3,axis=0);gt[:,0,3]=[0,1,2]
    est=gt.copy();est[:,0,3]=[10,12,14]
    aligned=align_trajectory(est,gt[0])
    assert np.allclose(aligned[:,0,3],[0,2,4])
    assert np.linalg.norm(aligned[-1,:3,3]-gt[-1,:3,3])==2.


def test_initial_camera_frame_rotation_is_applied_to_all_poses():
    est=np.repeat(np.eye(4)[None],2,axis=0);est[1,0,3]=1
    first=np.eye(4);first[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];first[:3,3]=[3,4,5]
    aligned=align_trajectory(est,first)
    assert np.allclose(aligned[1,:3,3],[3,5,5])
