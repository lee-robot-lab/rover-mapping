# ================================================================
# test_numpy_policy.py
# 설명: torch 없이 쓰는 NumpyMlpPolicy가 정규화기와 Linear/ELU MLP를 손으로 계산한 값과 같은지, npz 저장·읽기가 보존되는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_numpy_policy.py
# ================================================================
import numpy as np

from experiments.site_sim.rl_residual import NumpyMlpPolicy


def elu(x):
    return x if x > 0 else float(np.expm1(x))


def test_matches_hand_computed_two_layer_network(tmp_path):
    mean, std, eps = np.array([1., 2.]), np.array([.5, 1.]), .01
    w0, b0 = np.array([[1., 0.], [0., -1.], [.5, .5]]), np.array([0., .1, -.2])
    w1, b1 = np.array([[1., 1., 1.], [0., 2., -1.]]), np.array([.3, 0.])
    np.savez(tmp_path/'p.npz', mean=mean, std=std, eps=np.array(eps), W0=w0, b0=b0, W1=w1, b1=b1)
    policy = NumpyMlpPolicy.load(tmp_path/'p.npz')
    obs = np.array([[1.5, 0.]])
    z = (obs[0]-mean)/(std+eps)
    h = np.array([elu(v) for v in w0@z+b0])
    expected = w1@h+b1
    assert np.allclose(policy(obs), expected, atol=1e-6)
    assert policy(obs).shape == (2,) and policy(obs).dtype == np.float32
    assert np.allclose(policy(obs[0]), policy(obs))


def test_elu_negative_branch_is_continuous_and_bounded(tmp_path):
    np.savez(tmp_path/'q.npz', mean=np.zeros(1), std=np.ones(1), eps=np.array(0.), W0=np.array([[1.]]), b0=np.zeros(1),
             W1=np.array([[1.]]), b1=np.zeros(1))
    policy = NumpyMlpPolicy.load(tmp_path/'q.npz')
    assert np.isclose(policy(np.array([-1000.]))[0], -1.0) and np.isclose(policy(np.array([2.]))[0], 2.0)
    assert abs(policy(np.array([1e-9]))[0]-policy(np.array([-1e-9]))[0]) < 1e-6
