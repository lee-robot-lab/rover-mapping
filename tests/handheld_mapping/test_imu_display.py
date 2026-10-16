"""표시 영점은 보정/지도와 분리하고 수신 회복은 현재 경고에 반영한다."""
import json
import math
import time
from types import SimpleNamespace

import pytest

from experiments.handheld_mapping.imu import ImuCapture,TiltDisplay
from test_imu_capture import CALIBRATION
from test_gravity import samples
from test_imu_session import imu_boundary,config
from test_session import wait_state


def up(pitch):
    angle=math.radians(pitch)
    return [math.sin(angle),0.,math.cos(angle)]


def test_display_smoothing_does_not_erase_static_tilt_and_zero_is_explicit():
    display=TiltDisplay()
    state=display.update(up(5),0,0)
    assert state['display_pitch_deg']==pytest.approx(-5.)
    assert state['display_zero_set'] is False
    display.set_zero(up(5))
    assert display.snapshot()['display_pitch_deg']==pytest.approx(0.)
    for i in range(1,101):state=display.update(up(20),i*20_000_000,0)
    assert state['display_pitch_deg']==pytest.approx(-15.,abs=.01)
    assert state['display_zero_pitch_deg']==pytest.approx(-5.)
    # MCU reset은 이전 영점의 연속성으로 취급하지 않는다.
    state=display.update(up(10),3_000_000_000,1)
    assert state['display_zero_set'] is False
    assert state['display_pitch_deg']==pytest.approx(-10.)


def test_single_acceleration_pulse_is_smoothed_in_pi_time_not_arrival_burst():
    display=TiltDisplay()
    display.update([0.,0.,1.],1_000_000_000,0)
    state=display.update([.1,0.,1.],1_020_000_000,0)
    assert abs(state['display_pitch_deg'])<1.
    with pytest.raises(ValueError):display.update([0.,0.,1.],1_020_000_000,0)


def capture(tmp_path):
    calibration=tmp_path/'source.json'; calibration.write_text(json.dumps(CALIBRATION))
    urdf=tmp_path/'source.urdf'; urdf.write_text('<robot name="fixture"/>')
    imu=ImuCapture('pi@fixture',tmp_path/'run',calibration,urdf)
    imu._prepare()
    imu._last_fresh_ns=time.monotonic_ns()
    imu._status['state']='recording'
    imu._gravity_samples.extend(samples(up=up(3)))
    imu._tilt_display.update(up(3),4_000_000_000,0)
    imu._status.update(imu._tilt_display.snapshot())
    return imu,calibration


def test_zero_keeps_calibration_and_measurement_record_unchanged(tmp_path):
    imu,source=capture(tmp_path)
    before=source.read_bytes(); copied=(imu.run/'imu_calibration.json').read_bytes()
    raw=(imu.run/'imu.jsonl').read_bytes()
    try:
        result=imu.zero_display()
        assert result['display_pitch_deg']==pytest.approx(0.)
        assert result['display_zero_set']
        assert source.read_bytes()==before
        assert (imu.run/'imu_calibration.json').read_bytes()==copied
        assert (imu.run/'imu.jsonl').read_bytes()==raw
        event=json.loads((imu.run/'imu_display_events.jsonl').read_text())
        assert event['kind']=='display_zero' and 'display_only' in event['scope']
    finally:imu.stop()


@pytest.mark.parametrize('problem',['motion','stale','stopping'])
def test_zero_refuses_moving_stale_or_stopping_capture(tmp_path,problem):
    imu,_source=capture(tmp_path)
    try:
        if problem=='motion':imu._gravity_samples[-1]['gyro_dps']=[0.,0.,10.]
        if problem=='stale':imu._last_fresh_ns=time.monotonic_ns()-1_000_000_000
        if problem=='stopping':imu._stopping.set()
        with pytest.raises(ValueError):imu.zero_display()
        assert not (imu.run/'imu_display_events.jsonl').exists()
    finally:imu.stop()


def test_transient_delivery_delay_recovers_warning_and_does_not_alone_mark_partial(tmp_path,imu_boundary):
    from experiments.handheld_mapping.controller import Session
    session=Session(); session.start(config(tmp_path))
    try:
        wait_state(session,{'recording'})
        capture=imu_boundary.instances[-1]
        capture.state['stale']=True
        deadline=time.monotonic()+3
        while not session.snapshot().get('imu_warning') and time.monotonic()<deadline:time.sleep(.02)
        assert session.snapshot()['imu_warning']
        capture.state['stale']=False
        deadline=time.monotonic()+3
        while session.snapshot().get('imu_warning') and time.monotonic()<deadline:time.sleep(.02)
        assert not session.snapshot().get('imu_warning')
    finally:
        session.stop(); session.thread.join(timeout=20)
    assert session.snapshot()['state']=='complete',session.snapshot()


def test_qt_uses_display_angles_and_current_stale_state_and_zero_button():
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    app=QApplication.instance() or QApplication([])
    state={'state':'recording','valid':100,'unique':99,'hz':50.,'stale':False,
           'roll_deg':-1.2,'pitch_deg':2.6,'display_roll_deg':-1.,'display_pitch_deg':2.5}
    calls=[]
    def zero():
        calls.append(True)
        state.update(display_roll_deg=0.,display_pitch_deg=0.,display_zero_set=True)
        return state
    session=Session(); session.imu=SimpleNamespace(snapshot=lambda:dict(state),zero_display=zero)
    session.update(state='recording',imu_warning='previous historical delay')
    window=MappingWindow(session)
    try:
        window.refresh()
        assert 'roll -1.0' in window.imu_status.text()
        assert 'pitch 2.5' in window.imu_status.text()
        assert '수신 지연' not in window.imu_status.text()
        assert window.imu_zero_button.isEnabled()
        window.imu_zero_button.click()
        assert calls==[True]
        assert '평지 영점 기준' in window.imu_status.text()
        state['stale']=True; window.refresh()
        assert '수신 지연' in window.imu_status.text()
        assert not window.imu_zero_button.isEnabled()
        app.processEvents()
    finally:
        session.update(state='idle'); window.close()


def test_delivery_delay_and_source_packet_gap_are_counted_separately(tmp_path,monkeypatch):
    from experiments.handheld_mapping import imu as module
    imu,_=capture(tmp_path)
    imu._ready=True
    imu._gravity_samples.clear(); imu._tilt_display=TiltDisplay()
    arrivals=iter((1_000_000_000,1_020_000_000,2_020_000_000,2_280_000_000))
    original=module.time.monotonic_ns
    monkeypatch.setattr(module.time,'monotonic_ns',lambda:next(arrivals))
    def packet(t_ms,pi_ns):
        return {'kind':'telemetry','schema':1,'host_monotonic_ns':pi_ns,'host_receipt_ns':pi_ns,
                'telemetry':{'flags':16,'t_ms':t_ms,'tick':t_ms%256,
                 'gyro_dps':CALIBRATION['gyro_bias_dps'],
                 'acc_g':[CALIBRATION['acc_offset_g'][i]+(1/CALIBRATION['acc_scale'][i] if i==2 else 0.) for i in range(3)]}}
    try:
        imu._event(packet(100,1_000_000_000))
        imu._event(packet(120,1_020_000_000))
        imu._event(packet(140,1_040_000_000))
        assert imu._status['delivery_delay_events']==1
        assert imu._status['source_gap_events']==0
        imu._event(packet(400,1_300_000_000))
        assert imu._status['delivery_delay_events']==1
        assert imu._status['source_gap_events']==1
        assert imu._status['max_source_gap_ms']==260.
    finally:
        monkeypatch.setattr(module.time,'monotonic_ns',original)
        imu.stop()


@pytest.mark.parametrize('fault',['source_gap_events','tee_drops','resets','clock_jumps'])
def test_recovered_transport_does_not_hide_actual_recording_faults(tmp_path,imu_boundary,fault):
    from experiments.handheld_mapping.controller import Session
    session=Session(); session.start(config(tmp_path))
    try:
        wait_state(session,{'recording'})
        imu_boundary.instances[-1].state[fault]=1
        time.sleep(.15)
    finally:
        session.stop(); session.thread.join(timeout=20)
    assert session.snapshot()['state']=='partial',session.snapshot()
