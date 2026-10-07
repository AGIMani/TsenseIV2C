"""A3.2 causal input response across all three sources; no hardware."""
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'tools')]
from online_control.online_motion_track.landmark_filter import A32LandmarkFilter
from online_control.online_motion_track.main import WebSession, _OneEuroLandmarkFilter
from benchmark_gesture_filters import phase_lag

@pytest.mark.parametrize('rate',[30,60,90,120])
def test_response_improves_without_stationary_noise_or_overshoot_regression(rate):
    rng=np.random.default_rng(20260930+rate)
    t=np.arange(0,12,1/rate)
    truth=np.zeros((len(t),21,3))
    moving=(t>=3)&(t<9)
    truth[moving,:,0]=(.025*np.sin(2*np.pi*.8*(t[moving]-3)))[:,None]
    noisy=truth+rng.normal(0,.0005,truth.shape)
    rest=((t>1)&(t<2.9))|(t>10.5)
    old=_OneEuroLandmarkFilter(min_cutoff=1.5,beta=.25,derivative_cutoff=1.)
    new=A32LandmarkFilter()
    baseline=np.array([old.filter(p,float(at)) for p,at in zip(noisy,t)])
    actual=np.array([new.filter(p,float(at)) for p,at in zip(noisy,t)])
    mse=lambda y,mask:np.mean((y[mask]-truth[mask])**2)
    assert mse(actual,rest)<=mse(baseline,rest)*1.01
    assert mse(actual,moving)<mse(baseline,moving)*.25
    delays=[phase_lag(t[moving],truth[moving,0,0],np.mean(y[moving,:,0],axis=1))
            for y in (baseline,actual)]
    assert delays[1]<=delays[0]*.5
    new.reset()
    previous=new.filter(np.zeros((21,3)),0.)
    for i in range(1,150):
        raw=rng.normal(0,.05,(21,3))
        current=new.filter(raw,i/rate)
        assert np.isfinite(current).all()
        assert np.all(current>=np.minimum(raw,previous)-1e-14)
        assert np.all(current<=np.maximum(raw,previous)+1e-14)
        previous=current

@pytest.mark.parametrize('source',['mediapipe','webxr','wuji_glove'])
@pytest.mark.parametrize('side',['left','right'])
def test_real_pipeline_uses_metric_filter_and_recovers_after_loss(source,side):
    session=WebSession(side=side,recording=False,model_id='TsenseI_v3_2_withid',
                       collision_mode='off',input_source=source)
    try:
        assert isinstance(session.filter,A32LandmarkFilter)
        raw=session.pipeline.session.retargeter.fk.compute()
        def packet(sequence,points):
            return dict(sequence=sequence,timestamp_ms=1000+sequence*16.6667,
                        capture_unix_ms=1789600000000+sequence*16.6667,source=source,
                        hand=None if points is None else
                        dict(label=side.title(),world=points.tolist(),normalized=points.tolist()))
        first=session.process(packet(0,raw))
        assert first['tracking']
        for sequence in (1,3,5):
            result=session.process(packet(sequence,raw+sequence*.0001))
            assert result['tracking'] and np.isfinite(list(result['pose'].values())).all()
        assert not session.process(packet(6,None))['tracking']
        recovered=session.process(packet(7,raw))
        assert recovered['tracking']
        np.testing.assert_allclose(recovered['points'],first['points'],atol=1e-7)
    finally:
        session.close()

def test_tiny_motion_keeps_previous_resting_response_and_does_not_predict():
    f=A32LandmarkFilter()
    assert f.min_cutoff==1.5
    original=np.zeros((21,3))
    assert np.array_equal(f.filter(original,0.),original)
    f.filter(original+.00001,.01)
    f.reset()
    assert np.array_equal(f.filter(original,.02),original)
    with pytest.raises(ValueError):
        f.filter(np.full((21,3),np.nan),.03)
