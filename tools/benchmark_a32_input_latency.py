"""Offline causal filter comparison; seeded signals only, no devices."""
import argparse,sys,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'tools'),str(ROOT/'src/tracking')]
import numpy as np
from wuji_mediapipe import _OneEuroLandmarkFilter
from online_control.online_motion_track.landmark_filter import A32LandmarkFilter
from benchmark_gesture_filters import phase_lag,apply
factories={
'previous_a32':lambda:_OneEuroLandmarkFilter(min_cutoff=1.5,beta=.25,derivative_cutoff=1.),
'responsive_a32':A32LandmarkFilter,
}
result={'method':'identical seeded causal input, no hardware; response delay is separate from compute time','rates':{}}
for rate in [30,60,90,120]:
    rng=np.random.default_rng(20260930+rate)
    t=np.arange(0,12,1/rate)
    truth=np.zeros((len(t),21,3));moving=(t>=3)&(t<9)
    truth[moving,:,0]=(.025*np.sin(2*np.pi*.8*(t[moving]-3)))[:,None]
    noisy=truth+rng.normal(0,.0005,truth.shape)
    rest=((t>1)&(t<2.9))|(t>10.5)
    step=np.zeros_like(truth);step[t>=3,:,0]=.03
    result['rates'][str(rate)]={}
    for name,factory in factories.items():
        y,us=apply(factory,noisy,t);ys,_=apply(factory,step,t)
        idx=np.flatnonzero(t>=3)[0]
        def cross(fraction):
            a=np.mean(ys[:,:,0],axis=1)
            end=next(i for i in range(idx,len(a)) if a[i]>=.03*fraction)
            if end==idx:return 0.
            return (np.interp(.03*fraction,[a[end-1],a[end]],[t[end-1],t[end]])-t[idx])*1000
        result['rates'][str(rate)][name]={
          'sine_phase_delay_ms':phase_lag(t[moving],truth[moving,0,0],np.mean(y[moving,:,0],axis=1)),
          'stationary_rmse_mm':float(np.sqrt(np.mean((y[rest]-truth[rest])**2))*1000),
          'moving_rmse_mm':float(np.sqrt(np.mean((y[moving]-truth[moving])**2))*1000),
          'step50_ms':cross(.5),'step90_ms':cross(.9),
          'step_overshoot_mm':float(max(0,ys[:,:,0].max()-.03)*1000),
          'median_compute_us':us,
        }
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path)
args=parser.parse_args()
if args.output:
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
