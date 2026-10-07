"""Compare causal filters offline on recorded and seeded synthetic landmarks."""
import argparse,json,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'src/tracking')]
import numpy as np
from wuji_mediapipe import _OneEuroLandmarkFilter
from online_control.online_motion_track.landmark_filter import ResponsiveLandmarkFilter


class Fixed:
    def __init__(self,hz):self.hz=hz;self.reset()
    def reset(self):self.value=None;self.at=None
    def filter(self,p,t):
        p=np.asarray(p)
        if self.at is None:self.value=p.copy()
        else:self.value+=(p-self.value)/(1+1/(2*np.pi*self.hz*(t-self.at)))
        self.at=t;return self.value.copy()


class AlphaBeta:
    def __init__(self,a=.65,b=.08):self.a=a;self.b=b;self.reset()
    def reset(self):self.p=None;self.v=None;self.at=None
    def filter(self,p,t):
        p=np.asarray(p)
        if self.at is None:self.p=p.copy();self.v=np.zeros_like(p)
        else:
            dt=t-self.at;pred=self.p+self.v*dt;r=p-pred
            self.p=pred+self.a*r;self.v+=self.b/dt*r
        self.at=t;return self.p.copy()


def factories():
    f={'legacy':lambda:_OneEuroLandmarkFilter(min_cutoff=1.5,beta=.25,derivative_cutoff=1.),
       'fixed_4hz':lambda:Fixed(4.),'alpha_beta':lambda:AlphaBeta()}
    for beta in [10.,20.,40.,80.]:
        f['one_euro_beta_'+str(int(beta))]=lambda b=beta:_OneEuroLandmarkFilter(min_cutoff=1.5,beta=b,derivative_cutoff=1.)
        f['responsive_'+str(int(beta))]=lambda b=beta:ResponsiveLandmarkFilter(beta=b)
    return f


def phase_lag(t,src,out,upper=.25,segments=None):
    grid=np.arange(t[0]+upper,t[-1],.005)
    if segments is not None:
        # Never interpolate through a lost hand or correlate across filter resets.
        valid=np.zeros(len(grid),dtype=bool)
        segments=np.asarray(segments)
        for segment in np.unique(segments):
            sample_t=t[segments==segment]
            valid|=(grid>=sample_t[0]+upper)&(grid<=sample_t[-1])
        grid=grid[valid]
    if len(grid)<4:return None
    y=np.interp(grid,t,out)
    if np.std(y)<1e-8:return 0
    delays=np.arange(0,upper+.0001,.005)
    scores=[np.corrcoef(np.interp(grid-d,t,src),y)[0,1] for d in delays]
    return float(delays[int(np.nanargmax(scores))]*1000)


def apply(factory,p,t):
    f=factory();out=[];times=[]
    for point,at in zip(p,t):
        st=time.perf_counter_ns();out.append(f.filter(point,float(at)));times.append((time.perf_counter_ns()-st)/1000)
    return np.array(out),float(np.median(times[20:]))


def flex(p,a,b,c):
    u=p[:,b]-p[:,a];v=p[:,c]-p[:,b]
    return np.arccos(np.clip(np.sum(u*v,axis=1)/np.maximum(1e-15,np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1)),-1,1))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();fs=factories();report={'input':str(args.input),'synthetic':{},'recorded':{}}
    rows=[]
    for line in args.input.open(encoding='utf-8-sig'):
        r=json.loads(line)
        if r.get('type')=='mediapipe_raw':rows.append(r)
    for name,factory in fs.items():
        f=factory();raw=[];out=[];ts=[];last=-1.;cost=[];segments=[];segment=0;lost=True
        for r in rows:
            ix=next((i for i,x in enumerate(r['handedness']) if x and x[0]['label']=='Left'),None)
            if ix is None:f.reset();lost=True;continue
            at=r['timestamp_ms']/1000
            if lost or at-last>.3:f.reset();segment+=1
            p=np.asarray(r['world_landmarks'][ix]);st=time.perf_counter_ns();filtered=f.filter(p,at);cost.append((time.perf_counter_ns()-st)/1000)
            raw.append(p);out.append(filtered);ts.append(at);segments.append(segment);last=at;lost=False
        raw=np.asarray(raw);out=np.asarray(out);ts=np.asarray(ts)
        report['recorded'][name]={'median_compute_us':float(np.median(cost)),
          'valid_frames':len(ts),'continuous_segments':segment,
          'pinky_PIP_lag_ms':phase_lag(ts,flex(raw,17,18,19),flex(out,17,18,19),segments=segments),
          'pinky_DIP_lag_ms':phase_lag(ts,flex(raw,18,19,20),flex(out,18,19,20),segments=segments)}
    for rate in [15,30,60]:
        rng=np.random.default_rng(20260922+rate);t=np.arange(0,12,1/rate)
        base=np.zeros((len(t),21,3));truth=base.copy()
        moving=(t>=3)&(t<9);truth[moving,:,0]=(.025*np.sin(2*np.pi*.8*(t[moving]-3)))[:,None]
        noisy=truth+rng.normal(0,.0005,truth.shape)
        for name,factory in fs.items():
            y,us=apply(factory,noisy,t)
            rest=((t>1)&(t<2.9))|(t>10.5)
            # No future lookahead: each candidate sees the same noisy prefix.
            lag=phase_lag(t[moving],truth[moving,0,0],np.mean(y[moving,:,0],axis=1))
            # Abrupt 30mm step: quantify extrapolating filters' overshoot.
            step=np.zeros_like(truth);step[t>=3,:,0]=.03
            ys,_=apply(factory,step,t)
            report['synthetic'].setdefault(str(rate),{})[name]={'lag_ms':lag,
              'rest_rmse_mm':float(np.sqrt(np.mean((y[rest]-truth[rest])**2))*1000),
              'moving_rmse_mm':float(np.sqrt(np.mean((y[moving]-truth[moving])**2))*1000),
              'step_overshoot_mm':float(max(0,ys[:,:,0].max()-.03)*1000),'median_compute_us':us}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('name recorded_PIP/DIP_ms synthetic30_lag_ms rest_rmse_mm moving_rmse_mm overshoot_mm')
    for name in fs:
        r=report['recorded'][name];s=report['synthetic']['30'][name]
        print(name,r['pinky_PIP_lag_ms'],r['pinky_DIP_lag_ms'],*[round(s[k],4) for k in ['lag_ms','rest_rmse_mm','moving_rmse_mm','step_overshoot_mm']])


if __name__=='__main__':main()
