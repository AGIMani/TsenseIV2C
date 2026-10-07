"""Causal metric-landmark smoothing for browser A3 tracking; no extrapolation."""
import math
import numpy as np


class ResponsiveLandmarkFilter:
    """One Euro variant with a per-point speed and a stationary-noise floor.

    Input coordinates are metres. A scalar weight per 3-D point avoids different
    response speeds on x/y/z. No future samples or extrapolation are used.
    """

    def __init__(self, *, min_cutoff=1.2, beta=80., derivative_cutoff=1.,
                 noise_speed=.01, max_cutoff=15.):
        values=(min_cutoff,beta,derivative_cutoff,noise_speed,max_cutoff)
        if not all(math.isfinite(v) for v in values) or min_cutoff<=0 or beta<0 or derivative_cutoff<=0 or noise_speed<0 or max_cutoff<min_cutoff:
            raise ValueError('Invalid landmark filter configuration')
        self.min_cutoff=min_cutoff;self.beta=beta;self.derivative_cutoff=derivative_cutoff
        self.noise_speed=noise_speed;self.max_cutoff=max_cutoff
        self.reset()

    def reset(self):
        self.at=None;self.raw=None;self.value=None;self.velocity=None

    @staticmethod
    def alpha(cutoff, dt):
        return 1. / (1. + 1. / (2.*math.pi*cutoff*dt))

    def filter(self, points, timestamp_s):
        raw=np.asarray(points,dtype=np.float64)
        if raw.shape!=(21,3) or not np.isfinite(raw).all() or not math.isfinite(timestamp_s):
            raise ValueError('Expected finite 21x3 metric landmarks and timestamp')
        if self.at is not None and timestamp_s<=self.at:
            raise ValueError('Landmark timestamps must increase')
        if self.at is None or timestamp_s-self.at>.3:
            self.at=timestamp_s;self.raw=raw.copy();self.value=raw.copy();self.velocity=np.zeros_like(raw)
            return self.value.copy()
        dt=timestamp_s-self.at
        derivative=(raw-self.raw)/dt
        alpha=self.alpha(self.derivative_cutoff,dt)
        self.velocity+=alpha*(derivative-self.velocity)
        speed=np.maximum(np.linalg.norm(self.velocity,axis=1,keepdims=True)-self.noise_speed,0.)
        cutoff=np.minimum(self.min_cutoff+self.beta*speed,self.max_cutoff)
        weight=self.alpha(cutoff,dt)
        self.value+=weight*(raw-self.value)
        self.raw[:]=raw;self.at=timestamp_s
        return self.value.copy()
