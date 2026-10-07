"""Causal metric-landmark smoothing for browser A3 tracking; no extrapolation."""
import math
import numpy as np


class ResponsiveLandmarkFilter:
    """One Euro variant with a per-point speed and a stationary-noise floor.

    Input coordinates are metres. A scalar weight per 3-D point avoids different
    response speeds on x/y/z. No future samples or extrapolation are used.
    """

    # Keep the slow-motion time constant within 5% of the previous 1.2 Hz
    # default: 1.2 / 1.17 = 1.02565. Slightly raise beta to preserve onset
    # response; the separate step/phase-delay benchmark enforces the 5% budget.
    def __init__(self, *, min_cutoff=1.17, beta=82., derivative_cutoff=1.,
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
        # Same One Euro alpha, f / (f + 1/(2*pi*dt)); share the scalar term
        # between derivative and position filtering, without another stage.
        inverse_period=1. / (2.*math.pi*dt)
        alpha=self.derivative_cutoff/(self.derivative_cutoff+inverse_period)
        self.velocity+=alpha*(derivative-self.velocity)
        speed=np.maximum(np.sqrt(np.einsum('ij,ij->i',self.velocity,self.velocity))[:,None]-self.noise_speed,0.)
        cutoff=np.minimum(self.min_cutoff+self.beta*speed,self.max_cutoff)
        weight=cutoff/(cutoff+inverse_period)
        self.value+=weight*(raw-self.value)
        self.raw[:]=raw;self.at=timestamp_s
        return self.value.copy()


class A32LandmarkFilter(ResponsiveLandmarkFilter):
    """Metric A3.2 filter with fast motion response and unchanged resting cutoff.

    The old beta=0.25 was nearly constant at 1.5 Hz for metre-scale points.
    Keep that 1.5 Hz stationary cutoff, suppress derivative noise below 20 mm/s,
    and adapt to measured point speed. The existing 15 Hz cap and convex update
    prevent prediction/overshoot; actuator velocity and collision limits remain
    downstream. These settings are shared by MediaPipe, WebXR and Wuji inputs.
    """
    def __init__(self):
        super().__init__(min_cutoff=1.5, beta=82., derivative_cutoff=1.,
                         noise_speed=.02, max_cutoff=15.)
