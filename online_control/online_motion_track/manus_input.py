"""Session-bound Manus input using the shared SDK connection."""
import time

from .metric_input import MetricInputRelay
from .manus_source import ManusSource, STALE_AFTER_MS, SOURCE


class ManusInputRelay(MetricInputRelay):
    stale_after_ms = STALE_AFTER_MS

    def __init__(self, *, clock=time.monotonic, **options):
        super().__init__(ManusSource(clock=clock, **options), SOURCE, clock=clock,
            stale_after_ms=STALE_AFTER_MS, description=dict(transport='manus_sdk'))
