"""Loss, controlled stop and reacquisition shared by live control and replay.

The owner serializes these calls with sampling.  A held display skeleton is a
missing observation, never a new target.  No state transition resets the robot
to zero or treats an unexecuted target as the current hand pose.
"""

from __future__ import annotations

from typing import Any, Mapping


class TrackingTransition:
    def __init__(
        self,
        interpolator: Any,
        *,
        confirmation_frames: int = 3,
        recovery_duration_s: float = 1.0,
        observation_timeout_s: float = 0.30,
    ) -> None:
        self.interpolator = interpolator
        self.confirmation_frames = max(1, int(confirmation_frames))
        self.recovery_duration_s = max(0.01, float(recovery_duration_s))
        self.observation_timeout_s = max(0.05, float(observation_timeout_s))
        self.phase = "waiting"
        self.reason: str | None = None
        self.generation = 0
        self.loss_count = 0
        self.recovery_count = 0
        self.valid_streak = 0
        self._interrupted = True
        self._last_valid_time: float | None = None
        self._recovery_started: float | None = None
        self.speed_scale = 1.0

    def _set_speed(self, scale: float) -> None:
        self.speed_scale = float(scale)
        setter = getattr(self.interpolator, "set_speed_scale", None)
        if callable(setter):
            setter(scale)

    def _hold(self, current: Mapping[str, float] | None) -> None:
        # For explicit interlocks (wrong hand / Stop) no further sample may be
        # sent.  Discard hidden derivatives as well as the old destination.
        if not current:
            # A target can be interrupted before the first control tick. Peek
            # is the OTG's start state, not its unexecuted destination.
            peek = getattr(self.interpolator, "peek", None)
            current = peek() if callable(peek) else None
        if current:
            sync = getattr(self.interpolator, "sync_current_pose", None)
            if callable(sync):
                sync(dict(current))
            self.interpolator.push(dict(current))
        else:
            self.interpolator.reset()

    def block(self, reason: str, current: Mapping[str, float] | None) -> None:
        if self.phase != "blocked":
            self.generation += 1
        self._hold(current)
        self.phase = "blocked"
        self.reason = reason
        self.valid_streak = 0
        self._interrupted = True
        self._last_valid_time = None

    def reset(self, current: Mapping[str, float] | None = None) -> None:
        self.block("tracking stopped", current)
        self.phase = "waiting"
        self.reason = None
        self._recovery_started = None

    def missing(self, reason: str, current: Mapping[str, float] | None) -> None:
        self.valid_streak = 0
        if self.phase == "blocked":
            # Absence cannot reopen a hard handedness / user-stop interlock.
            return
        self.reason = reason
        if self._interrupted:
            if self.phase == "confirming":
                self.phase = "holding" if current else "waiting"
            return
        self._interrupted = True
        self.generation += 1
        self.loss_count += 1
        self._recovery_started = None
        brake = getattr(self.interpolator, "brake", None)
        if callable(brake) and (current or getattr(self.interpolator, "ready", False)):
            brake()
            self.phase = "braking"
        else:
            self._hold(current)
            self.phase = "holding" if current else "waiting"

    def observe(
        self,
        valid: bool,
        *,
        now: float,
        current: Mapping[str, float] | None,
        reason: str = "hand_missing",
    ) -> dict[str, Any]:
        if not valid:
            self.missing(reason, current)
            return self._decision(False)

        if (
            self._last_valid_time is not None
            and float(now) - self._last_valid_time > self.observation_timeout_s
        ):
            self.missing("observation_timeout", current)
        self._last_valid_time = float(now)
        self.valid_streak = min(self.valid_streak + 1, self.confirmation_frames)
        if self._interrupted:
            if self.phase == "braking" and not bool(
                getattr(self.interpolator, "stationary", False)
            ):
                return self._decision(False)
            if self.valid_streak < self.confirmation_frames:
                self.phase = "confirming"
                return self._decision(False)
            self._hold(current)
            self._set_speed(0.35)
            self.phase = "recovering"
            self.reason = None
            self._interrupted = False
            self._recovery_started = float(now)
            self.recovery_count += 1
            return self._decision(True, reset_planner=True)
        return self._decision(True)

    def tick(self, *, now: float, current: Mapping[str, float] | None) -> None:
        if (
            not self._interrupted
            and self._last_valid_time is not None
            and float(now) - self._last_valid_time > self.observation_timeout_s
        ):
            self.missing("observation_timeout", current)
        if self.phase == "braking" and bool(
            getattr(self.interpolator, "stationary", False)
        ):
            self.phase = "holding"
        if self.phase == "recovering" and self._recovery_started is not None:
            fraction = min(
                1.0, max(0.0, (float(now) - self._recovery_started) / self.recovery_duration_s)
            )
            # Smoothstep prevents a discontinuity in the rate of limit changes.
            blend = fraction * fraction * (3.0 - 2.0 * fraction)
            self._set_speed(0.35 + 0.65 * blend)
            if fraction >= 1.0:
                self.phase = "tracking"

    def _decision(self, allowed: bool, *, reset_planner: bool = False) -> dict[str, Any]:
        return {
            "allow_retarget": allowed,
            "reset_planner": reset_planner,
            "generation": self.generation,
            **self.snapshot(),
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "phase": self.phase,
            "reason": self.reason,
            "generation": self.generation,
            "valid_streak": self.valid_streak,
            "confirmation_frames": self.confirmation_frames,
            "loss_count": self.loss_count,
            "recovery_count": self.recovery_count,
            "speed_scale": round(self.speed_scale, 5),
            "recovery_duration_s": self.recovery_duration_s,
            "observation_timeout_s": self.observation_timeout_s,
        }


__all__ = ["TrackingTransition"]
