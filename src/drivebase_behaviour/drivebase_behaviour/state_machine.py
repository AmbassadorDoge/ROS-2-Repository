"""The mission state machine.

PURE. No rclpy, no clock, no topics - `now` arrives as a float and every
decision is a function of the Inputs it is handed. That is what lets the whole
mission be tested in milliseconds with no simulator, and it is why this file
must stay free of ROS imports.

THE INVARIANT: every state has a timeout whose fallback is resuming the patrol.
A failed pickup loses one piece of litter. It must never wedge the mission.
"""

from dataclasses import dataclass, field
from enum import Enum

from drivebase_behaviour.detection import Detection


class State(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    APPROACHING = "approaching"
    CONFIRMING = "confirming"
    PICKING = "picking"
    STOWING = "stowing"


@dataclass(frozen=True)
class Config:
    deadband: float
    confirm_frames: int
    confirm_frames_fallback: int
    grasp_min: float
    grasp_max: float
    lost_timeout: float
    approach_timeout: float
    confirm_timeout: float
    detection_stale_after: float
    min_confidence: float


@dataclass(frozen=True)
class Inputs:
    now: float
    detection: Detection | None
    range_m: float
    range_valid: bool
    in_workspace: bool
    nav_goal_active: bool
    nav_goal_succeeded: bool
    nav_goal_aborted: bool
    arm_sequence_done: bool


@dataclass(frozen=True)
class Outputs:
    state: State
    cancel_nav_goal: bool = False
    send_next_waypoint: bool = False
    store_grasp_point: bool = False
    start_grasp: bool = False
    start_stow: bool = False


@dataclass
class StateMachine:
    config: Config
    state: State = State.IDLE
    _entered_at: float = 0.0
    _last_seen_at: float = 0.0
    _confirm_count: int = 0
    _confirm_attempts: int = 0
    _started: bool = field(default=False, repr=False)

    def start(self) -> None:
        self._started = True

    def _enter(self, state: State, now: float) -> None:
        self.state = state
        self._entered_at = now
        if state is not State.CONFIRMING:
            self._confirm_count = 0
        if state is State.APPROACHING:
            self._last_seen_at = now
        if state is State.NAVIGATING:
            self._confirm_attempts = 0

    def _usable(self, inputs: Inputs) -> bool:
        """A detection worth acting on: present, confident, and recent.

        The recency test exists because std_msgs/String carries no stamp, so
        the adapter stamps on arrival and a backed-up queue looks exactly like
        a target that is still there.
        """
        d = inputs.detection
        if d is None or not d.detected:
            return False
        if d.confidence < self.config.min_confidence:
            return False
        age = inputs.now - d.stamp_seconds
        return age <= self.config.detection_stale_after

    def tick(self, inputs: Inputs) -> Outputs:
        if self.state is State.IDLE:
            if self._started:
                self._enter(State.NAVIGATING, inputs.now)
                return Outputs(state=self.state, send_next_waypoint=True)
            return Outputs(state=self.state)

        if self.state is State.NAVIGATING:
            return self._navigating(inputs)
        if self.state is State.APPROACHING:
            return self._approaching(inputs)
        if self.state is State.CONFIRMING:
            return self._confirming(inputs)
        if self.state is State.PICKING:
            return self._picking(inputs)
        return self._stowing(inputs)

    def _navigating(self, inputs: Inputs) -> Outputs:
        if self._usable(inputs):
            self._enter(State.APPROACHING, inputs.now)
            # Cancel first: controller_server must stop publishing before the
            # coordinator writes /cmd_vel_nav, or two publishers fight over it.
            return Outputs(state=self.state, cancel_nav_goal=True)

        if inputs.nav_goal_succeeded or inputs.nav_goal_aborted:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)

        if not inputs.nav_goal_active:
            return Outputs(state=self.state, send_next_waypoint=True)

        return Outputs(state=self.state)

    def _approaching(self, inputs: Inputs) -> Outputs:
        if inputs.now - self._entered_at > self.config.approach_timeout:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)

        if self._usable(inputs):
            self._last_seen_at = inputs.now
            if inputs.in_workspace:
                self._enter(State.CONFIRMING, inputs.now)
                return Outputs(state=self.state)
            return Outputs(state=self.state)

        if inputs.now - self._last_seen_at > self.config.lost_timeout:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)

        return Outputs(state=self.state)

    def _confirming(self, inputs: Inputs) -> Outputs:
        if inputs.now - self._entered_at > self.config.confirm_timeout:
            self._confirm_attempts += 1
            # One retry from a fresh approach; a second timeout means the
            # target is not graspable from here and the patrol matters more.
            if self._confirm_attempts >= 2:
                self._enter(State.NAVIGATING, inputs.now)
                return Outputs(state=self.state, send_next_waypoint=True)
            attempts = self._confirm_attempts
            self._enter(State.APPROACHING, inputs.now)
            self._confirm_attempts = attempts
            return Outputs(state=self.state)

        good = (
            self._usable(inputs)
            and abs(inputs.detection.horizontal_error) <= self.config.deadband
            and inputs.range_valid
            and self.config.grasp_min <= inputs.range_m <= self.config.grasp_max
        )

        if not good:
            self._confirm_count = 0
            return Outputs(state=self.state)

        self._confirm_count += 1
        needed = (
            self.config.confirm_frames_fallback
            if inputs.detection.range_is_fallback
            else self.config.confirm_frames
        )
        if self._confirm_count < needed:
            return Outputs(state=self.state)

        self._enter(State.PICKING, inputs.now)
        return Outputs(
            state=self.state, store_grasp_point=True, start_grasp=True)

    def _picking(self, inputs: Inputs) -> Outputs:
        if inputs.arm_sequence_done:
            self._enter(State.STOWING, inputs.now)
            return Outputs(state=self.state, start_stow=True)
        return Outputs(state=self.state)

    def _stowing(self, inputs: Inputs) -> Outputs:
        if inputs.arm_sequence_done:
            self._enter(State.NAVIGATING, inputs.now)
            return Outputs(state=self.state, send_next_waypoint=True)
        return Outputs(state=self.state)
