from drivebase_behaviour.detection import Detection
from drivebase_behaviour.state_machine import (
    Config,
    Inputs,
    State,
    StateMachine,
)

CONFIG = Config(
    deadband=0.12,
    confirm_frames=5,
    confirm_frames_fallback=10,
    grasp_min=0.15,
    grasp_max=0.35,
    lost_timeout=2.0,
    approach_timeout=30.0,
    confirm_timeout=8.0,
    detection_stale_after=0.5,
    min_confidence=0.4,
)


def seen(now, error=0.0, confidence=0.9, fallback=False):
    return Detection(
        stamp_seconds=now, detected=True, frame_number=1,
        confidence=confidence, horizontal_error=error,
        range_is_fallback=fallback,
    )


def unseen(now):
    return Detection(stamp_seconds=now, detected=False, frame_number=1)


def blank(now, **kw):
    base = dict(
        now=now, detection=None, range_m=float("nan"), range_valid=False,
        in_workspace=False, nav_goal_active=True, nav_goal_succeeded=False,
        nav_goal_aborted=False, arm_sequence_done=False,
    )
    base.update(kw)
    return Inputs(**base)


def test_starts_idle_and_leaves_on_start():
    m = StateMachine(CONFIG)
    assert m.state is State.IDLE
    m.start()
    out = m.tick(blank(0.0, nav_goal_active=False))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_detection_interrupts_navigation_and_cancels_the_goal():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, detection=seen(1.0)))
    assert out.state is State.APPROACHING
    assert out.cancel_nav_goal is True


def test_low_confidence_detection_does_not_interrupt():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, detection=seen(1.0, confidence=0.2)))
    assert out.state is State.NAVIGATING


def test_stale_detection_does_not_interrupt():
    # std_msgs/String has no header, so a stamp can lag badly under load.
    # Acting on one is how the robot chases a target that is no longer there.
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(5.0, detection=seen(1.0)))
    assert out.state is State.NAVIGATING


def test_reaching_a_waypoint_requests_the_next_one():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, nav_goal_succeeded=True, nav_goal_active=False))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_aborted_goal_moves_on_rather_than_wedging():
    m = StateMachine(CONFIG)
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    out = m.tick(blank(1.0, nav_goal_aborted=True, nav_goal_active=False))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def approach(m, t=1.0):
    m.start()
    m.tick(blank(0.0, nav_goal_active=False))
    m.tick(blank(t, detection=seen(t)))
    return m


def test_entering_the_workspace_moves_to_confirming():
    m = approach(StateMachine(CONFIG))
    out = m.tick(blank(2.0, detection=seen(2.0), in_workspace=True))
    assert out.state is State.CONFIRMING


def test_losing_the_target_during_approach_resumes_the_patrol():
    m = approach(StateMachine(CONFIG))
    out = m.tick(blank(2.0, detection=unseen(2.0)))
    assert out.state is State.APPROACHING       # inside lost_timeout
    out = m.tick(blank(5.0, detection=unseen(5.0)))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_approach_timeout_resumes_the_patrol():
    m = approach(StateMachine(CONFIG))
    out = m.tick(blank(100.0, detection=seen(100.0)))
    assert out.state is State.NAVIGATING


def confirming(m):
    approach(m)
    m.tick(blank(2.0, detection=seen(2.0), in_workspace=True))
    return m


def test_confirmation_needs_consecutive_good_frames():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames - 1):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t), range_m=0.25,
                           range_valid=True, in_workspace=True))
        assert out.state is State.CONFIRMING
    t += 0.1
    out = m.tick(blank(t, detection=seen(t), range_m=0.25,
                       range_valid=True, in_workspace=True))
    assert out.state is State.PICKING
    assert out.store_grasp_point is True
    assert out.start_grasp is True


def test_an_off_centre_frame_resets_the_confirmation_count():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames - 1):
        t += 0.1
        m.tick(blank(t, detection=seen(t), range_m=0.25,
                     range_valid=True, in_workspace=True))
    t += 0.1
    out = m.tick(blank(t, detection=seen(t, error=0.4), range_m=0.25,
                       range_valid=True, in_workspace=True))
    assert out.state is State.CONFIRMING
    t += 0.1
    out = m.tick(blank(t, detection=seen(t), range_m=0.25,
                       range_valid=True, in_workspace=True))
    assert out.state is State.CONFIRMING


def test_an_invalid_range_never_confirms():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames + 3):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t), range_valid=False,
                           in_workspace=True))
        assert out.state is State.CONFIRMING


def test_a_range_outside_the_grasp_window_never_confirms():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames + 3):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t), range_m=0.80,
                           range_valid=True, in_workspace=True))
        assert out.state is State.CONFIRMING


def test_a_fallback_range_demands_more_frames():
    # Bbox-size distance is crude, so committing the arm on it needs more
    # evidence than a real ToF read does.
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t, fallback=True), range_m=0.25,
                           range_valid=True, in_workspace=True))
        assert out.state is State.CONFIRMING
    for _ in range(CONFIG.confirm_frames_fallback - CONFIG.confirm_frames):
        t += 0.1
        out = m.tick(blank(t, detection=seen(t, fallback=True), range_m=0.25,
                           range_valid=True, in_workspace=True))
    assert out.state is State.PICKING


def test_confirm_timeout_falls_back_to_approaching_once_then_gives_up():
    m = confirming(StateMachine(CONFIG))
    out = m.tick(blank(20.0, detection=seen(20.0), in_workspace=True))
    assert out.state is State.APPROACHING
    out = m.tick(blank(21.0, detection=seen(21.0), in_workspace=True))
    assert out.state is State.CONFIRMING
    out = m.tick(blank(40.0, detection=seen(40.0), in_workspace=True))
    assert out.state is State.NAVIGATING


def test_grasp_completion_stows_then_resumes_the_patrol():
    m = confirming(StateMachine(CONFIG))
    t = 2.0
    for _ in range(CONFIG.confirm_frames):
        t += 0.1
        m.tick(blank(t, detection=seen(t), range_m=0.25,
                     range_valid=True, in_workspace=True))
    assert m.state is State.PICKING
    out = m.tick(blank(t + 1.0, arm_sequence_done=True))
    assert out.state is State.STOWING
    assert out.start_stow is True
    out = m.tick(blank(t + 2.0, arm_sequence_done=True))
    assert out.state is State.NAVIGATING
    assert out.send_next_waypoint is True


def test_a_failed_pickup_never_wedges_the_mission():
    """Every terminal path must return to NAVIGATING. This is the invariant
    that keeps one bad piece of litter from ending the run."""
    for build in (
        lambda: (approach(StateMachine(CONFIG)),
                 blank(100.0, detection=unseen(100.0))),
        lambda: (confirming(StateMachine(CONFIG)),
                 blank(100.0, detection=seen(100.0), in_workspace=True)),
    ):
        m, late = build()
        for _ in range(5):
            out = m.tick(late)
        assert out.state in (State.NAVIGATING, State.APPROACHING,
                             State.CONFIRMING)
        for _ in range(20):
            out = m.tick(Inputs(**{**late.__dict__, "now": late.now + 200.0}))
        assert out.state is State.NAVIGATING
