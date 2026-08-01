# Behaviour coordinator: logic review, 2026-08-01

Read this before changing `drivebase_behaviour`. It records what a logic review
found, what it deliberately did NOT check, and which suspicious-looking things
were checked and cleared so nobody spends an afternoon re-deriving them.

Every finding below is also marked in the source with a greppable
`REVIEW 2026-08-01:` comment at the line it concerns:

```bash
grep -rn "REVIEW 2026-08-01" src/
```

**Nothing here has been fixed.** The comments and this file are the whole
change. Findings are ordered by severity, not by file.

## Scope

| | |
|---|---|
| Reviewed | Mission logic only: `state_machine.py`, `coordinator_node.py`, `arm_driver.py`, `test_state_machine.py` |
| Not reviewed | Config values, `docs/`, `scripts/`, URDF/xacro, meshes, RViz, and the geometry internals of `ik.py` / `kinematics.py` / `localisation.py` / `detection.py` |
| Branch | `experimental/behaviour-coordinator` at `3a0552f` |
| Method | Read-only. No simulator was run, so every claim here is from source, not from observed behaviour. Anything phrased as a runtime consequence is reasoned, not measured. |

## Findings

### F1 — `PICKING` and `STOWING` have no timeout, and neither does the test guarding them

**Severity: latent, not live.** `state_machine.py:8` states the module
invariant: "every state has a timeout whose fallback is resuming the patrol...
It must never wedge the mission." `_picking` and `_stowing` leave on
`arm_sequence_done` alone, and `Config` has no timeout field for either.

What bounds them today lives in a different module. `ArmDriver.start_sequence([])`
sets `sequence_done = True` immediately, and `ArmDriver.update` advances purely
on deadlines, so any sequence finishes in about `5 x grasp_hold_seconds` (7.5 s
at the default). **The mission does not currently wedge.** The problem is that
`state_machine.py` claims a property only `arm_driver.py` enforces, with no test
across the seam.

The test makes it worse rather than better.
`test_state_machine.py:test_a_failed_pickup_never_wedges_the_mission` has the
docstring "Every terminal path must return to NAVIGATING" and exercises exactly
two paths: `approach(...)` and `confirming(...)`. Those are the two states that
already have timeouts. The two without one are unexercised, so the test's name
asserts a guarantee its body does not check.

Fix: `pick_timeout` / `stow_timeout` in `Config`, falling back to `NAVIGATING`,
plus two more cases in that test.

### F2 — The litter is never deposited, and `stow_pose` is dead

**Severity: mission gap. Needs a product decision, not just a patch.**

`start_stow` reaches `coordinator_node._tick`, which runs
`start_sequence([(search_pose, hold)])`. But `grasp_sequence` (`arm_driver.py`)
already ends at `(search_pose, gripper_closed)`. So `STOWING` re-commands the
pose the arm is already holding, waits `grasp_hold_seconds`, and resumes. It is
a timed no-op.

Nothing ever reopens the gripper. The first thing that releases a piece of
litter is the *next* grasp's opening step, which means piece N is dropped in
mid-air wherever the base is standing when it begins piece N+1.

`stow_pose` is declared in two places (`coordinator_node.py` defaults and
`config/coordinator.yaml`) and read nowhere.

The plan (`docs/superpowers/plans/2026-07-30-behaviour-coordinator.md`) specifies
the `STOWING` state and the `start_stow` output but no deposit either, so this
is a hole in the mission rather than a drift from the plan. An onboard bin, a
release step, and a "carrying" state are all unbuilt.

**Open question for the maintainer: where is the litter supposed to go?** Until
that is answered the right fix is unknowable. The dead `stow_pose` suggests
someone intended an answer here.

### F3 — `_send_next_waypoint` spins at `tick_hz` once a finite patrol ends

`waypoint_index` is incremented before the bounds test. With `loop_patrol: false`
and the list exhausted, the function returns early, but the state machine emits
`send_next_waypoint=True` on *every* tick while no goal is active. So
`waypoint_index` grows without bound and "Patrol complete" logs at 10 Hz forever.

`loop_patrol` defaults to `true`, so this is off the default path, but it is
live on a supported setting. Needs a terminal state, or a latched `patrol_done`
the machine can read.

### F4 — A blocking 2 s wait sits inside the timer callback

`_send_next_waypoint` calls `nav_client.wait_for_server(timeout_sec=2.0)`, and
is itself called from `_tick`. Every callback on this node uses the node's
default mutually exclusive callback group, so a block there stalls the entire
coordinator: `arm.update()` stops advancing the grasp sequence, detections back
up, `/cmd_vel_nav` goes quiet.

Combined with F3, a Nav2 outage means blocking 2 s per tick indefinitely.
`server_is_ready()` is the non-blocking test; the wait belongs at startup.

### F5 — `Outputs.start_grasp` is set and never consumed

Defined and set in `state_machine.py`, asserted in `test_state_machine.py`, read
nowhere in `coordinator_node.py`. The coordinator triggers the grasp off
`store_grasp_point` instead. Two flags, one signal, and the only thing keeping
`start_grasp` alive is a test. Drop it, or make the coordinator use it.

Note the two are safely coupled *today* by an implicit invariant worth knowing
about: `store_grasp_point` can only be true on a tick where `grasp_reachable`
was true, which requires `target is not None`. That is why the
`if outputs.store_grasp_point and target is not None` guard never silently skips
a grasp the machine already committed to. It holds only because both are
computed in the same tick.

### F6 — The `_confirm_attempts` save/restore is a no-op

`state_machine._confirming` saves `_confirm_attempts`, calls
`_enter(State.APPROACHING, ...)`, then restores it. `_enter` clears that counter
only on entry to `NAVIGATING`, never `APPROACHING`, so there is nothing to
protect against. Harmless, but it asserts a behaviour `_enter` does not have.

### F7 — `MultiThreadedExecutor` provides no parallelism, and that is load-bearing

Every callback uses the node's default mutually exclusive group, so the executor
serialises them regardless. That serialisation is what makes the unguarded
cross-callback state safe without a lock: `nav_goal_handle`, `nav_succeeded` and
`nav_aborted` are written from action callbacks and read in `_tick`.

This is correctness-critical and documented nowhere, which stands out in a file
where every other non-obvious choice carries a paragraph. **Do not swap in a
reentrant callback group or add a second spinning thread without adding
locking first.**

## Checked and cleared

Do not re-investigate these.

| Suspicion | Verdict |
|---|---|
| Pure-logic modules import `rclpy`, violating CLAUDE.md | **False alarm.** A `grep -l rclpy` lights up all eight modules; every hit in `state_machine`/`detection`/`localisation`/`kinematics`/`ik` is a line-3 docstring *declaring* the constraint. The boundary holds. |
| `__pycache__` and `.pytest_cache` committed to git | **False alarm.** Present in the working tree, zero tracked. `.gitignore` covers both. |
| The `_store_and_grasp` failure path wedges `PICKING` | **No.** `start_sequence([])` sets `sequence_done = True`, so the abandon path advances to `STOWING` on the next tick. |
| `store_grasp_point` could fire with `target is None`, skipping the grasp | **No.** See the coupling note in F5. |
| `inputs.detection.horizontal_error` could dereference `None` | **No.** `_usable(inputs)` is first in the `and` chain and returns `False` for `None`; Python short-circuits. |

## Repo gotcha found while reviewing

**`origin/master` is a single "Initial commit!".** The whole project lives on
`experimental/behaviour-coordinator`, 51 commits ahead of it, and there is no
local `master` branch at all.

Two consequences for agents:

1. `git diff master...HEAD` is not a review unit. It is the entire repo: 215
   files, 22,335 insertions, meshes and vendored URDF included.
2. **A git worktree created with the default `fresh` base ref branches from
   `origin/master` and comes up empty.** This review's worktree had to be made
   explicitly from `HEAD`:
   ```bash
   git worktree add .claude/worktrees/<name> -b <branch> HEAD
   ```

## What a follow-up should do

F1, F3, F5 and F6 are self-contained: they touch pure logic plus its tests and
need no simulator to verify. F4 is a small independent change. F2 is blocked on
the deposit question above and should not be "fixed" by guessing.
