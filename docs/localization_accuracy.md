# Localization accuracy, measured in simulation

All figures from `drivebase_sim/scripted_drive.py` against Gazebo ground truth
(`OdometryPublisher`, the model's true world pose). Analysis by
`scripts/analyse_drift.py`. Nine runs, three paths, three repeats each.

**Read the limitations section before quoting any of this.** One of these numbers
transfers to hardware and the rest are lower bounds at best.

---

## Headline: turns cost position, distance is nearly free

| Path | Distance | 90° turns | Local EKF error |
|---|---|---|---|
| `square2x8` | 3.0 m | 1 | 0.347 m |
| `square5` | 5.5 m | 1 | 0.350 m |
| `straight20` | 20.4 m | 1 | 0.347 m |

Path length varies almost 7× and the error does not move. Every run performed
exactly one successful 90° rotation, so:

> **A 90° turn costs roughly 0.35 m of position error. Twenty metres of straight
> driving costs essentially nothing.**

That resolves the contradiction that prompted this work — straight-line runs
measuring ~0.5% error while a Nav2 run with two turns measured ~9% per metre. It
was never about distance.

**Not verified: linearity.** Every run got one turn, because of the rotation bug
below, so whether two turns cost 0.70 m is unmeasured. Treat 0.35 m as the cost
of *a* turn, not as a rate to extrapolate.

### Why this matters beyond the number

- Nav2's controller is configured to prefer arcs over point turns
  (`rotate_to_heading_min_angle: 0.90` in `drivebase_navigation`). That was
  justified on current draw and CPU; this makes it a *localization* argument too.
- The base cannot be trusted to position the arm. A pickup approach involving a
  couple of turns lands ~0.7 m out, and litter is much smaller than that. Final
  approach has to close the loop on the camera.

---

## Effective-track factor

Ratio of yaw change reported by raw wheel odometry to true yaw change, over
6 clean turns:

| | |
|---|---|
| median | **1.742** |
| mean | 1.749 |
| range | 1.717 – 1.779 |
| stdev | 0.025 |

Tight, and stable across lateral-friction values from 0.15 to 0.6 — as expected,
since it is kinematic rather than frictional. `DiffDrive` derives odometry from
ideal two-wheel kinematics while the real robot loses rotation to scrub.

Earlier ad-hoc measurements of 1.53× and 1.94× bracket this but were single
samples from an unreliable harness. Use 1.74.

To make wheel odometry self-consistent you would set an effective
`wheel_separation` of **0.749 m** against a physical track of 0.430 m. It is
deliberately *not* set that way in `gazebo.xacro`: the sim should reproduce the
same bias the real robot has, which is exactly what forces the EKF to take yaw
from the IMU rather than the wheels.

---

## GPS-corrected global filter

| | Local (dead reckoning) | Global (GPS) |
|---|---|---|
| across all nine runs | 0.347 – 0.351 m | 1.44 – 5.59 m |
| median | 0.35 m | ~3.0 m |

Global error is **bounded and trendless**, which is the property GPS provides.
But it is an order of magnitude worse than dead reckoning at these scales, and
enabling `use_gps:=true` will look like a regression. See the limitations —
this comparison does not transfer.

---

## Limitations

**1. The simulated IMU is unrealistically good.** It reports near-perfect absolute
heading: no bias drift, no magnetometer disturbance, no temperature dependence.
Real MEMS parts have all three, and heading error is precisely what becomes
position drift. So the local filter's 0.35 m is a **lower bound**, and the
local-versus-global comparison above cannot settle whether GPS is worth carrying.
That decision has to be made on hardware.

Corollary worth stating plainly: **do not use these numbers to justify skipping
the 9-DOF IMU.** They assume something better than a BNO055.

**2. The simulated robot cannot rotate in place more than once per run.** The
first rotation of any run succeeds; every subsequent one leaves the body
stationary while the wheel joints spin. Measured directly — during a failed turn
`odom_yaw` advanced 279° while `true_yaw` moved 0.0°.

Ruled out by experiment:

| Suspect | Test | Result |
|---|---|---|
| Commanded angular velocity too low | 0.5, 0.8, 1.2 rad/s | all fail identically |
| The arm's mass / joint controllers | `arm:=false` | fails, and worse |
| Lateral wheel friction | `wheel_mu2` 0.6, 0.3, 0.15 | all fail identically |
| Driving between turns | `twoturns` path, no translation | still fails |

Remaining hypothesis: `DiffDrive` commands joint *velocity* with no torque limit,
so wheels spin regardless of load and the contact simply slips. Coulomb friction
then yields a force independent of commanded speed, which is exactly why raising
the angular velocity changed nothing. Worth trying
`gz-sim-wheel-slip-system`, or moving to `ros2_control` with effort-limited
joints, before trusting any multi-turn result.

**Consequence:** no simulation-derived claim about repeated turning is
trustworthy, including the earlier "~65% of commanded rotation is achieved". The
Nav2 obstacle-avoidance results are unaffected — Regulated Pure Pursuit arcs
rather than point-turns, and those runs reached their goals.

---

## Reproducing

```bash
ros2 launch drivebase_sim sim.launch.py headless:=true gps:=true
ros2 run drivebase_sim scripted_drive --ros-args -p use_sim_time:=true \
    -p path_name:=straight20 -p output_csv:=/tmp/run.csv
python3 scripts/analyse_drift.py /tmp/run.csv
```

Paths: `straight20`, `square5`, `square2x8`, `smoke`, `twoturns`.

The node aborts loudly if commanded motion produces no movement for
`stall_timeout` seconds. That watchdog is the reason the rotation bug was found
at all — the previous shell harness recorded those same runs as clean squares.
