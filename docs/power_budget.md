# Power budget and electrical architecture

Sizing for the litter-collection drivebase. Figures are estimates from datasheet
and typical-part values, not measurements — the robot is not assembled yet. Every
number here should be replaced with a measured value once the INA226 monitor is
in place, and this document updated.

Nominal voltages, after the 2026-07-31 pack decision (§2): motion pack
**18–20 V** (power tool battery), compute pack **5 V** (USB-C PD). Wattages
below are unchanged by that decision — only the currents they imply are.

---

## 1. Load breakdown

Two isolated packs, so the budget is split by which pack carries the load.

### Compute pack

| Load | Idle | Active |
|---|---|---|
| Raspberry Pi 5 (YOLO + Nav2) | 3 W | 10 W |
| Hailo-8 AI HAT *(if fitted)* | 1 W | 2.5 W |
| Pico + NEO-6M GPS + IMU | 0.3 W | 0.3 W |
| USB camera | 1 W | 1 W |
| Ultrasonics ×5 (HC-SR04, 15 mA each) | 0.4 W | 0.4 W |
| **Total** | **~5.7 W** | **~14.2 W** |

### Motion pack

| Load | Idle | Cruise (pavement) | Cruise (grass/incline) | Pickup |
|---|---|---|---|---|
| Drive motors ×4 | 0 | ~72 W | ~145 W | 0 |
| SO101 servos ×6 | 11 W *(holding)* | 11 W *(holding)* | 11 W | ~40 W |
| **Total** | **~11 W** | **~83 W** | **~156 W** | **~40 W** |

### Combined, by scenario

| Scenario | Total |
|---|---|
| Parked, powered, arm holding | ~17 W |
| Cruising, pavement | ~97 W |
| Cruising, grass or incline | ~170 W |
| Stopped, arm executing pickup | ~54 W |

**Drive motors are 75–85% of total draw.** The entire Pi-vs-Hailo question is a
~3 W swing — under 3% of budget. Fit the accelerator for CPU headroom, which is
a real constraint, not for battery life. Efficiency effort belongs in gearing,
tire rolling resistance, and driving policy.

### Currents

Derived from the wattages above, at 18 V — the low end of the tool pack's
range, so these are the conservative (highest-current) figures.

| Scenario | Power | Current |
|---|---|---|
| Parked, arm holding | ~11 W | 0.6 A |
| Cruising, pavement | ~83 W | 4.6 A |
| Cruising, grass or incline | ~156 W | 8.7 A |
| Stopped, arm executing pickup | ~40 W | 2.2 A |
| Mission average, pavement | ~62 W | 3.4 A |
| Mission average, grass | ~105 W | 5.8 A |
| **Four wheels stalled** | — | **~8 A** |

Compute pack at 5 V: **1.1 A** idle, **2.8 A** active. Note that is well under
5 A — the 5 V/5 A supply requirement is about the Pi 5 only lifting its 600 mA
peripheral cap when it negotiates 5 A, not about total draw.

**In-place rotation on grass is the peak-current maneuver** (§5), approaching
stall. Size fuses and wire against the ~8 A stall figure, not the 8.7 A cruise
one — they happen to be close here, which is itself a consequence of moving to
24 V motors.

**The 40 A figure that drove the old BMS requirement was never the load.** It
is what two MDD10A boards could demand at their rating. Four motors physically
cannot pull it. That is why a tool pack, built for 40–60 A tool loads, clears
this requirement without being shopped for.

---

## 2. Duty-cycle averages and pack sizing

Assumed mission profile: **60% driving / 25% stopped-and-picking / 15% idle.**

| Surface | Motion pack average | 1 h energy |
|---|---|---|
| Pavement | ~62 W | ~62 Wh |
| Grass / incline | ~105 W | ~105 Wh |

### Motion pack — power tool battery, 18–20 V / 5 Ah

**Decided 2026-07-31.** DeWalt 20 V or Makita 18 V with a screw-terminal
adapter plate. ~90 Wh nominal, ~72 Wh usable at 80% depth of discharge.
Runtime: **~1.2 h on pavement, ~0.7 h on grass.** Buy two — they hot-swap in
seconds, which covers a longer session better than one larger pack.

Chosen over 12 V LiFePO₄ on three grounds, in order of weight:

- **Wiring.** A latching pack on a screw-terminal plate is the easiest
  mechanical interface available: no connector to crimp, no charger-specific
  lead, and a swap between runs takes seconds.
- **The BMS requirement comes free.** Tool packs are built for 40–60 A
  continuous tool loads, so the rating that was driving the LiFePO₄ search is
  satisfied without shopping for it. Cheap 12 Ah LiFePO₄ packs ship 20–30 A
  BMSs that trip on a stall and cut power mid-run.
- **Cost**, if the ecosystem is already in the building.

**This is why the motors are specified at 24 V** (§6). A 24 V motor on an
18–20 V pack runs at ~80% of rated speed, safely, with no buck converter on the
motion rail. Buying 12 V motors would have required a 15–20 A buck: new cost,
new heat, and a new single point of failure between the pack and the wheels.

The cost of the decision is runtime — ~72 Wh usable against ~123 Wh from the
12 Ah LiFePO₄ — and a slightly thinner market for 24 V 37D-class gearmotors.

3S LiPo was rejected for sagging under load; sealed lead acid for ~50% usable
capacity and ~4 kg.

### Compute pack — USB-C PD power bank, 5 V / 5 A capable

At ~14 W active, even a modest 10000 mAh bank (~37 Wh nominal) gives 2+ hours; a
20000 mAh gives 4+. Hot-swappable between runs without rebooting the Pi, and
charges independently of the motion pack.

> **Must negotiate 5 V / 5 A (25 W+).** A 5 V/3 A supply makes the Pi 5 cap
> total peripheral current at 600 mA, which can brown out the USB camera. The
> symptom looks like a camera driver bug, not a power problem, and will cost you
> an afternoon.

---

## 3. Why two packs

The most common failure mode in a robot this size: motors and compute share a
rail, a stall transient sags the voltage, the Pi browns out and reboots — losing
the run *and* its data. Two packs remove the failure mode entirely rather than
mitigating it.

**The isolation is signal-level, not galvanic.** The Pico's USB link to the Pi
ties their grounds, and the MDD10A logic inputs are not opto-isolated, so motor
ground is referenced too. This is normal and works, but:

- Run a **heavy, short ground** between Pico and motor driver.
- **Never route logic wiring alongside motor leads** — bundle them separately.
- Fuse each rail independently at the battery.

---

## 4. Safety

**E-stop cuts the motion pack only.** Compute stays up so the Pi keeps logging
through the event, which is exactly the data worth having. Wire it as a
normally-closed switch in the motion pack's positive lead, rated for stall
current (~8 A — see §1, so a 15 A-rated switch is ample), plus a firmware-level
disable on the Pico.

**The Pico firmware must implement a command timeout (deadman).** Measured in
simulation: Gazebo's DiffDrive latches the last velocity command and keeps
driving indefinitely when commands stop arriving. Real hardware must not behave
that way — if `/cmd_vel` goes silent for >0.5 s (ROS crash, USB unplug, Wi-Fi
drop), the wheels must stop on their own. This is a firmware requirement, not a
nice-to-have.

---

## 5. Consequences that reach into other configuration

**In-place rotation is the peak-current maneuver.** Skid steering scrubs all four
wheels laterally; on grass this approaches stall. The Nav2 controller should be
tuned to prefer arcs over point turns, which is a power decision expressed as a
navigation parameter. Belongs in the Nav2 config when that lands.

**The arm draws ~11 W holding a stowed pose**, because the servos actively resist
gravity. A mechanical rest that unloads them during transit removes a continuous
drain that is easy to overlook.

**Thermal.** A Pi 5 at 10 W sustained inside a sealed printed cube, outdoors in
sun, will thermal throttle. That directly corrupts measured YOLO inference rate,
which is a graded deliverable. Needs a vent path and active cooling.

---

## 6. Drive motor specification

Sizing math, so the choice is auditable:

- **Speed.** 120 mm wheel → 0.377 m circumference. Target 0.6–0.8 m/s →
  **100–130 RPM at the wheel.** A 24 V motor on an 18–20 V pack turns at ~80%
  of its rating, so buy **120–155 RPM rated at 24 V** to land in that window.
- **Torque.** ~12 kg loaded. The binding case is skid-steer turning, not driving
  straight: scrub force ≈ µ·W ≈ 0.7 × 118 N ≈ **83 N**, needing ~5 N·m total at
  the wheels → **≥1.2 N·m stall per wheel.** Straight-line grass on a 15% grade
  needs roughly half that, which is why rotation dominates.

**Specification:** 37 mm-diameter **24 V** metal gearmotor, integrated
quadrature hall encoder, **120–155 RPM rated at 24 V**, **≥1.2 N·m stall**,
~2.5 A stall at 24 V.

Buy by output RPM, not by reduction ratio — a ratio only means something
against a known base-motor speed. If a vendor's 1:100 part is rated 60 RPM at
24 V, its base motor is slower and its torque and current figures will not
match this sizing either.

Generic parts run $15–25 each. Pololu 37D equivalents are ~$45 each but ship
published torque and current curves — worth paying for when the writeup needs
justified numbers rather than assumed ones. The 24 V market is thinner than the
12 V one, which is the main practical cost of the pack decision.

### Why encoder counting goes on the Pico

At 11 PPR on the motor shaft and 1:90 reduction, quadrature gives
**11 × 4 × 90 ≈ 3960 counts per wheel revolution**. At 0.6 m/s that is ~6300
counts/s per wheel, **~25,000 edges/s across four wheels**.

Linux userspace GPIO will drop counts at that rate, and dropped counts do not
raise an error — they silently under-report distance travelled, which corrupts
odometry in a way that is genuinely hard to diagnose. The Pico counts these in
PIO hardware and reports position over USB serial.

---

## 7. Open items

- [ ] Replace every estimate here with an INA226 measurement once assembled
- [ ] Confirm actual motor stall/rated current at 24 V against the part finally
      ordered
- [x] ~~Confirm the LiFePO₄ pack's BMS continuous rating ≥40 A~~ — **moot.**
      Pack decision moved to a power tool battery, whose BMS is built for
      40–60 A tool loads. See §2
- [ ] Confirm the ~80% speed derate empirically once the motors arrive — the
      figure assumes a linear speed/voltage relationship, which is close enough
      for sizing but not for odometry
- [ ] Measure SO101 holding current — the 11 W figure is inferred from STS3215
      idle current, not measured
- [ ] Revise the ~12 kg loaded estimate: the SO-101 turns out to be **0.632 kg**
      across 8 links (from its URDF), not the ~1.2 kg assumed when sizing motors.
      Torque figures are therefore conservative, not optimistic.
- [x] ~~Arm cannot reach the ground~~ — **resolved.** The SO-101 reaches 219 mm
      below its own mount. Moved from the top plane (460 mm, gripper bottomed out
      at 241 mm) to the front face at 260 mm, which puts the gripper at **41 mm
      above ground** — within grasping range of litter. Both figures measured in
      simulation, not calculated.
- [ ] Publish `sensor_msgs/BatteryState` from the motion pack so Nav2 can see
      remaining charge, and so run logs carry voltage alongside results
