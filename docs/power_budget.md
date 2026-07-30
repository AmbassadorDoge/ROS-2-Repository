# Power budget and electrical architecture

Sizing for the litter-collection drivebase. Figures are estimates from datasheet
and typical-part values, not measurements — the robot is not assembled yet. Every
number here should be replaced with a measured value once the INA226 monitor is
in place, and this document updated.

Nominal system voltage: **12 V**.

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

---

## 2. Duty-cycle averages and pack sizing

Assumed mission profile: **60% driving / 25% stopped-and-picking / 15% idle.**

| Surface | Motion pack average | 1 h energy |
|---|---|---|
| Pavement | ~62 W | ~62 Wh |
| Grass / incline | ~105 W | ~105 Wh |

### Motion pack — LiFePO₄ 12.8 V / 12 Ah

154 Wh nominal, ~123 Wh usable at 80% depth of discharge, ~1.6 kg, $80–110.
Runtime: **~2 h on pavement, ~1.2 h on grass.** Comfortable margin for a
one-hour field session with re-runs.

Chosen over the alternatives because the flat discharge curve sits right at the
motors' nominal voltage for most of the pack's range, it survives 2000+ cycles,
and it will not vent in a competition pit. 3S LiPo is lighter but sags under
load. Sealed lead acid is disqualified: only ~50% usable capacity and ~4 kg.

> **Check the BMS continuous-current rating before buying, not the Ah.** Two
> Cytron MDD10A boards can pull 40 A peak. Many cheap 12 Ah LiFePO₄ packs ship
> with a BMS limited to 20–30 A, which will trip on a stall and cut power
> mid-run. Look for ≥40 A continuous, or accept a current limit in firmware.

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
current, plus a firmware-level disable on the Pico.

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
  **100–130 RPM at the wheel.**
- **Torque.** ~12 kg loaded. The binding case is skid-steer turning, not driving
  straight: scrub force ≈ µ·W ≈ 0.7 × 118 N ≈ **83 N**, needing ~5 N·m total at
  the wheels → **≥1.2 N·m stall per wheel.** Straight-line grass on a 15% grade
  needs roughly half that, which is why rotation dominates.

**Specification:** 37 mm-diameter 12 V metal gearmotor, integrated quadrature
hall encoder, **1:70–1:100** reduction, **≥1.2 N·m stall**, ~5 A stall.

Generic parts run $15–25 each. Pololu 37D equivalents are ~$45 each but ship
published torque and current curves — worth paying for when the writeup needs
justified numbers rather than assumed ones.

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
- [ ] Confirm actual motor stall/rated current against the part finally ordered
- [ ] Confirm the LiFePO₄ pack's BMS continuous rating ≥40 A
- [ ] Measure SO101 holding current — the 11 W figure is inferred from STS3215
      idle current, not measured
- [ ] Revise the ~12 kg loaded estimate: the SO-101 turns out to be **0.632 kg**
      across 8 links (from its URDF), not the ~1.2 kg assumed when sizing motors.
      Torque figures are therefore conservative, not optimistic.
- [ ] **Blocking mechanical issue:** with the arm on the chassis top plane
      (460 mm above ground) the gripper bottoms out at **389 mm above ground** —
      measured in simulation across the joint range. It cannot reach litter. The
      arm needs a lower mount before any pickup work is meaningful.
- [ ] Publish `sensor_msgs/BatteryState` from the motion pack so Nav2 can see
      remaining charge, and so run logs carry voltage alongside results
