# Bill of materials

Everything the design implies. Ordering is the critical path — hardware weeks all
wait on lead times, so this is item 1 in `STATUS.md`.

**Status column is unfilled.** Nothing in the repo records what is already on
hand; mark each row as you confirm it.

Every spec here traces back to `power_budget.md` or a measured figure in
`localization_accuracy.md`. Where a spec has a *reason*, the reason is given —
several of these parts have a cheap variant that will silently fail in a way
that costs a week.

---

## 1. The pack decision, and what it changed

**Decided 2026-07-31: power tool battery pack, not 12 V LiFePO₄.** DeWalt 20 V
or Makita 18 V, with a screw-terminal adapter plate. Three consequences, and
they reach into the rest of this list:

1. **Motors are 24 V, not 12 V.** A 24 V motor on an ~18–20 V pack runs at
   ~80% of rated speed, which is safe and needs no buck converter on the motion
   rail. Buying 12 V motors instead would mean a 15–20 A buck — new cost, new
   heat, and a new single point of failure. Spec RPM accordingly; see §2.
2. **The BMS problem disappears.** A tool pack's BMS is built for circular saws
   pulling 40–60 A continuously, so the ≥40 A requirement that was driving the
   LiFePO₄ search is satisfied without shopping for it.
3. **Current roughly halves**, which is the real "easier to wire" win: 14 AWG
   instead of 12, a 15 A fuse instead of 30 A, less voltage drop, and a stall
   transient near 8 A rather than 21 A.

The cost is runtime: ~72 Wh usable per 5 Ah pack against ~123 Wh from the
12 Ah LiFePO₄. Packs hot-swap in seconds, so buy two rather than one big one.

### Still easy to get wrong

- **Power bank must negotiate 5 V / 5 A (25 W+).** At 5 V/3 A the Pi 5 caps
  total peripheral current at 600 mA and can brown out the USB camera. The
  symptom presents as a camera driver bug. The cable must be 5 A-rated too — it
  can be the thing that blocks negotiation.
- **IMU must be 9-DOF (BNO055 or ICM-20948), never an MPU6050.** A 6-DOF part
  cannot provide absolute heading. Yaw comes from the IMU and never from the
  wheels (skid steer over-reports by a measured 1.742×), so heading *is* the
  error budget.
- **Buy gearmotors by output RPM, not by reduction ratio.** See §2.

---

## 2. Motion pack

| Part | Qty | Spec that matters | Status |
|---|---|---|---|
| Tool battery pack | 2 | DeWalt 20 V or Makita 18 V, ≥5 Ah. Two so they hot-swap between runs | |
| Battery adapter plate | 2 | Screw-terminal plate for the chosen brand — this is the whole wiring win | |
| Gearmotor + quadrature encoder | 4 | 37 mm dia, **24 V**, **120–155 RPM rated at 24 V**, **≥1.2 N·m stall**, integrated hall encoder | |
| Cytron MDD10A motor driver | 2 | Accepts 5–30 V, so unaffected by the voltage change | |
| Wheels | 4 | **120 mm** diameter | |
| E-stop switch | 1 | Normally-closed, in the motion pack positive lead, rated ≥15 A | |
| Blade fuse block | 1 | 6-way ATC. Gives a common positive bus **and** per-rail fusing in one part | |
| Blade fuses | — | 15 A motion rail; size the compute rail to its own draw | |
| Anderson Powerpole set | — | Genderless and hot-swappable — much better than XT60 when pulling packs between runs | |
| Negative bus bar | 1 | Gives the heavy short Pico↔driver ground an obvious landing point | |
| Silicone wire | — | **14 AWG** motion rail | |

### Motor sizing, so the choice is auditable

- **Speed.** 120 mm wheel → 0.377 m circumference. Target 0.6–0.8 m/s →
  1.6–2.1 rev/s → **96–127 RPM at the wheel.** A 24 V motor on an 18–20 V pack
  turns at ~80% of its rating, so buy **120–155 RPM rated at 24 V** to land in
  that window.
- **Torque.** ~12 kg loaded. The binding case is skid-steer turning, not
  driving straight: scrub force ≈ µ·W ≈ 0.7 × 118 N ≈ 83 N, needing ~5 N·m
  total at the wheels → **≥1.2 N·m stall per wheel.** Straight-line grass on a
  15% grade needs roughly half that.

**Rated RPM is no-load**, and a loaded 12 kg robot on grass pulls it down a
further 10–20% on top of the voltage derate. Bias toward the top of the range.

**Do not spec reduction and RPM independently.** A ratio only means something
against a known base-motor speed. If a vendor's 1:100 part is rated 60 RPM at
24 V, its base motor is slower and its torque and current figures will not
match this sizing either.

**Encoder counts must be re-derived from the part actually bought.** The
~25,000 edges/s figure that justifies PIO counting assumes 11 PPR × 4 × 1:90 ≈
3960 counts per wheel revolution. A different encoder changes this, and it
feeds odometry directly.

---

## 3. Compute pack

| Part | Qty | Spec that matters | Status |
|---|---|---|---|
| Raspberry Pi 5 | 1 | 8 GB — YOLO and Nav2 share it | |
| microSD or NVMe + HAT | 1 | | |
| USB-C PD power bank | 1 | **5 V / 5 A capable.** 10000 mAh gives 2+ h at ~14 W active; 20000 mAh gives 4+ | |
| USB-C cable | 1 | Rated for 5 A | |
| Active cooler + vent hardware | 1 | A Pi 5 at 10 W sustained in a sealed printed cube outdoors will thermal throttle, which corrupts the measured YOLO inference rate — a graded deliverable | |
| Hailo-8 AI HAT | 0–1 | **Optional.** Fit it for CPU headroom, which is a real constraint. Not for battery life: it is a ~3 W swing, under 3% of budget | |

The compute pack stays a separate USB-C bank. Two packs remove the
motors-brown-out-the-Pi failure mode entirely rather than mitigating it, and
that reasoning is unaffected by the motion pack changing chemistry.

---

## 4. Sensing and microcontroller

| Part | Qty | Spec that matters | Status |
|---|---|---|---|
| Raspberry Pi Pico | 1 | **PIO required** — ~25,000 encoder edges/s across four wheels. Linux userspace GPIO drops counts, and dropped counts raise no error: they silently under-report distance | |
| 9-DOF IMU | 1 | **BNO055 or ICM-20948.** See §1 | |
| NEO-6M GPS module | 1 | With antenna | |
| HC-SR04 ultrasonic | 5 | Five forward-facing cones — this is the entire obstacle-avoidance sensor suite. No LIDAR | |
| Level shifter | 1 | 5 V → 3.3 V on the HC-SR04 echo lines. The Pico is not 5 V tolerant | |
| USB camera | 1 | Handles litter detection and final approach | |
| INA226 current monitor | 1 | Every power figure in `power_budget.md` is an estimate until this is fitted | |

---

## 5. Arm and mechanical

| Part | Qty | Spec that matters | Status |
|---|---|---|---|
| SO-101 arm | 1 | 6× STS3215 servos + servo bus driver board | |
| Arm mounting bracket | 1 | **Front face, 260 mm above ground.** May not be in the current chassis print — see below | |
| Mechanical arm rest | 1 | Unloads the servos in transit. The arm draws ~11 W holding a stowed pose because the servos actively resist gravity; a rest removes a continuous drain that is easy to overlook | |
| Chassis print | 1 | Also needs a pocket for the battery adapter plate | |

**The 260 mm mount height is measured, not chosen.** From the top plane at
460 mm the gripper bottomed out at 241 mm above ground and could not reach
litter. At 260 mm on the front face it sits at **41 mm** — the arm reaches
219 mm below its own mount. Both figures measured in simulation. Check the
current print actually has this mounting face.

---

## 6. Wiring notes that affect what you buy

The two packs are isolated at signal level, **not galvanically** — the Pico's
USB link to the Pi ties their grounds, and the MDD10A logic inputs are not
opto-isolated, so motor ground is referenced too. This is normal and works, but:

- Run a **heavy, short ground** between Pico and motor driver.
- **Never route logic wiring alongside motor leads** — bundle them separately.
- Fuse each rail independently at the battery. The blade fuse block in §2 is
  how that gets done in one part rather than loose inline holders.

E-stop cuts the **motion pack only**. Compute stays up so the Pi keeps logging
through the event, which is exactly the data worth having.

---

## 7. Open questions before ordering

- [ ] Which brand — DeWalt 20 V or Makita 18 V? Pick whichever ecosystem is
      already in the building; the adapter plate is brand-specific
- [ ] Is the SO-101 hardware already in hand? It exists in simulation only so far
- [ ] Is the chassis printed, and does it have the 260 mm front mounting face?
- [ ] Confirm the chosen motor's actual stall/rated current at 24 V once picked
      (`power_budget.md` §7 tracks this)
- [ ] Re-derive encoder counts-per-revolution from the part actually ordered
