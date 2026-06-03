# Multirotor Mixer — Missing Features

What our [`multirotor_mixer`](Inc/multirotor_mixer.hpp) doesn't do yet, and what
each thing actually means at the stick. Sorted by how much you'd feel it during
flight; pick from the top.

For context, what the current mixer **does** today:
- 4×3 hardcoded QuadX allocation (roll/pitch/yaw torque → 4 motors)
- Proportional saturation rescale on the upper clamp (`range > 1` → scale axes down)
- Idle floor + upper clamp `[idle, 1]`
- Disarmed → zeros

Everything below is on top of that baseline.

---

## Tier 1 — directly visible at the sticks

### 1. Airmode

**What it does.** When you pull the throttle to zero but flick a stick (e.g.
trying to recover from a tilt), the current mixer can't deliver the requested
torque because clamping motors to `idle` swallows the negative side of the
torque spread. The drone loses attitude authority right when it needs it.
Airmode fixes this by *lifting the whole motor band up* by whatever offset is
needed to fit the requested torque inside `[idle, 1]`, even if that means motors
spin faster than the commanded thrust. Net effect: full attitude authority at
any throttle, at the cost of a small unwanted climb when you're trying to
descend hard.

**Why you want it.** Without airmode the drone is uncontrollable in any
maneuver that briefly cuts throttle (flips, hard descents, recovery from
upset). With airmode the response stays consistent across the whole throttle
range. Standard on every FPV/acro stack (Betaflight, KISS, Emuflight).

**Cost.** ~15 LOC inside `Mix()`, no new state, one bool config field.

---

### 2. Battery voltage compensation

**What it does.** A motor's thrust output scales roughly with battery voltage —
draining the pack from 4.2V/cell to 3.5V/cell loses ~16% of thrust authority.
Same stick command late in a flight produces less torque than early. Voltage
compensation multiplies every motor command by
`nominal_voltage / current_voltage` so the *physical* response stays constant
as the battery drains.

**Why you want it.** Without it, PID tuning is implicitly voltage-specific. The
drone feels snappy at full charge and mushy at low charge. Hover throttle
creeps up across the flight. Pilots learn to compensate but it's a real
nuisance and it makes auto-anything (auto-level, auto-land) harder to tune.

**Cost.** ~10 LOC. Needs a voltage reading on every `Mix()` call (we already
have `Battery::ReadVoltageMv()`). One new config field for nominal voltage.

---

### 3. Thrust curve compensation (thrust factor)

**What it does.** Propeller thrust scales roughly with `rpm²`, but the DShot
command is linear in `rpm`. So commanding 50% throttle gives you ~70% of hover
thrust, not 50%. Stick response feels nonlinear — sluggish at low throttle,
twitchy near hover. Thrust factor warps the command through
`out = (1−k)·in + k·sqrt(in)` so the operator sees a more linear thrust feel.

**Why you want it.** Removes throttle-dependent gain variation. Tuning at 30%
throttle stays valid at 70%. Hover throttle ends up near 50% stick (matches
muscle memory). Especially relevant when you add altitude hold — the altitude
controller wants linear thrust → vertical-acceleration mapping.

**Cost.** ~5 LOC. One float config field `thrust_factor ∈ [0, 1]`. Typical
value 0.3–0.7.

---

## Tier 2 — robustness and tuning quality

### 4. Slew rate limit

**What it does.** Caps how fast each motor's command can change per tick:
`|Δmotor| ≤ slew_max · dt`. Prevents step changes from sensor spikes or
noisy PID outputs from being sent straight to the motors.

**Why you want it.** Protects motors and ESCs from ringing on noisy inputs.
Reduces audible "buzz" from PID output noise. Modest latency cost.

**Cost.** ~10 LOC. Adds 4 floats of state (last per-motor command), making
the mixer stateful for the first time. One config field `slew_per_s`.

---

### 5. Yaw-deprioritized saturation

**What it does.** When the motor spread exceeds 1.0, our current rescale
scales *all three axis demands* by the same factor — pitch, roll, and yaw
lose authority proportionally. PX4 (and modern Betaflight) instead drops yaw
*first*, only sacrificing pitch/roll if yaw alone wasn't enough to fit.

**Why you want it.** During a hard maneuver, you'd rather lose heading hold
than lose roll authority and tip over. Yaw mismatch costs you a few degrees
of heading; pitch/roll mismatch crashes you.

**Cost.** ~20 LOC. Replaces the proportional rescale block in `Mix()`. No new
state.

---

### 6. Per-motor disarm value

**What it does.** Currently we emit `{0, 0, 0, 0}` when disarmed. DShot
interprets `0` as `kMotorStop` so this works for us, but a future PWM ESC or
3D ESC might want a different disarm signal per motor. Make the disarm value a
config field instead of hardcoded zero.

**Why you want it.** Freedom to swap ESC types or run 3D mode without touching
the mixer.

**Cost.** Trivial. Drop only if you might ever run something other than
unidirectional DShot.

---

## Skipped (Tier 3 — not needed at this stage)

- **Multi-geometry (hex, octo, Y6, +, V-tail, VTOL)** — we're a quad-X.
  Hardcoded matrix is fine until there's a second airframe.
- **Pseudo-inverse allocation** — required when there are more motors than
  DOFs (hex/octo). N/A for a quad.
- **Reverse motor / 3D mode** — bidirectional DShot for inverted flight.
  Niche, can add later.
- **Online hover-thrust estimator** — needs altitude estimation first.
- **Motor-failure detection + reallocation** — advanced research feature.
- **Servo support / VTOL transition** — not a multirotor concern.

---

## Implementation plan

Add in this order. Each step is independently testable and shouldn't
regress anything above it.

| # | Feature | LOC | New state | New config | Notes |
|---|---|---|---|---|---|
| 1 | Battery voltage compensation | ~10 | none | `nominal_voltage_mv` | Needs voltage on every `Mix()` — change signature OR pass voltage via a setter. Setter is simpler. |
| 2 | Thrust factor | ~5 | none | `thrust_factor` ∈ [0, 1] | One-line warp applied to `in.thrust` before the matrix step. |
| 3 | Airmode | ~15 | none | `airmode` (bool) | Modifies the saturation step: shift motor band up if low side would clamp. |
| 4 | Slew rate limit | ~10 | 4 floats | `slew_per_s` | First stateful change. `Mix()` needs `dt_s` parameter or mixer needs to know tick rate. |
| 5 | Yaw-deprioritized saturation | ~20 | none | none | Replaces existing rescale block. Defer until 1–4 are solid. |
| 6 | Per-motor disarm value | ~3 | none | `disarm_value` | Trivial; do only if you'll run non-DShot ESCs. |

Total Tier 1 (steps 1–3): ~30 LOC, 3 config fields, no architectural changes.

### Suggested approach

1. **Step 1 first** — single multiplier, almost free, and battery comp is the
   feature you'll notice the fastest in real flights.
2. **Step 2 next** — also a single transformation, makes step 3's tuning more
   predictable.
3. **Step 3** — airmode. The big flight-feel change. Validate by recording PID
   error during fast descents and confirming attitude error stays bounded.
4. **Stop here** unless you've actually felt a need for slew limit / yaw
   deprio / per-motor disarm. Don't add complexity ahead of demand.

### One open question

Steps 1 and 4 both want runtime inputs (`voltage`, `dt`). Two options:
- **(A)** Change `Mix(const Inputs &in)` to `Mix(const Inputs &in, float voltage_mv, float dt_s)`. Cleanest but breaks the existing call site.
- **(B)** Add setters `SetBatteryVoltage(mv)`, `SetDt(dt)` called from the
  state machine. Keeps the `Mix()` signature stable; mixer becomes slightly
  more stateful.

Recommend **(A)** — the mixer's job is to take *current* inputs and produce
*current* outputs. Hiding state in the mixer when it's all latency-sensitive
makes debugging harder.
