# 32Raven — Roadmap

The single source of truth for **32Raven firmware and handbook work**. Every `TBD(#N)` marker
in the handbook points to an item here, and `scripts/lint/check_docs.py` fails the build if it
points at an item that does not exist — so a placeholder cannot be quietly forgotten.

**Scope — this repository.** Firmware and handbook work only.

**Priority legend**

- 🎯 **CRITICAL** — blocks the prototype build, blocks the handbook being usable by someone
  who is not the author, or leaves the firmware in a shape that other queued work has to
  build around.
- 🟢 **SUPPORTING** — real work; do it when the area is being touched anyway.
- 🧊 **DEFERRED** — real work, parked deliberately.

---

## 1. Handbook

The build guide is being written alongside the first from-scratch prototype assembly. These
are the gaps that assembly is expected to close.

### #1 — Prototype photograph — 🎯 CRITICAL

`docs/assets/prototype-placeholder.svg` is a drawn placeholder. Replace it with a photograph
of the assembled aircraft.

Highest-value single addition to the site: it answers "what am I building?" faster than any
paragraph, and it is the one thing a reader deciding whether to start actually looks for.

- Shoot the finished aircraft against a plain background, props off.
- Downscale to ~1600 px wide and save as WebP or JPEG under `docs/assets/`.
- Update the image reference in `docs/index.md`.

### #2 — Cost and build time — 🎯 CRITICAL

The **Parts cost** and **Build time** cards on the handbook index are `TBD(#2)`.

Nearly every DIY build guide omits both, and they are the first two questions a reader has.
Fill them from the actual build rather than an estimate:

- Total parts cost, with the currency and the month priced — component prices move.
- Assembly time and bring-up time as separate figures. They are different activities and
  people budget them differently.

### #3 — Airframe specifications — 🎯 CRITICAL

The **Airframe** card is `TBD(#3)`, and **Power** is missing capacity and all-up weight.

Needed: frame class and prop size, motor and ESC selection, battery capacity, all-up weight,
and measured flight time. Weight and flight time must come from the built aircraft — an
estimate here is worse than the blank, because a reader will size their own pack from it.

### #4 — The remaining stubbed build stages — 🎯 CRITICAL

Materials, boards and wiring have pages. Still to write: frame and motors, power, toolchain,
configure and flash, smoke test, sensors, RC link, bench test, and first flight.

Write each as its stage is reached during the real build, while the details are fresh and
the mistakes are still visible. A stage documented from memory six months later is the kind
that omits the step that actually caused the problem.

### #6 — Buzzer — 🟢 SUPPORTING

The buzzer is visible in the photographs on **The brain** and wired to `GPIO10`, but its
section on **Sensors and peripherals** is a `TBD(#6)` stub.

Needed: the part used and whether it is an active or passive type, how it is wired to
`GPIO10`, where it mounts, and what the firmware actually sounds it for.

---

## 2. Firmware

### #7 — Divide the STM32 into real states — 🎯 CRITICAL

Mostly delivered. `IdleState`, `ArmedState` and `EscConfigState` are three real states sharing
`StepFlightLoop`/`ControlTickFlightLoop`, and the interlock is structural rather than remembered:
`EscConfigState` clears the control-tick hook so the mixer does not run, and `Sentinel::RequestArm`
refuses from any state but Idle instead of each arming path checking for itself.

Two pieces remain:

- **Rename `IdleState`.** It runs the full flight cascade — the name is left over from the
  single-state design and misdescribes the code.
- **A failsafe state**, once #15 decides what a trip does. `failsafe_flags` is still hardcoded
  to `0` in `BuildVehicleStatusMsg`, with the `TODO(fc)` still on it.

### #9 — Share the FcLink frame parser — 🧊 DEFERRED

The byte-at-a-time receive state machine exists twice, in `stm32/Services/fc_link.cpp` and
`esp32/services/fc_link.cpp` — same seven states, same transitions, same field accumulation,
plus an `RxState` enum and an `rx_pkt_internal_` struct declared identically in both headers.

The reason to fix it is not the line count. **CRC verification is implemented twice,
differently.** The STM32 rebuilds header and payload into one contiguous buffer and runs
`checksum::XModem` over it; the ESP32 feeds `XModemUpdate` byte by byte across magic, id, length
and payload. The two agree only for as long as `message::Header` stays packed as exactly
`{magic[2], id, len}`, and a divergence would not fail loudly — the link would simply stop
carrying packets.

What differs between the two sides is policy, not parsing. The STM32 resyncs silently and
dispatches inline; the ESP32 counts invalid frames, logs them, sounds the error tone, panics
past a threshold, and queues rather than dispatching.

- Add a policy-free parser to `libs/inc/`, alongside `ring_buffer.hpp` and `dispatcher.hpp`:
  one byte in, one verdict out — need-more, bad magic, bad length, bad CRC, packet complete —
  so each firmware keeps its own reaction to each verdict.
- Leave `Poll`, transmit, handshake, the ring buffers and the read budgets where they are.
  Those genuinely differ: an interrupt-fed byte ring on one side, block reads from the ESP-IDF
  driver on the other.
- Folds away the `should_alert_invalid` lambda the ESP32 currently writes out twice in the same
  file.

Deferred rather than supporting because it rewrites the receive hot path on both firmwares. Do
it once the ESC configurator work has been confirmed on hardware, so that a misbehaving bench
session has one candidate cause instead of two.

### #10 — Derive the motor count from the airframe — 🧊 DEFERRED

The airframe is one physical fact about the vehicle, and it is declared in two places that
cannot see each other.

Neither MCU offers a choice today, and each hardcodes the frame separately. The ESP32 fixes
`kMavlinkSysAutostart` at 4001 in `generate_esp32_config.py` and hardcodes `MAV_TYPE_QUADROTOR`
in the heartbeat; the STM32 references `QuadX::kFactors` directly at six sites in
`multirotor_mixer.cpp`. The `ESP32_MAVLINK_SYS_AUTOSTART` choice that used to offer `QUAD_PLUS`
is gone — it was a knob whose only non-default position lied to the ground station about a frame
the STM32 could not mix.

There is no damage today, because neither side can be pointed at anything but an X quad. What
makes this worth recording is that the moment either side gains a real geometry selection there
are two sources for one truth, on two MCUs that flash independently. This is
the same shape as the FcLink baud and exchange interval, and it wants the same answer — one
airframe choice in Kconfig, with motor count *and* geometry table derived from it into
`common_config`, rather than a standalone motor-count integer that can disagree with the frame
it describes.

Three things resist a motor count that is not 4:

- **TIM1 has four compare channels.** DShot is one burst-DMA transfer per bit, `DCR` configured
  with base `CCR1` and length 4 (`DBL`), so all four motors are driven from one stream and stay
  perfectly synchronised. Six motors needs TIM8 and a second DMA stream, with both bursts
  started together — two frames arriving at different times reads as a yaw bias.
- **`MotorThrust = std::array<float, 4>`** threads through `Mix`, `MixOutput`,
  `EscService::WriteMotorsThrust`, and back into the rate controller's anti-windup via
  `applied_torque`. Widening it touches the whole cascade signature.
- **The literal 4 is written five times** with nothing checking that they agree:
  `dshot_codec.hpp:11`, `esc_telemetry.hpp:13`, `dshot_tim1.hpp:54` (`kMotors`), and
  `multirotor_mixer.hpp` twice (the `MotorThrust` alias and `kFactors[4][3]`).

What already scales without change: `FourWayService` validates its channel against
`DShotCodec::kMotorCount` and stores one `selected_esc_`, MSP already reports eight motor slots,
and `EscTelemetry` round-robins `expected_motor_` over a single USART. Fewer motors than four is
also cheap — leave the unused channels configured and idle, which costs a pin and some DMA
bandwidth but changes no structure.

Deferred because no non-quad airframe is planned. Unifying the declaration is the half worth
doing first if the mixer is being touched anyway; the timer work only becomes real when a frame
with more than four motors does.

### #11 — Warn the pilot when an ESC starts derating — 🎯 CRITICAL

AM32 carries its own thermal and current limits at settings-page bytes 43 and 44, and starts
pulling power back when a motor reaches them. The flight controller already reads the matching
telemetry — `Sample::temperature_c` and `Sample::current_centiamps` — but knows neither
threshold, so a derate arrives as unexplained thrust loss. The rate controller responds by
winding up I-term against a limiter it cannot see.

Both bytes sit in the 17–46 window that is identical across AM32 eeprom layouts 2 and 3, so
reading them needs no version handling, only two more fields in `EscTelemetry::Info`.

The harder half is the warning itself. There is no screen on the aircraft, but CRSF reaches the
transmitter and two mechanisms already exist:

- **`CRSF_FRAMETYPE_FLIGHT_MODE` (0x21)** carries a free-text string that EdgeTX and OpenTX
  render as the `FM` telemetry field. Betaflight and INAV use it for `!ERR` and `!FS!`.
- **Temperature as a telemetry sensor (0x0D)** lets the radio's own Logical Switches and
  Special Functions fire a voice callout, which beats a screen nobody is looking at mid-flight.

Both frame types are already listed in the `TODO(crsf)` at `crsf_link_service.hpp`, so this
lands as part of filling that table in rather than as new transport work.

Deliberately not a panic: an ESC derating is a degraded aircraft the pilot may still want to
land, not a reason to stop the motors.

### #12 — Decide what a desync looks like from the flight controller — 🎯 CRITICAL

`stall_protection` (byte 29) and `stuck_rotor_protection` (byte 22) change what the ESC does
when a motor loses sync: whether it cuts, retries, or keeps trying to drive a rotor that is not
turning. Each choice presents differently in telemetry — eRPM collapsing to zero, current
spiking, or both recovering after a pause — and the flight controller currently interprets none
of it. A desync today reads as a motor that simply stopped producing thrust.

Both bytes are in the version-stable window, so the settings are readable now. What is missing
is the decision of what the FC should do with each combination, which has to come before any
detector: a mixer that compensates for a motor the ESC is about to restart makes the recovery
worse than doing nothing.

Settle the policy against the ESC's configured behaviour first, then decide whether the
firmware check should constrain those two settings the way it now constrains input type and
direction.

### #13 — Signed thrust, so 3D mode means something — 🧊 DEFERRED

The thrust chain is unsigned end to end. `MultirotorMixer::Mix` clamps every motor to
`[idle, 1]`, and `EscService::ThrustToDshot` maps that onto `kMotorStop` plus
`[kThrottleMin, kThrottleMax]` — stop or forward, with no third case. Nothing in the cascade
can ask a motor to push the other way.

Two halves of the feature are already in the tree, which is why this is worth recording rather
than leaving implicit. `EscService::DshotCommand` declares `k3dModeOff` and `k3dModeOn` and
neither is ever sent, and `EscTelemetry::Info::bidirectional` is parsed from settings byte 18.

What it needs: signed thrust `[-1, 1]` through the mixer and both controllers, a three-way
`ThrustToDshot` over the 3D split (`48–1047` reverse, `1048–2047` forward), `k3dModeOn` actually
issued, a configurator path to enable it, and a settings re-read to confirm the ESC took it.
`kEscDirectionReversed` has to be revisited in the same pass — with 3D on, which way a motor
turns stops being the fixed property that check assumes.

Deferred because no planned flight mode wants it. Autolevel and autonomous flight only ever ask
a motor for *less* lift, never for lift in the other direction; reverse thrust is an acro
capability, and this airframe is not being built for acro.

Until then the mode is refused rather than flown. `EscService::CheckEscFirmware` panics with
`kEsc3dModeEnabled` when a motor reports it, because an ESC in 3D reads everything below half
throttle as reverse — so the forward-only values `ThrustToDshot` produces would drive that
motor backwards across the bottom half of its range, on an aircraft whose mixer believes it is
commanding lift.

### #14 — The gyro decimation aliases into the loop band — 🟢 SUPPORTING

`Ahrs::Update` reduces each FIFO batch to one sample by averaging it:
`state_.gyro_body_rad_s = gyro_accum / batch.count` in `ahrs.cpp`. That is a boxcar of eight
records at 8192 Hz decimated to the 1024 Hz loop, and a boxcar is a weak anti-alias filter —
its first null lands at 1024 Hz, while the rate that matters is Nyquist at **512 Hz**.

The gyro's own anti-alias filter is configured at **585 Hz**
(`CONFIG_STM32_IMU_GYRO_AAF_585HZ`), which is above that. Everything between 512 and 585 Hz
therefore folds down into the loop band, where it arrives indistinguishable from real airframe
motion and the rate controller acts on it. Prop wash and frame resonance both live up there.

Raising the loop to 2048 Hz is the fix: Nyquist moves to 1024 Hz, clear of the AAF corner.
Lowering the AAF instead does not work — the next step below 585 Hz is 536 Hz, still above a
512 Hz Nyquist, and the one after that is 258 Hz, which costs more loop bandwidth than the
aliasing does.

Two constraints to carry into that change:

- **The 8192 Hz record rate exists because CLKIN is enabled.** The datasheet quotes the 8 kHz
  ODR against a 32 kHz reference, and the 32768 Hz external clock scales it. 2048 divides 8192
  exactly; it does not divide the 8000 Hz the part produces on its internal oscillator. So
  raising the loop rate and disabling `STM32_IMU_EXTERNAL_CLOCK_ENABLED` are mutually
  exclusive, and the generator already rejects the combination.
- **`STM32_CONTROL_LOOP_HZ` stays a knob.** It sets the FIFO watermark rather than the other way
  round, and the build already fails when the division is not whole. Deriving it from the ODR
  would remove the one place this trade-off is visible.

**Raising the rate is not the only fix, and Betaflight offers both.** Its downsampler runs a
PT1 over every raw sample and the PID takes the filtered value -- filter-then-decimate, where
the filter state carries every sample forward. Disable that filter and it falls back to
`sampleSum / sampleCount`, a plain boxcar: *exactly what we do*. So this is a mode choice on a
dial they also expose, not a stage we are missing. Their PT1 would not transfer as-is -- at
-6 dB/octave it is barely down one octave above our 512 Hz Nyquist, where their denom-2
Nyquist of 2 kHz gives them enormous margin. #23 covers the filtering properly, including
notching the motor harmonics *before* the decimation, which is the version that suits an 8:1
ratio.

Not urgent on its own: it is a fidelity limit rather than a fault, and it has been flying this
way. Worth doing at the same time as any other change to the control loop, since the watermark,
the loop rate and the filter corner all have to move together.

### #15 — Nothing acts on a lost link — 🎯 CRITICAL

`StatPublisher::BuildVehicleStatusMsg` now reports `kVehicleFailsafeFlagRcLoss`, and that is all
it does. No code anywhere changes behaviour when a link goes down.

The control loop reads `rc.roll_us`, `rc.throttle_us` and the rest straight out of `VehicleState`
with no freshness test, so RC loss means the last stick values are held and flown indefinitely.
`states.cpp` says so where the cascade starts — *"No tx_online check yet — disarmed mixer +
disarmed ESC means worst case is harmlessly computing zeros from stale RC."* That is true only
because arming is currently reachable solely through `kPrivilegedArm` over FcLink.

**The tripwire: stick arming must not land before this does.** The moment a stick gesture can
arm, holding stale sticks stops being harmless.

#### Detection is a timestamp away

The tick-maintained `rx_online`/`tx_online` pair is gone; freshness is derived at read time
from `RcData::timestamp_us`, and each consumer picks its own bound. `StatPublisher` uses 1.5 s
because it only has to answer "is the GCS being shown a live link".

So the control path needs its own age test against the same field, not a new detector.
Betaflight triggers at 150 ms and PX4 at 500 ms, and recovery wants the opposite hysteresis
from an indicator — Betaflight demands 1 s of clean data plus low throttle before handing
control back, where a display should be quick to recover and slow to drop.

A timeout is the only detector available: CRSF carries no receiver-asserted failsafe bit, and
ExpressLRS signals loss by *stopping* RC frames rather than flagging them.

#### The action is a real choice

Betaflight offers `DROP_IT`, `AUTO_LANDING` against a tuned `failsafe_throttle`, and GPS
rescue. This airframe has no barometer and no position controller, so a timed descent would be
an invented constant with nothing closing the loop on it. Disarm is the honest action today;
autoland belongs behind an altitude backbone. Make it a knob either way — all three reference
stacks parameterise it.

#### Both links have to be considered together

RC loss should not be fatal while the pilot still holds authority over the telemetry link, and
with a 900 MHz radio as the primary link that is the *expected* case rather than an edge one —
it will likely outlive 2.4 GHz control. PX4 models this as two independent conditions,
`manual_control_signal_lost` and `gcs_connection_lost`, each with its own action parameter.

Two pieces are missing before that rule can be evaluated at all:

- **The GCS cannot arm or disarm.** `mavlink_cmd.cpp` handles `START_RX_PAIR`,
  `REQUEST_MESSAGE` and `PREFLIGHT_CALIBRATION`; everything else returns `MAV_RESULT_UNSUPPORTED`.
  `MAV_CMD_COMPONENT_ARM_DISARM` needs wiring to the `kPrivilegedArm` path that already exists.
- **The STM32 cannot tell whether a GCS is connected.** `MAVLINK_MSG_ID_HEARTBEAT` reaches
  `Mavlink::HandleMessage` and only blinks the LED — no timestamp kept, nothing forwarded over
  FcLink.

FcLink peer loss is the third condition this rule reads, and it belongs to #16 and #17 —
each side needs its own, because the premise is that the other side is gone.

#### Arming interaction

Whatever the action, it needs PX4's `_manual_control_lost_at_arming` rule — if RC was already
absent when the vehicle armed, its absence must not count as a loss until RC has been seen at
least once. Otherwise arming over FcLink with no transmitter powered on disarms instantly and
the bench path stops working. The reporting side already takes this shape, gating the flag on
`rc.timestamp_us != 0`.

#### The other three flags

`kVehicleFailsafeFlagBattery` needs a threshold knob chosen; the voltage and percentage are
already on the blackboard. `kVehicleFailsafeFlagImu` has both a detector and a written intent
waiting — `states.cpp` currently calls `Panic()` on sustained IMU frame loss and says halting
is the wrong answer once this flies with props. `kVehicleFailsafeFlagGps` stays zero until
something actually navigates by GPS, since there is no consequence to report yet.

This item is the *policy*: which conditions count, how fast, and what each one does. Where that
policy lives and what enforces it is #16.

### #16 — Sentinel, one owner for arming and failsafe authority — 🎯 CRITICAL

Authority over whether this vehicle may fly was spread across four files. No single place could
answer "may this vehicle fly, and what must it do right now". Sentinel is that place: it owns
the arm decision, evaluates every failsafe condition #15 defines, writes the result to the
`SharedState` once per slow loop, and is the only thing that disarms. `StatPublisher` then
reduces to copying that word into the message, and the control loop reads the same word rather
than re-deriving it — one predicate, not one per consumer.

#### The arming half exists

`Sentinel::RequestArm` is now the only entry point for an arm request and the only writer of
`SharedState::armed_`, which is private to it. The duplicate `armed_` members in `EscService`
and `MultirotorMixer` are gone — both read the one representation, and `EscService` still
checks at the wire as defense in depth. The ESC-configuration interlock moved out of
`CommandHandler`, which now only turns a refusal back into a tone. The transition orders its
effects so the flag sits at its safe end throughout: cleared before the stop frames on disarm,
set only after the ESC path is clean on arm, leaving any control-loop preemption reading disarmed.

An RC arm switch is the second caller this shape exists to make cheap, and does not exist yet —
`RcReceiver::Config` maps roll, pitch, yaw and throttle and nothing else, so it needs a channel
mapping, a Kconfig knob, calibration schema and a throttle-low interlock before it is a
one-line addition.

#### What is left

Failsafe conditions are still computed in a telemetry builder
(`StatPublisher::BuildSystemStatusMsg` derives IMU, GPS, battery and RC health from freshness),
`VehicleStatusMsg::failsafe_flags` is still hardcoded to zero, and sustained IMU frame loss
still calls `Panic()` from the middle of the slow loop. Moving detection into Sentinel means
*relocating* that health computation rather than writing a second copy of it — writing one
would recreate exactly the duplication the arming half removed. `Sentinel::Supervise` is the
slot it lands in — it exists, and today watches only the sample path (#20).

**Two panics, not one, and the second is the harder of the two.** Alongside the slow-loop guard,
`Icm42688p::HandleOverrunFault` counts overruns into a sliding window and calls
`Panic(kImuOverrun)` once `overrun_threshold` of them land inside `overrun_window_s`. It carries
no TODO and it is worse placed than the one that does: it fires from **interrupt context**, so
the armed state a correct response depends on is not merely inconvenient to reach but arrives
too late to matter — the decision has already been taken by the time any loop could weigh it.
An ISR can raise a condition; it cannot choose a response. So the driver's job ends at counting
and flagging, and the threshold that currently halts has to become a flag Sentinel reads,
which also lets one policy cover both detectors instead of each carrying its own halt.

**It must live on the STM32**, because its whole purpose is to keep working when the ESP32 is
gone. That is also why FcLink peer loss is Sentinel's to detect on this side (#17 owns the
other): `FcLink` exposes `Poll` and the `Send*` helpers and nothing that notices silence, and
since arming currently lives behind FcLink, an ESP32 failure while armed removes the disarm
path entirely and leaves the cascade running on the last RC values.
`VehicleStatusMsg::failsafe_flags` has 28 spare bits for the flag.

#### What "high priority" means without an RTOS

The STM32 is a superloop, and `StepSlow` sheds work at `budget_exhausted()` bail points, so
priority is placement rather than a number: Sentinel's `Poll` has to run **above the first
bail**, since a failsafe skipped under load is a failsafe that does not exist.

`MspSvc().CheckArmed()` used to hold that slot and was the precedent for it — a per-slow-loop
poll revoking the ESC-configuration grant if the vehicle armed underneath it. It is gone: with
`RequestArm` refusing from any state but Idle, and `SharedState::SetArmed` private to Sentinel,
arming during a session became unreachable rather than something to watch for. That is the
shape to keep aiming at — an interlock that makes a state impossible costs nothing per pass,
where one that polls for it costs something forever.

#### Sentinel should own the watchdog

`Wdg().Kick()` sits unconditional in the superloop in `main.cpp`, so it proves only that the
superloop iterates. A wedged control loop under a healthy superloop is still fed — at
`STM32_CONTROL_LOOP_HZ` of 1024 against a ~1 s IWDG, that is roughly a thousand missed control
cycles that nothing detects. Kicking only after Sentinel has confirmed the control-loop counter
advanced turns the watchdog from "the loop turns" into "the control loop is running".

Two constraints:

- **`EscBootloader` also kicks, deliberately.** During ESC flashing the control loop is not the
  liveness criterion, so the kick policy has to be mode-aware or that path resets mid-write.
- **WWDG is unused and is not redundant.** The F407's second watchdog is a *window* watchdog
  off PCLK1: it resets when fed too early as well as too late, catching a loop running
  wrong-fast, which IWDG structurally cannot. Its ceiling is short — order of 50 ms at typical
  PCLK1 — which suits it to the 1024 Hz control loop while IWDG stays on the superloop. Unlike
  IWDG it runs off the main clock, so it does not survive a clock failure; the two cover
  different faults and want both.

### #17 — Doctor, health observation on the ESP32 — 🟢 SUPPORTING

The counterpart to #16, and the split between them is **authority, not location**. Sentinel
decides and acts; Doctor observes, correlates and reports, and has no authority at all. Draw
the line anywhere else and conditions like IMU health become ambiguous — with this line, Doctor
describes the IMU's condition and Sentinel decides whether that condition grounds the aircraft.
Doctor being wrong must be harmless, which is what allows it to live across a link that can die.

It does not duplicate `StatPublisher`, which reports STM32-local health because only the STM32
can see it. Doctor adds what only the ESP32 can see — heap, task stack headroom at run time
rather than only through `esp32_stack_check.py` statically, WiFi, flash, transport link stats —
and correlates across FcLink. FreeRTOS makes it a task with a real priority, unlike #16.

#### The part worth doing before the service exists

`Mavlink::StartHeartbeatFrame` freshness-checks `system_status_` against
`kMavlinkSystemStatusFreshMs`, but reads `vehicle_status_` gated only on `have_data`. If the
STM32 goes silent, the ESP32 keeps reporting the last-known armed state, flight mode and
failsafe flags to the GCS as current — `MAV_MODE_FLAG_SAFETY_ARMED` in particular would keep
asserting *armed* indefinitely from a dead flight controller. The stale `system_status_` does
force `MAV_STATE_CRITICAL`, so it is half-caught, but a GCS reading the mode flags is being
told something false. Not reporting stale cross-link data as current is Doctor's remit, and
this instance is a small fix that does not need to wait for it.

#### Constraint

Doctor reports to MAVLink and the logs, **not to the OLED**. The display stays a bench tool and
flight state stays off it; a health service naturally tempts a status screen, so the boundary
belongs in its definition rather than in review comments later.

### #18 — Flash the two firmwares as one thing — 🟢 SUPPORTING

Both MCUs compile `libs/inc/message.hpp`, so the FcLink wire format is a contract between two
images that are flashed separately. When they disagree, `IsPayloadLengthValid` rejects the
mismatched frames and the link degrades silently — no panic, no log, just a stream that stops
arriving. Nothing in either firmware detects it, and nothing in the build system prevents it.

The only protection today is remembering to flash both. That held while the wire format was
stable; narrowing `RcChannelsMsg` from 36 bytes to 20 is the first change that would have
punished forgetting, and `static_assert`s catch only the half of it that lives inside one
build.

#### What already exists

- **The ESP32 owns the STM32's only flash path.** `Programmer` drives BOOT0 and the shared
  FcLink UART, so "one device flashes both" is mostly wiring parts that are already there.
- **The ESP32 already has a version.** `generate_esp32_config.py` emits
  `kMavlinkFlightSwVersion` and `kMavlinkGitHashShort` from the repo version and git HEAD, for
  MAVLink `AUTOPILOT_VERSION`.
- **The STM32 has none**, and the FcLink handshake that would carry one is a bare `kPing` with
  a zero-length payload. It runs at boot with retries already, so it is the natural place to
  exchange versions without inventing a message.

#### Shape

- `make flash-wifi` decides what to flash rather than flashing everything: read both versions,
  compare against the build, skip what already matches.
- Version the *protocol*, not the build. A build hash forces a reflash of both MCUs for a
  comment change; what actually has to match is the message layout. Deriving the value from
  the message definitions — a compile-time hash over the wire structs — makes it impossible to
  change a struct and forget to bump it, which is the failure this item exists to prevent.
- Keep a build identity too, but as information rather than a gate: knowing *which* image is
  stale is what turns "they disagree" into "flash the ESP32".
- If it becomes one addressed binary with the ESP32 flashing the STM32 and then itself, order
  matters: the ESP32 last means a failure leaves a matched STM32 and a stale ESP32, which is
  recoverable over WiFi. The reverse can leave a half-flashed ESP32 that can no longer program
  the STM32 it was meant to fix.

#### What to do about a mismatch in flight

Refusing to arm on a protocol mismatch is Sentinel's call, not the flasher's — see #16. The
flasher's job ends at making the mismatch visible and easy to fix on the bench.

Supporting rather than critical only because the manual practice works. It stops being enough
the moment someone other than the author flashes this aircraft, or a wire change lands
alongside a flight-day rebuild.

### #19 — Give every Blackboard field an owner the compiler knows about — 🟢 SUPPORTING

`SharedState` is a const-correct store, not an access-controlled one. Readers get `const &` so
they cannot mutate shared state, but every `Update*` is **public**, so the pattern governs *how*
a write happens and never *who* performs it. `UpdateImu` is meant to be the AHRS path's alone
and `UpdateRc` `RcReceiver`'s alone; nothing says so and nothing checks. The setters are now
plain assignments — the `updated`-flag side effect that once justified them had no consumer and
is gone.

`armed_` and `failsafe_flags_` are the fields with a real owner — private, with
`friend class Sentinel` — because a second writer there spins motors. That asymmetry is now
visible in the file: nine public setters and two private ones.

#### The failure it prevents is one this repo already had

`EscService` and `MultirotorMixer` each held their own `armed_`, kept in step only by
`CommandHandler::OnPrivilegedArm` remembering to write both. A second disarm path — RC
failsafe, battery cutoff, the IMU fault handler — that updated one and not the other would have
left the ESCs disarmed while the mixer kept producing torque, or the reverse, with no gate to
catch it. #16 fixed that for `armed`. Nothing prevents the same shape recurring on any other
field, and the next one will not announce itself either.

#### Shape

The passkey idiom, one key per producer:

```cpp
class GpsKey { friend class M10Service; GpsKey() = default; };
void UpdateGps(const GpsData &data, GpsKey) { gps_ = data; gps_.updated = true; }
```

`UpdateGps` stays public, but only `M10Service` can construct the key, so only `M10Service` can
call it. An empty class is elided entirely — identical generated code, no RAM, no indirection
in the 1024 Hz path. Roughly twenty lines in `shared_state.hpp` plus a `{}` at each call site.

It also narrows what #16 currently grants: `friend class Sentinel` opens *every* private in
`SharedState` to Sentinel, including `gps_` and `imu_`. A key exposes exactly one function.

#### Why not the alternatives

- **A lint** counting call sites per setter measures a proxy. The property wanted is exclusive
  ownership — not how often the owner writes, but whether anything else can. C++ states that
  directly, and a compile-time guarantee needs no exceptions file. The call-site count itself is
  not expressible in the language, and is not worth having.
- **Attorney-client** is equivalent in power and cost, and keeps producer names out of
  `shared_state.hpp`. Rejected because that coupling is wanted here: one file declaring who owns
  what is the ownership map this codebase lacked, and adding a producer becoming a visible edit
  to the shared store is the reviewable-act property, not a cost.
- **Pimpl** solves compilation firewalling and ABI stability, not access control — the interface
  stays public to everyone — and needs either a heap (forbidden) or a pointer chase on every
  `GetImu()`.
- **A writer handle** (`SharedState::GpsWriter`, constructed once and stored by its producer) is
  the most principled: it is the only option giving real least privilege, since services today
  hold a whole `SharedState *` and can call every setter. Costs a pointer per writer, makes the
  single-owner property runtime rather than compile-time, and touches every service's `Init`.
  Revisit it if the replay harness needs to substitute a producer — a handle is trivially
  redirectable and a passkey is not.

### #20 — Sentinel watchdogs on the IMU and the control path — 🎯 CRITICAL

Two panics decide policy where Sentinel cannot see them, and nothing watches whether the fast
loop runs at all. The groundwork is done: every cause has its own counter, `ImuHealth` is on
the Blackboard, and `Sentinel::Supervise` runs off the slow tick.

`Icm42688p::HandleOverrunFault` calls `Panic(kImuOverrun)` once `overrun_threshold` faults land
inside `overrun_window_s`. That is a judgement — how broken is too broken — taken inside an
interrupt at the highest priority on the board. #16 gave Sentinel that authority over arming;
this is the same question about a different subsystem, answered where Sentinel cannot overrule
it. An ISR can raise a condition; it cannot choose a response.

#### The control-path guard is now armed, and unmeasured

`StepSlow` panics with `kImuDroppedFrame` after `kLossPanicConsecutiveSec` seconds above
`kLossPanicPerSec`. It read a structurally-zero counter for its whole life. `missed_samples`
now has a live writer in `PublishLatestBatch`, so it can fire for the first time — on a
threshold never checked against hardware, and with a `TODO(fc)` beside it already saying that
halting is the wrong answer once this flies with props.

What it counts is real. `PENDSVSET` is a bit, not a queue: two bursts published inside one fast
tick collapse to one PendSV run and the older is dropped. That is the measurement that says
whether the cascade fits its budget, and the one that would tell us whether a `CONTROL_LOOP_HZ`
bump landed safely.

#### Nothing watches whether the control loop runs

`StatPublisher::BuildSystemStatusMsg` sets `msg.flags = kSystemStatusFlagLoopAlive`
unconditionally, and the ESP32 gates `MAV_STATE_CRITICAL` on that bit. A SystemStatus frame
needs only the *slow* loop, so a wedged control loop leaves every indicator on the ground green.
The IWDG misses it too: the main loop kicks it, and the main loop is fine.

`Supervise` covers the neighbouring failure, not this one — it watches
`ImuHealth::timestamp_us`, which the *sample interrupt* stamps, and recovers a stalled sample
path through `Icm42688p::RestartSampling`. Liveness is the opposite case: the interrupt
publishing normally while the control loop fails to consume. It needs a stamp the control loop
writes, which is also what makes the flag mean its name.

#### What is left

The driver counts and recovers; it does not judge. `FlushAndResync` and `resync_pending_`
stay in the ISR as mechanism. The thresholds, the window and both `Panic` calls leave, and
Sentinel raises `kVehicleFailsafeFlagImu` instead — a halt stays right while disarmed, which
Sentinel is the only thing positioned to know.

Two decisions gate it, now that `failsafe_flags_` has a home: what a trip does in flight, and
whether that loss threshold stands now that it is live code rather than a hypothesis.

**The second cannot be answered yet, because the numbers cannot leave the board.** `ImuHealth`
carries nine counters and all three consumers reduce them to booleans -- an LED blink, a health
bit, and the panic comparison. `SystemStatusMsg` has no field for any of them. So setting an
honest threshold needs one of them on the wire first, read across the four cases that actually
load the loop: disarmed idle, armed at motor idle, throttle sweeping, and every link streaming
at once. The same measurement decides whether `CONTROL_LOOP_HZ` can double, so it is owed either
way. Until then a panic is the loudest instrument available and no props are fitted, which is
why both sites stay as they are.

One gap `Supervise` does not close: a stall with `inflight_` stuck true — a DMA that started
and never completed — survives the flush, because nothing clears it or the SPI driver's
`busy_`. Aborting a live DMA is real surgery and belongs here rather than in the driver.

### #21 — Know why the board restarted — 🟢 SUPPORTING

`RCC->CSR` records what caused the last reset — power-on, brownout, IWDG, window watchdog,
software, pin — and holds it until `RMVF` is written. Nothing reads it. So an in-flight
watchdog reset is indistinguishable from a cable glitch: the FC silently restarts and the GCS
sees the handshake replay. #20 names one concrete route to exactly that.

Capture it at boot, clear it, and carry it in `SystemStatusMsg`. Three lines, and it turns a
silent restart into a reported one.

#### It is a prerequisite for gyro calibration, not a nicety

`Init` zeroes `OFFSET_USER` on every boot and nothing puts a value back, so #16's MAVLink
calibration work will add a trigger for `CalibrateGyro`. Running that after a *non*-power-on
reset either trips `kImuCalibrationMotionDetected` or writes the motion permanently into the
chip's offset registers. "Only calibrate on a power-on reset" is the check that prevents it,
and it needs the cause to exist first.

#### In-flight restart recovery is a separate project — 🧊 DEFERRED

Persisting prior state is the easy half, and it belongs in a `.noinit` SRAM section rather than
on the ESP32: a reset does not clear SRAM, so the marker is readable microseconds in, with no
FcLink handshake to wait on and no second MCU to depend on. A brownout deep enough to lose that
RAM took the ESP32 with it anyway -- they share a battery.

The hard half is that the attitude is gone. Gyro bias is a second-order term -- `Ahrs::bias_`
already estimates it in flight, and a few degrees of drift is nothing beside a quaternion reset
to identity. In free fall the accelerometer reads about zero g in every direction, so it cannot
say which way is up: rate can be held, level cannot be recovered.

Auto-arming also inverts #16's charter, where arming is a deliberate act behind interlocks. A
bench reset would spin props on the table unless a trustworthy airborne test gates it, and
sustained near-zero-g is about the only honest one -- a signal that only arrives once things
have already gone wrong. Fixing what causes the reset is worth more than recovering from it.

### #22 — Nothing finds dead code — 🟢 SUPPORTING

`-Wunused` fires only for internal-linkage functions, so an unused public header-inline accessor
is invisible to every build: no TU odr-uses it, so no TU emits it, so it costs no flash and
raises no warning. `Ahrs::Current()` survived that way until it was found by reading. Three tools
each catch part of the gap, and they do not overlap:

- `-Wl,--print-gc-sections` -- ground truth on the binary. Finds unused data, vtables and
  transitively dead code. Blind to never-emitted inline functions, which is most of what
  accumulates here.
- `cppcheck --enable=unusedFunction` -- source-level, so it does see inline accessors. Weak on
  virtual and function-pointer dispatch, and needs the whole program in one pass.
- clang `-Wunused-private-field` -- the only one that finds unused *data members*. Needs nothing
  but `-fsyntax-only`; clang is on the host already, though not in the Docker image.

All three stay advisory. The confidential SIL consumes this repo's public API, so a whole-program
"unused" verdict from inside the repo is the exact false positive that rule exists to catch --
a list to review, never a build gate.

### #23 — Nothing filters the gyro in software — 🟢 SUPPORTING

The entire gyro path is `gyro_accum / batch.count`. #14 covers what that costs in aliasing; the
other half is that **no software filter exists at all** -- no notch, no lowpass, no RPM tracking,
and no biquad anywhere in the repo. Betaflight runs five stages before its PID sees a sample.

Which of those stages earn their place is not obvious, and two of them do not:

- **Static notches default to off** (`gyro_notch1_hz = 0`), and have since the 3.4 defaults pass
  that introduced the dynamic notch. A fixed notch fights a peak that sweeps ~80-800 Hz with
  throttle, paying its phase cost for the whole flight to intersect the noise for a fraction of
  it. They survive for fixed frame resonances, found by hand from a log.
- **The RPM filter is the one that works**, and it is the one we are best placed to build: it
  needs per-motor frequency, and `EscTelemetryMotorData::rpm` is already on the Blackboard.
  Four motors x three harmonics x three axes is 36 biquads; coefficient updates need sin/cos,
  which is why Betaflight uses polynomial approximations and staggers one motor per loop.
- **The dynamic notch** tracks peaks with a sliding DFT when no RPM reference exists. A real
  project, and largely redundant once the RPM filter runs.

The filter itself is trivial -- one biquad struct, five multiplies and four adds per apply,
roughly 0.4% CPU for two notches on three axes at 1024 Hz.

**Where the burst lets us beat the reference.** Betaflight filters one sample per interrupt and
runs its notches *after* decimation. Our burst gives the same per-sample rate at 1/8 the
interrupt cost and lets the notches run **before** the decimation -- which matters here and not
there, because our Nyquist is 512 Hz and the 2nd and 3rd motor harmonics (~530 and ~800 Hz at
full throttle) fold into the loop band before any post-decimation filter can see them. That is
#14's aliasing attacked from the other side, and it is the reason to do both together.

Wants #24 for an RPM source worth tracking.

### #24 — Bidirectional DShot — 🟢 SUPPORTING

#23's RPM filter needs motor frequency every loop, and the KISS serial telemetry we poll cannot
give it: one motor at a time, tens of Hz, so a notch chasing a throttle punch would sit on
50 ms-stale RPM. Betaflight feeds its filter from bidirectional DShot instead -- the ESC answers
each frame on the same wire.

Bigger than it sounds, but it does **not** cost the burst-DMA design. The trap to avoid is
Betaflight's timer-capture path: latching edge timestamps into CCR1-4 needs four edge-triggered
DMA streams, because four ESCs reply on their own schedules and a DMAR burst has exactly one
trigger -- which is why their code makes burst and telemetry mutually exclusive. That is a
constraint of *capture*, not of receiving. Their own default (bit-bang) receives the way we
transmit: one DMA stream on a fixed cadence -- pointed at `GPIOE->IDR`, where one 16-bit read
carries all four pins' levels at once. Cadence-sampling levels scales to four asynchronous
repliers; edge-latching timestamps never can.

So the transaction becomes: DMAR burst out of CCR1-4 (unchanged) -> one MODER write flips
PE9/11/13/14 to input -> the same stream retargeted at IDR samples at ~3x the GCR rate
(~130 samples over the ~56 us reply) -> flip back. What it costs:

- **The signal inverts.** Bidir DShot idles high, ours idles low, so the output stage changes.
- **The turnaround window is real but bounded**: the ESC replies ~30 us after the frame ends,
  and the whole TX + gap + RX transaction is ~140 us of a 976 us loop at DSHOT300.
- **Undoing GCR is ~400 lines of software** (`dshot_bitbang_decode.c`): edge-find over the
  sample buffer, 21 bits at 5/4 the DShot rate, transition-decoded, 5-to-4 GCR lookup, CRC.

One thing falls our way: port sampling needs every motor on one GPIO port, and ours are PE9,
PE11, PE13 and PE14 -- all GPIOE. AM32 supports the protocol (input type 4, EDT) and
`EscTelemetry::Info::bidirectional` already decodes the flag.

#### Where each piece lives

The IMU path already answers this, layer for layer:

- **Sampler and phase machine -> `DShotTim1`.** The turnaround is the same transaction as
  transmit -- same timer, same stream, same pins -- and a phase machine split across files
  would give one DMA stream two owners.
- **GCR decode -> a pure function beside `DShotCodec`**, encode's mirror: samples in, eRPM
  out, no hardware. Pure is what lets the SIL test 400 lines of bit-twiddling off-target with
  canned buffers.
- **Publish -> the driver, from the RX-complete ISR**, exactly as `Icm42688p` parses and
  publishes `ImuHealth` from its DMA-done ISR. Into a **new** Blackboard struct --
  `MotorRpmData { timestamp_us, erpm[4], valid_mask, crc_error_count }` -- not into
  `EscTelemetryData::rpm`, which has one writer (#19) and keeps it. KISS telemetry stays the
  volts/amps/temp source and its slow rpm becomes the cross-check on the fast one.
- **Consumers read the Blackboard, nothing holds a `DShotTim1 *`.** The RPM filter (#23) in
  PendSV, the desync detector (#12) in Sentinel, `stat_publisher` if the wire wants it. The
  struct's timestamp is load-bearing from day one: it is what lets the filter fade a stale
  notch the way Betaflight's `rpm_filter_fade_range_hz` does.

#### Two files, split by purity rather than direction

A `dshot_tx` / `dshot_rx` pair is the one arrangement to avoid. They are not two things: the
turnaround is a single transaction over one timer, one DMA stream retargeted mid-flight, one
set of pins and one phase machine. Separate files give that stream two owners, or have one file
reach into the other's state. A third coordinator file is worse -- it exists only to sequence
halves that were never separable.

- **`dshot_tim1.*` keeps the whole transaction**: burst out, MODER flip, IDR sampling, flip
  back, phase machine. 304 lines today, perhaps 500 after. Meaning-free throughout.
- **The GCR decoder gets its own file, not a half of `DShotCodec`.** ~400 lines onto a 133-line
  codec would leave the codec 80% decoder, and the two are not an encode/decode pair anyway:
  outbound is 16 bits (11 throttle + 1 telemetry request + 4 CRC), inbound is 21 bits of
  transition-encoded GCR carrying a 12-bit period, a 4-bit exponent and a CRC. Two protocols
  sharing a wire.

The decoder should be genuinely pure -- `<array>`, `<cstdint>`, no hardware header -- which
puts it in `libs/` beside `message.hpp` rather than under `stm32/`, where the SIL reaches it
without dragging in STM32 headers to feed it canned sample buffers.

Worth noting on the way past: `DShotCodec` is *not* pure today. `dshot_codec.cpp` includes
`dshot_tim1.hpp`, so the codec already reaches into the driver. That coupling is worth undoing
whether or not bidir ever lands.

It pays for itself twice. Besides #23, per-motor eRPM every loop is the desync detector #12 is
looking for.

It also caps the loop rate: a reply roughly doubles DSHOT300's frame budget, which is why
Betaflight forces `pid_process_denom >= 2` on F4. This and any `STM32_CONTROL_LOOP_HZ` bump
constrain each other and want deciding together.

### #25 — Calibration is stored, never applied, and would reset the board — 🎯 CRITICAL

Two unrelated halves, both armed the moment #16 wires a MAVLink trigger.

**Accel calibration does nothing.** `accel_calibration_` is loaded from EE at `Init` and written
back by `SaveAccelCalibration`, and `ScaleSample` never reads it. It round-trips through storage
and never reaches a sample.

**Gyro calibration would trip the watchdog.** `CalibrateGyro` collects for `gyro_duration_s` of
2 with a `gyro_timeout_s` ceiling of 5, and there is no `Watchdog::Kick` anywhere in that
driver. The IWDG's worst-case period is ~681 ms, so the call resets the board three times over
-- seven at the timeout. It has no caller today, which is the only reason it has never fired.

The gyro half is incomplete beyond that: `ClearUserOffsets()` wipes the chip's offsets every
boot, there is no EE record for them the way there is for accel, and #21 explains why the
trigger has to check the reset cause as well.

### #26 — Blackbox logging — 🟢 SUPPORTING

An SD driver and a service. The Blackboard is now the right shape to log from: every fact in one
place, each timestamped, each with one writer, and change detection held per-consumer rather
than shared -- which is why `PopGps`/`PopRc` had to go, since a single `updated` bit would let a
logger and the telemetry publisher clear it out from under each other.

The fast path is the constraint. `ImuState` at 1024 Hz is ~56 bytes a tick, about 57 KB/s --
fine for the card, not something the slow loop should marshal synchronously. It wants its own
buffer and a bulk flush.

### #27 — An estimator tier below the control loop — 🧊 DEFERRED

Autonomy needs a state estimate the rate loop does not: position, velocity, and an attitude that
stays consistent through aggressive manoeuvring. An invariant EKF is the interesting choice --
its error dynamics are trajectory-independent, which is exactly the regime a fast autonomous
craft lives in, and neither PX4 nor ArduPilot ships one.

The architecture is half-built already. The control loop iterates the burst once and produces two
things at different rates: filtered gyro for control (#23), and delta-angle / delta-velocity
increments accumulated per sample for the estimator, the way PX4's `ImuDownSampler` does with
coning and sculling corrections. Those increments are the **one** legitimate case for IMU data
on the Blackboard -- about 28 bytes, rate-decoupled by construction, and safe there precisely
because an accumulator tolerates a missed read where a stateful filter does not.

Two things are not the filter's algebra, and are where the time actually goes: the delayed-time
fusion shell that lets 100 ms-old GPS fuse correctly, and the innovation gating that decides
when a sensor is lying. EKF2's real value is that shell, not its equations.

One rule if it lands: **attitude gets a single owner.** Mahony and an IEKF both estimate it, and
both writing `ImuState::attitude_world_to_body` is the duplication this repo has spent three
days removing. PX4 retires the complementary filter into an output predictor; ArduPilot keeps
DCM as an explicit fallback lane. Either is fine; two writers is not.

### #28 — The vehicle cannot be armed without the ESP32 — 🎯 CRITICAL

`CommandHandler::OnPrivilegedArm` is the only caller of `Sentinel::RequestArm` on the board, and
it is reachable only by `kPrivilegedArm` over FcLink. So arming *and disarming* both require the
second MCU alive and a GCS talking to it. #16 records the consequence: an ESP32 failure while
armed removes the disarm path entirely and leaves the cascade running on the last RC values.

An arm switch is the fix, and Sentinel is already shaped for it -- `RequestArm` is the single
entry point, so this is a second caller, not a second mechanism.

#### The switch is the easy part

`RcReceiver::Config` carries roll, pitch, yaw and throttle and nothing else, so an aux channel
needs a Kconfig knob, a calibration schema slot, and a wider `RcMapConfigMsg` -- a wire change,
so both firmwares. That is plumbing. The interlocks are the work:

- **Throttle low**, or the props spin the instant the switch flips.
- **The switch must have been seen *low* once since boot.** Otherwise powering on with it
  already high arms on the bench, which is how people lose fingers.
- **RC fresh**, plus #15's `_manual_control_lost_at_arming` rule so arming over FcLink with no
  transmitter powered on does not disarm instantly.
- **A refusal reason.** Betaflight carries roughly twenty `armingDisableFlags` because "it will
  not arm and will not say why" is its own failure mode. `RequestArm` returns a bare `bool`
  today; the caller turns it into a tone. With one arming path that is survivable, with two it
  is not.

#### Ordering

#15 first, and its tripwire says why: while `kPrivilegedArm` is the only route, holding stale
sticks is harmless because nothing can arm behind them. A stick gesture removes that guarantee,
so link-loss policy has to exist before this lands, not after.

### #29 — `Services/` is four different things — 🟢 SUPPORTING

The directory holds sixteen classes and the `Poll()` split separates them cleanly. Everything
with one owns a peripheral and runs on the slow tick. The four without it -- `Ahrs::Process`,
`AttitudeController::Step`, `RateController::Step`, `MultirotorMixer::Mix` -- are called per IMU
burst, hold no buffer and no `SharedState *`, and cannot fail asynchronously. They are a control
tier wearing a services label.

Worth separating because it turns a convention into a check. **Nothing in the fast path does
I/O** is currently something you have to remember; as its own directory it is a lint rule -- no
`Poll`, no driver includes, no `System::GetInstance()`.

Moving only those four would make the directory worse, though, because three more are already
misfiled. `DShotCodec` owns TIM1 and its DMA and `Write()` puts bits on the wire -- a driver.
`EscBootloader` owns a `UartSoft` and speaks BLB -- also a driver. `config_storage` is six
static functions over `EE` with no instance and no state -- a free-function library. Leaving
those behind means `Services/` would mean "polling services, plus the leftovers" rather than
anything. `CommandHandler` is the one genuine judgement call: `Dispatch` is reactive rather than
periodic, but it acts on the same tier as the services and belongs with them.

Pure file moves, zero behaviour change, so the cost is entirely in include paths and
`stm32/CMakeLists.txt`. That is also the argument for timing: it churns every diff it touches,
so it lands as one commit on a clean index, after the current review sweep clears -- not
interleaved with work that has to be read line by line.

### #30 — Name the IMU orientations rather than configuring a triple — 🟢 SUPPORTING

`axis_map` is hardcoded identity in `stm32_config.hpp.j2`, with a comment saying to promote it
to Kconfig when a board rotates the chip. Promote it as a **named choice**, not as the signed
permutation the struct stores.

A signed permutation has 6 orderings x 8 sign combinations = 48 settings, and only 24 are
rotations. The other 24 have determinant -1: reflections no rigid mount can produce.
`ValidateConfig` does not catch them -- it checks the ordering is a permutation of {0,1,2} and
never looks at the signs -- so `x_from=1, y_from=0, z_from=2` all-positive passes, swaps X and
Y, and hands the AHRS a left-handed frame.

Betaflight's set is the right size: 8 orientations, 4 yaw x {upright, flipped}
(`common/sensor_alignment.h`), plus a sentinel for the driver default and one custom escape.
It covers every mount anyone builds on a board they designed, stays a signed permutation so
`ScaleSample` remains a table lookup, and structurally cannot express a reflection. PX4 carries
41 rotations behind an Euler table and a DCM multiply, including 45-degree steps and one
`ROTATION_ROLL_90_PITCH_68_YAW_293`, because it runs on airframes somebody else laid out. That
generality is a liability here, not a feature.

Shape: a Kconfig choice, expanded by the generator into the permutation and signs the driver
already consumes, with a determinant check in the generator so a bad expansion fails the build
instead of the flight. Nothing in the fast path changes.

**One rule survives regardless: `CalibrateGyro` stays on raw counts.** `OFFSET_USER` is
per chip axis, so a body-frame bias has to be run back through the inverse map before it is
written -- and that write is permanent, silent when wrong, and shows up as drift on an axis
that was never calibrated. Keeping calibration in chip frame means the map never enters that
path and the inversion cannot be forgotten. It is also the second reason the sample batch
should stay raw, alongside #25 and #26.

Pairs with #25: a rotated mount invalidates the stored accel calibration as well, so the two
land together or not at all.
