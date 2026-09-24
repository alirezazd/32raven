# 32Raven — Roadmap

The single source of truth for **32Raven firmware and handbook work**. Every `TBD(#N)` marker
in the handbook points to an item here, and `scripts/lint/check_docs.py` fails the build if it
points at an item that does not exist — so a placeholder cannot be quietly forgotten.

**Scope — this repository.** Firmware and handbook work only.

Delivered items are deleted rather than marked done; git history is the record. Numbers are
never reused, so gaps are expected.

**Priority legend**

- 🎯 **CRITICAL** — blocks the prototype build, blocks the handbook being usable by someone
  who is not the author, or leaves the firmware in a shape that other queued work has to
  build around.
- 🟢 **SUPPORTING** — real work; do it when the area is being touched anyway.
- 🧊 **DEFERRED** — real work, parked deliberately.

---

## Handbook

Written alongside the first from-scratch prototype assembly; these are the gaps that assembly
closes.

### #1 — Prototype photograph — 🎯 CRITICAL

`docs/assets/prototype-placeholder.svg` stands in for a photograph of the assembled aircraft:
plain background, props off, ~1600 px wide as WebP or JPEG under `docs/assets/`, referenced
from `docs/index.md`.

### #2 — Cost and build time — 🎯 CRITICAL

The **Parts cost** and **Build time** cards on the index are `TBD(#2)`. From the actual build:
total parts cost with the currency and the month priced; assembly time and bring-up time as
separate figures.

### #3 — Airframe specifications — 🎯 CRITICAL

The **Airframe** card is `TBD(#3)`, and **Power** lacks capacity and all-up weight. Frame class
and prop size, motor and ESC, battery capacity, all-up weight and measured flight time — the
last two from the built aircraft, since a reader sizes their own pack from them.

### #4 — The remaining stubbed build stages — 🎯 CRITICAL

Still to write: frame and motors, power, smoke test, RC link, bench test, first flight — each
as its stage is reached during the real build. The wiring page waits on two photographs of real
harnesses.

### #6 — Buzzer — 🟢 SUPPORTING

A `TBD(#6)` stub on **Sensors and peripherals**: the part and whether it is active or passive,
its wiring to `GPIO10`, where it mounts, and what the firmware sounds it for.

---

## Safety and failsafe

### #15 — The failsafe conditions besides RC loss — 🎯 CRITICAL

RC loss is the only condition Sentinel answers.

- **GCS connection as a fact on the STM32.** RC loss is not fatal while the pilot holds the
  telemetry link — with a 900 MHz radio that is the expected case — and PX4 keeps the two as
  independent conditions. `HEARTBEAT` reaches the bridge and only blinks the LED; nothing
  stamps it or forwards it over FcLink. FcLink peer loss is the third condition: #16's on the
  STM32, #17's on the bridge.
- **Three flags never set.** `kVehicleFailsafeFlagBattery` — the arming interlock reads the
  resting voltage; the in-flight condition sags with throttle, and what it should do is #50's
  landing rather than a cut. `kVehicleFailsafeFlagImu` reaches the ground and nothing acts on
  it. `kVehicleFailsafeFlagGps` until something navigates by GPS.
- **The policy**: which conditions count and how fast. The ranked resolution they feed is
  #50's; where it lives is #16.

### #16 — Sentinel, one owner for arming and failsafe authority — 🎯 CRITICAL

Sentinel owns the arm decision, the IMU conditions and the interlocks. Still outside it:

- **Condition detection lives in a telemetry builder.** `TelemetryPublisher::BuildSystemStatusMsg`
  derives GPS, battery and RC health from freshness; `Sentinel::Supervise` is where that
  computation moves. It stays on the STM32 and first in `System::Poll`: a failsafe skipped
  under load does not exist.
- **FcLink peer loss.** `FcLinkData::timestamp_us` is stamped on every good frame and nothing
  evaluates it. A bridge failure while armed takes every annunciation the pilot has.
- **The watchdog.** `Wdg().Kick()` is unconditional in the superloop, so a wedged control loop
  under a healthy superloop is fed — over a thousand missed control cycles inside the ~681 ms
  IWDG window. Kick only once `ControlLoopLoad::timestamp_us` has advanced, and mode-aware:
  `EscBootloader`, `Sdio`'s blocking waits and `LogService::DrainFlush` kick from outside the
  superloop for reasons of their own. WWDG is unused: a window watchdog off PCLK1 with a ~50 ms
  ceiling catches a loop running wrong-fast, which IWDG cannot — the control loop's, while IWDG
  keeps the superloop.

### #17 — Doctor, health observation on the ESP32 — 🟢 SUPPORTING

The counterpart to #16, split by authority: Sentinel decides, Doctor observes and reports and
must be harmless when wrong, which is what lets it live across a link that can die. It adds what
only the bridge sees — heap, task stack headroom at run time, WiFi, flash, transport stats — and
correlates across FcLink; a FreeRTOS task with a real priority. It reports to MAVLink and the
logs, never the OLED.

Before the service exists: nothing tells the GCS *why* the peer went quiet (#41 is the usual
reason).

**The card that left.** `LogService` panics at boot on a missing card and is near silent about
one pulled afterwards; the first notice is a failed write tens of milliseconds into the next
flight. No card-detect line and no `Sdio` ISR, so presence is a CMD13 over the bus — asked at
`StartFlight`, the one moment the answer changes a decision — and reported through the logger
counters `TelemetryPublisher` already carries. Doctor turns "mandatory at boot, absent now"
into something a GCS sees.

### #20 — Sentinel watchdogs on the IMU and the control path — 🎯 CRITICAL

- **Every threshold is asserted, not measured.** `imu_loss_threshold_samples` (5‰ of gyro
  ODR, 3 windows), `imu_fault_threshold` (3 in 1 s), `imu_stall_timeout_us` (20 ms). The loss
  rule measures something real — `PENDSVSET` is a bit, so two bursts in one tick collapse and
  the older drops — and the loop went to 2048 Hz without that measurement. The `imu_health`
  record already carries the counters: read `missed_samples` across disarmed idle, armed at
  idle, a throttle sweep, and every link streaming.
- **The stall recovery cannot clear.** A DMA that started and never completed leaves
  `inflight_` and the SPI driver's `busy_` set through `RestartSampling`. Aborting a live DMA
  belongs here, not in the driver.

### #50 — The four failsafe conditions, once there is somewhere to go — 🧊 DEFERRED

What #15's conditions become once autoland and return-to-home exist. Gated on #46 (altitude),
#27 (position and velocity), #45 (heading) and #49 (a fix worth trusting as a flight input).

- **RC loss** → return. Recovery needs sustained clean frames plus a deliberate pilot action,
  so a flapping link cannot toggle the aircraft between returning and manual.
- **FcLink loss** → a failsafe only *together with* RC loss; a live transmitter and a dead
  bridge is a pilot with full authority.
- **GPS loss** → RTH unavailable if lost before, degrade mid-manoeuvre if lost during. #49's
  integrity messages become flight inputs here: a spoofed fix reports excellent `hAcc`.
- **Low battery** → the one continuous condition and the one that preempts: a return threshold
  as a function of distance-to-home, a lower one that lands where it is. Without current sense
  the estimate is voltage under load, worst while climbing.

One ranked resolution evaluated every pass, not four handlers:
`enum class FailsafeAction : uint8_t { kNone, kWarn, kReturn, kLand, kDisarm };` — each
condition reports what it wants, most severe wins, and the result latches downward, since a
battery that recovers when the throttle drops must not talk a landing back into a return.

Needs a mode Sentinel can command rather than fly — the cascade flies (#52) — and a home
position captured at arm as a pre-arm check, with the never-seen rule
`_manual_control_lost_at_arming` already applies to RC.

### #52 — The vehicle states a failsafe procedure needs — 🧊 DEFERRED

Four states, two of which fly; enough while every procedure is "disarm". A descent or a return
sources setpoints from something other than the pilot, for a bounded time, with its own exit
rules — sequencing, which must not run inside the safety authority.

- **`Failsafe`** — the parent, entered on a condition Sentinel raises while armed, exited when
  the pilot takes the aircraft back; #50 maps condition to procedure.
- **`ReturnHome`** — setpoints from a navigator against the home vector (#27, #45).
- **`Landing`** — closed-loop on altitude (#46); a fixed throttle and a timer is not one.

To settle: the request as a blackboard field the machine reads, the way `armed_` works, never
Sentinel calling `ReqTransition`; the setpoint source switchable at one point, where
`ControlTickFlightLoop` reads `RcData` directly today; takeback needing clean frames plus a
stick action, of which `RcLinkPhase::kRecovering` is the first half; and every state here
implementing `IControlTickState`, so `kArmBlockNotStandby` is re-derived once more than one
flying state is disarmed-and-armable.

---

## Motors and ESCs

### #10 — More than four motors — 🧊 DEFERRED

TIM1 has four compare channels; six motors needs TIM8 and a second DMA stream with both bursts
started together, or the skew reads as a yaw bias. `stm32_limits::kDshotChannelCount` is the
ceiling every consumer asserts against; fewer than four is idle channels. No non-quad airframe
is planned.

### #11 — Warn the pilot when an ESC starts derating — 🎯 CRITICAL

AM32's thermal and current limits sit at settings bytes 43 and 44, in the layout-stable window,
and the flight controller reads the matching telemetry without knowing either threshold — a
derate arrives as thrust loss the rate controller winds I-term against. Read the two into
`EscTelemetry::Info`, detect the derate, and tell the pilot: the CRSF flight-mode text, and
temperature as a sensor (0x0D) so the handset's logical switches can voice it. Not a panic — a
degraded aircraft the pilot may still want to land.

### #12 — Decide what a desync looks like from the flight controller — 🎯 CRITICAL

`stall_protection` (byte 29) and `stuck_rotor_protection` (byte 22) decide whether the ESC cuts,
retries or keeps driving on lost sync, each presenting differently in telemetry, and the FC
interprets none of it. Settle the policy per combination first — a mixer compensating for a
motor about to restart makes the recovery worse — then whether the firmware check constrains
the two bytes as it does input type and direction. #24's per-loop eRPM is the detector's input.

### #13 — Signed thrust, so 3D mode means something — 🧊 DEFERRED

The thrust chain is unsigned end to end: `MultirotorMixer::Mix` clamps to `[idle, 1]`,
`ThrustToDshot` maps to stop-or-forward. Needs signed `[-1, 1]` through the mixer and both
controllers, a three-way `ThrustToDshot` over the 3D split (48–1047 reverse, 1048–2047 forward),
`k3dModeOn` actually issued, a configurator path, a settings re-read, and `kEscDirectionReversed`
revisited. No planned mode asks a motor for lift the other way.

### #24 — Bidirectional DShot — 🟢 SUPPORTING

Per-loop eRPM for #23's RPM filter and #12's desync detector; KISS polling is one motor at a
time at tens of hertz.

The design that keeps the burst-DMA transmit: after the DMAR burst, one MODER write flips
PE9/11/13/14 to input, the same stream retargeted at `GPIOE->IDR` samples all four pins at ~3×
the GCR rate, flip back. Port sampling scales to four asynchronous repliers where Betaflight's
timer-capture path cannot, and all four motors are on GPIOE. AM32 speaks it (input type 4, EDT).

- The signal inverts: bidir idles high.
- ~92 µs per transaction at DSHOT600 against a 488 µs tick, 19 %; any further
  `STM32_CONTROL_LOOP_HZ` increase and this decide each other.
- GCR decode is ~400 lines: edge-find, 21 bits at 5/4 rate, 5-to-4 lookup, CRC.

Placement: the whole transaction stays in `dshot_tim1.*` — one stream, one owner, no
`dshot_tx`/`dshot_rx` pair. The decoder is a pure function in `libs/`, testable off-target,
and not a half of `DShotCodec`: two protocols share the wire. The driver publishes from the
RX-complete ISR into a new `MotorRpmData { timestamp_us, erpm[4], valid_mask, crc_error_count }`,
never into `EscTelemetryMotorData::rpm`, which keeps its one writer; consumers read the
blackboard. On the way past: `dshot_codec.cpp` includes `dshot_tim1.hpp`, so the codec already
reaches into the driver — undo that regardless.

---

## Sensors and estimation

### #23 — Nothing filters the gyro in software — 🟢 SUPPORTING

The gyro path is `gyro_accum / burst.count`: no notch, no lowpass before the PID, and the chip's
hardware notch is configured off. Of Betaflight's stages, the RPM notch is the one that earns
its place — static notches default off there too, and the dynamic notch is redundant once RPM
runs: four motors × three harmonics × three axes of biquads, coefficients by polynomial sin/cos,
staggered one motor per loop. Run it on the 8192 Hz pre-decimation stream, where the 2nd and
3rd harmonics (~1600 and ~2400 Hz at full throttle) are still real rather than folded into the
1024 Hz loop band. Wants #24 for an RPM source worth tracking.

### #25 — Calibration without a ground station — 🟢 SUPPORTING

`MAV_CMD_PREFLIGHT_CALIBRATION` is the only way in, so every calibration means a laptop. The
bridge carries a display, a buzzer and a button — everything a pose routine needs — and the
wire already carries what a page would draw (`kCalStatus`: state, sides, progress). A
calibration page on the bridge runs the same routines over FcLink with no GCS.

### #27 — An estimator tier below the control loop — 🧊 DEFERRED

Position, velocity and an attitude consistent through aggressive manoeuvring. An invariant EKF
is the interesting choice — trajectory-independent error dynamics, and neither reference stack
ships one. The inputs are half-built: the control loop already produces filtered gyro for
control and can accumulate delta-angle / delta-velocity per sample the way PX4's `ImuDownSampler`
does, rate-decoupled by construction. The time goes into the delayed-time fusion shell (100 ms-old
GPS) and innovation gating, not the algebra.

- **Attitude gets a single owner.** Mahony and an IEKF both writing
  `EstimatorState::attitude_world_to_body` is two writers; PX4 retires the complementary filter
  into an output predictor, ArduPilot keeps DCM as a fallback lane — either, not both.
- **The compass hands over** (#45): the airborne interference check — compass heading against
  gyro-integrated yaw, DJI's "yaw error", which the strength check cannot see — and PX4's answer
  when it fires: stop fusing, keep flying, never a mode the pilot did not choose.
- **GPS becomes a flight input** (#48, #49): `sAcc` and the NED velocities are what it consumes.

### #45 — The compass reads, and nothing trusts it yet — 🟢 SUPPORTING

- **The iron the motors add is not corrected.** The calibration is taken with the motors off;
  four ESCs switching tens of amps centimetres away add a field it never saw. PX4's
  `CAL_MAG_COMP_TYP`: correlate the field against battery current (already read) or throttle,
  subtract the fitted share live. Needs a flight log carrying both before the term is worth
  writing.
- **Interference is caught by strength alone**; a field turned but not stretched passes. The
  yaw-consistency half, and the estimator consuming the compass at all, are #27's.

### #46 — Barometer, DPS310 — 🟢 SUPPORTING

No altitude source, so nothing can hold altitude or descend under control (#50, #52). DPS310 on
the I2C1 bus already built: a driver and a blackboard fact, with the part's
temperature-compensation coefficients read at boot — skipping them yields plausible nonsense.
Its zero is a ground reference re-established at arm, estimator-side, not a `SensorCalService`
record. Land two CRSF frames with it: `0x09` BARO_ALTITUDE (the referenced altitude, not raw
pressure) and `0x07` VARIO (a derivative — from the estimator, #27, or it is noise).

### #48 — Decide whether GPS quality gates arming — 🟢 SUPPORTING

`hDOP` is plumbed end to end and compared to nothing; Sentinel's arm path takes no view of GPS.
Warn, do not refuse, while nothing navigates — a quad that will not arm indoors is broken for
the bench; the moment #27 consumes position, a degraded fix at arm is a flyaway. `num_sats` and
`fix_type` are the coarse conditions; DOP says degraded rather than absent; #49's integrity
messages say whether the signal is real. The threshold comes from numbers off the card (#33,
#49), not the datasheet. A Sentinel condition, not a check in the GPS driver.

### #49 — Match PX4's UBX message set, and use it to decide the fix is trustworthy — 🟢 SUPPORTING

| Message | PX4 (`u_blox10`) | 32Raven |
| --- | --- | --- |
| NAV-PVT | yes | yes |
| NAV-DOP | yes | **no** |
| NAV-STATUS | yes | no |
| MON-RF | yes | no |
| SEC-SIG | yes, non-fatal on NAK | no |
| RXM-COR | yes | no |
| NAV-SAT | on request only | no |
| NAV-COV | no | yes |
| NAV-EOE | no | yes |

- **NAV-DOP is off** and four DOP fields are plumbed anyway: MAVLink sends `UINT16_MAX` for the
  zero, the log writes `0.00` — a perfect fix in any viewer. One config line.
- **NAV-COV goes.** No reference estimator consumes a covariance; EKF2 builds R from `hAcc`,
  `sAcc` and its own floors. `M10PVTData` already parses `sAcc`, `headAcc` and `velN/E/D` and
  `GpsData` drops them: add those plus `hAcc`/`vAcc`, drop `posCov*`. NAV-EOE exists only for
  the multi-message join, so it goes when the epoch is NAV-PVT alone — the two are one change.
- **The integrity half is the point.** NAV-STATUS, MON-RF and SEC-SIG carry jamming, spoofing,
  AGC and noise — whether the fix can be *believed*, which accuracy fields cannot say, and what
  #48 and #50 need. PVT+DOP+EOE is 1380 B/s of 11.5 kB/s, leaving room at a divided rate as PX4
  does with NAV-SAT.
- PX4 also writes zero to NAV-TIMEGPS and RXM-SFRBX in case another firmware left them on.

Independent of everything; #33 records the new fields once they exist.

### #51 — Per-cell voltage, sensed rather than divided — 🟢 SUPPORTING

`Battery::EstimatePercentage` divides pack voltage by the cell count and maps it linearly: blind
to one cell collapsing under five healthy ones, wrong in the flat middle of the curve and under
load. Sensing it is a balance-lead tap into an ADC network — a board change — which is why the
estimate stands meanwhile. It unblocks CRSF `0x0E` CELLS (unsent until the reading is real), a
failsafe on imbalance rather than pack voltage (#50), and a state of charge combined with
`EscTelemetryData::consumption_mah`.

---

## Links and telemetry

### #9 — Share the FcLink frame parser — 🧊 DEFERRED

The byte-at-a-time receive machine exists twice, and CRC verification is implemented differently
on each side — a contiguous rebuild on the STM32, byte-fed `XModemUpdate` on the bridge —
agreeing only while `message::Header` stays exactly `{magic[2], id, len}`, and failing silently
otherwise. A policy-free parser in `libs/`: byte in, verdict out — need-more, bad magic, bad
length, bad CRC, complete — with each side keeping its own reaction, ring buffers and read
budgets. Deferred because it rewrites the receive hot path on both firmwares.

### #41 — The radio goes dark on every bench page — 🎯 CRITICAL

The Telem UART is the aircraft's MAVLink link; WiFi and USB are bench transports. Four states
call `Mavlink().SetTelemetryLink(false)` in `OnEnter` and one has a reason:

| State | STM32 | FcLink | Telem UART | Wanted |
| --- | --- | --- | --- | --- |
| Service, waiting for a host | Standby, running | free | free | on |
| WifiLog, waiting for a host | Standby, running | free | free | on |
| EscConfig | suspended | MSP relay | MAVLink, full | not-ready |
| UsbLog (MSC) | suspended | grant only | free | on, not-ready |
| LogPull, transferring | Standby, running | saturated | free | narrowed |
| Program, flashing | ROM bootloader | held by Programmer | free | dark |

Recovery Service mode is the seventh case (#40). A dark link and a dead board are the same
thing from the ground.

- **A page picks a profile, not the radio**: full, heartbeat-plus-status, or nothing, with only
  Program picking nothing. `LogPull` saturates FcLink's TX budget while the Telem UART sits idle.
- `SetTransport` has the same shape — the four states inherit whatever the last page left — and
  `Mavlink().Poll()` runs only from the four streaming states.
- **`boot_state` is a readiness state.** The bench pages report `kBooting` when the truth is
  "a configurator holds the motor lines": `MAV_STATE_CALIBRATING`, which PX4 sends for
  `in_esc_calibration_mode`, via a `kNotReady` enumerator — a wire change, travelling with #42.
  EMERGENCY ("lost control over parts") is the other unreachable `MAV_STATE` worth closing; #15
  decides it.

### #42 — SystemStatus reports values nothing produces — 🟢 SUPPORTING

`error_code` is always `kOk` while `Sentinel::imu_fault_latched_` holds a real fault an aircraft
is flying on; `errors_count1..4` are four literal zeros where four `ImuHealth` counters fit. A
wire change — same flash as #21's reset cause and #41's `kNotReady`. Reading any of it needs #41.

### #43 — A current reading is trusted as far as it can saturate an int — 🟢 SUPPORTING

Blocked on a real sensor reaching `PC1`. Then: `STM32_BATTERY_CAPACITY_MAH` and a max-current
knob, a plausibility bound in the driver beside the clamp and deadband (above what the pack can
deliver is provably false), and an over-current condition feeding #15's battery flag. The bound
cannot detect an absent sensor — a floating pin reads amps that fit any budget — which is what
the build-time knob is for.

### #55 — A byte count is summed into the ESC fault total — 🟢 SUPPORTING

`EscTelemetryData::Total()` adds `rx_drop_bytes` to four event counters, unlike every other
fault struct, so a windowed count reads one drain as however many bytes were in flight — and
`PublishIfChanged` stamps the bus heartbeat on a dropped byte. Count the overflowing drain as one
event in `Total()`, keep `rx_drop_bytes` as the magnitude, add the field to the ULog
`system_health` schema. A wire change for #42's flash.

### #53 — The LR900-P replaces WiFi as the MAVLink link — 🧊 DEFERRED

The radio goes on the Telem UART, and landing it is setting `ESP32_MAVLINK_TX_LINK_AIR_RATE` to
the mode it is configured in and reading what the build says: at 2.1 KB/s the default ladder
(~1.57 KB/s, of which RC_CHANNELS at 40 ms is 1.35) fits with 7 % of margin; at 1.1 or 0.4 it
does not, and RC_CHANNELS is the first knob. Open with the radio: the narrower FHSS uplink
carrying commands and parameters, which the check does not model; #41's profiles as budgets;
whether the ladder should stretch at runtime like CRSF's — probably not, an air rate does not
change under the config.

---

## Operator interface

### #36 — Notifications the display can carry — 🟢 SUPPORTING

Several conditions are a warning tone plus an `ESP_LOGW` nobody sees: the ESC port refused while
armed, no session granted, the TX queue dropping. A notification is a message drawn over the
current screen with the state machine untouched — unlike `kHardError`. The pieces exist
(`Ui::LoadWidget`, `IWidget`, `NotifyUserActivity`). To decide: dismissal is a hold, which must
consume the press rather than swap menus (`CycleOnButton`, one place); a queue with depth, drop
policy and dedupe; expiry per notification, not globally; a severity on `kLog` packets so the
STM32's lines qualify without a second wire format. First users: the sites that play
`kWarning`, and #11's derating warning.

### #39 — The LED says less than it has states — 🟢 SUPPORTING

Six pages, three patterns: Serving breathes, Service blinks 400 ms, EscConfig/UsbLog/WifiLog all
blink 800 ms, the two MAVLink pages set nothing and show whatever was left. A one-shot
(`SetPattern(..., repeat_count)`) ends in `kOffStep` rather than restoring the page's pattern,
so `mavlink_rx.cpp`'s per-heartbeat `kDoubleBlink` would silently end any page pattern — the
mechanism wants a foreground/background split. The STM32 LED is `Set`/`Toggle`, a pin not a
signal, so an arm refused with the bridge dead has no tone, no display and no LED. Decide what
the LED means — page identity or link liveness on the bridge, armed on the STM32 — before adding
patterns.

---

## Logging

### #26 — Blackbox retrieval — 🟢 SUPPORTING

Neither retrieval path has been exercised: the USB Log page mounting the card over MSC, and
`tools/pull_logs.py` returning a byte-identical copy over WiFi.

### #31 — Tuning-grade log content — 🟢 SUPPORTING

The log records what the vehicle did, not what the controller asked for: rate setpoints, torque
command and per-motor thrust are locals through rate_controller → mixer → esc_service. A
`ControlOutputs` POD on the blackboard written by the control tick (~36 B at 2048 Hz, ~80 KB/s),
a decimation knob once flights show whether full rate pays, and a firmware identity in the ULog
`I` messages once the STM32 has one (#18).

### #32 — Format the card on the vehicle — 🟢 SUPPORTING

An unmountable card is a trip to a PC; one over 32 GB a trip to a PC with third-party tooling.
`f_mkfs` with `FM_FAT32` costs +2,088 B of flash and no RAM (`LogService::staging_[0]` is idle
whenever a format could run). Never automatic — a mount failure is also a transient SDIO error,
or a corrupted directory over intact log sectors. A hidden entry documented only in the
handbook, hold-to-confirm showing capacity and label, refused while armed (in `MscState` after
`ReleaseCard()`), and `f_mkfs` checked against the ~700 ms watchdog window. After the SD path
has flown.

### #33 — The log records less than the vehicle already knows — 🟢 SUPPORTING

Already on the blackboard and timestamped, never written:

| Source | Not recorded |
| --- | --- |
| `GpsData` | `hAcc`, `vAcc`, `gDOP`, `pDOP`, `vDOP`, UTC date/time, `valid`, `tAcc`, `posCov*`, `velCovValid` — 18 of 27 |
| `EscTelemetryData` | the whole topic, deliberately (below) |
| SharedState | `flight_mode`, `IsArmed()`, `uptime_ms`, `loop_counter` |
| `CrsfLinkData` | `active_antenna` |

- The `GpsData` row changes shape with #49, and the DOPs are zero until NAV-DOP is on.
- **The estimate is not recorded**: PX4's `vehicle_angular_velocity` (20 ms) and
  `vehicle_attitude` (50 ms) in PX4's field order — copied deliberately, both are versioned —
  with the caveat that ours is the burst mean, not filtered and bias-subtracted.
- **ESC telemetry stays off until its record is designed**: per-motor timestamps (one stamp
  dates the record by whichever motor was freshest), the six bus counters, `consumption_mah`,
  `electrical_rpm`. ~134 B, ~3 KB/s; cost is not the reason.
- `error_code` and the failsafe flags record once #42 and #15 produce them.
- **ULog facilities unused**: `'P'` parameters (byte-identical to `'I'`; every log carries the
  tune that flew it, and Flight Review draws `IMU_GYRO_CUTOFF` on the spectrum), `'L'` logged
  strings (`FcLink::SendLog` already produces the stream), `'O'` dropout records (a full ring
  reads as nothing happened).
- Topic names: PX4's only where PX4's record is not the poorer (#44).

### #44 — `imu_health` is our name for `vehicle_imu_status` — 🟢 SUPPORTING

The die temperature is published every second and read by nobody: four bytes at 5 Hz in the
`imu_health` record. Whether the record becomes `vehicle_imu_status` — what Flight Review reads
vibration from, eleven of twenty fields already held — is open: the nine missing fields
summarise a raw stream we log in full, and PX4's fault taxonomy collapses four of our counters
into one. **Clipping wants doing whatever the name**: `accel_clipping[3]`/`gyro_clipping[3]` —
a sample pinned at the rail is indistinguishable in the log and ±16 g is reachable on a quad;
`invalid_samples` counts something else.

### #47 — Nothing starts a log without arming — 🟢 SUPPORTING

`ArmedState::OnEnter` is the only caller of `StartFlight`, so every bench measurement costs an
arm with props on. A `LOG START`/`LOG STOP` on the ctrl channel forwarded as one `MsgId`,
refused while armed. Scalar results back as log lines (`FcLink::SendLog`, mirrored to `'L'` by
#33); time series stay recorded topics; the card benchmark must not write through the logger it
measures; chaining is the host's (`tools/pull_logs.py`). Then measure: card throughput against
the preallocation, loop rate and jitter, tick load. A wire addition for #21/#42's flash.

---

## Diagnostics after the fact

### #21 — Know why the board restarted — 🟢 SUPPORTING

`System` reads `RCC->CSR` at boot and `GetResetCause()` has no consumer, so an in-flight
watchdog reset (#20 names one route) reaches the ground as a cable glitch and a handshake
replay. Carry it in `SystemStatusMsg`, in #42's flash.

**In-flight restart recovery — 🧊 DEFERRED.** The marker belongs in `.noinit` SRAM: survives a
reset, needs no bridge. The hard half is the attitude — in free fall the accelerometer cannot
say which way is up, so rate can be held and level cannot be recovered — and auto-arming needs a
trustworthy airborne test (sustained near-zero g) that only arrives once things have gone wrong.
Fixing what causes the reset is worth more.

### #40 — Faults that survive the battery being pulled — 🟢 SUPPORTING

`Sentinel::imu_fault_latched_` lives in SRAM, so a pilot who lands and pulls the battery takes
the only record of the fault with them; every deferrable fault has the same hole. The answer is
the car ECU's: a code that refuses to be forgotten until someone reads and clears it. `.noinit`
survives a reset, not a power cycle, so the store is the bridge's NVS — a bounded ring of
`ErrorCode` plus flight index, sent over FcLink when raised and again on handshake, a pending
indicator on the UI, cleared only explicitly on the page that shows it. Boot policy stays with
Sentinel: a stored fault reports, never refuses to boot or arm. Doctor (#17) is the reader; #36
carries the notification.

---

## Codebase and tooling

### #18 — Flash the two firmwares as one thing — 🟢 SUPPORTING

The flash targets rewrite everything every time, and two disagreeing images cannot say which is
stale. The bridge owns both STM32 flash paths, so one device deciding what to flash is wiring:
read both build identities, compare against the build, skip what matches. The bridge has
`kMavlinkFlightSwVersion` and `kMavlinkGitHashShort`; the STM32 has no identity at all (also
#31's ULog header). Identity informs; refusing to arm on a mismatch is Sentinel's (#16).

### #19 — Give every SharedState field an owner the compiler knows about — 🟢 SUPPORTING

Fourteen of sixteen setters are public, so nothing says `UpdateRc` is `RcReceiver`'s alone. The
passkey idiom, one key per producer —
`class GpsKey { friend class M10Service; GpsKey() = default; };` on
`void UpdateGps(const GpsData &, GpsKey)` — is elided entirely: ~twenty lines plus `{}` per call
site. It also narrows `friend class Sentinel` and the `Icm42688p`/`Ahrs` mailbox friendship from
every private to one function. A lint measures a proxy; a writer handle is more principled but
runtime and touches every `Init` — revisit if a replay harness needs to substitute a producer.

### #22 — Nothing finds dead code — 🟢 SUPPORTING

`-Wunused` misses unused public header-inline functions entirely. Three advisory tools, no
overlap: `-Wl,--print-gc-sections` (binary ground truth, blind to never-emitted inlines),
`cppcheck --enable=unusedFunction` (sees inline accessors, weak on virtual dispatch), clang
`-Wunused-private-field` (the only one for data members). A list to review, never a gate — a
public API has callers no single tree sees.

### #34 — Linter exceptions are scattered, and no rule can be silenced on one line — 🟢 SUPPORTING

Twenty scripts, and "why does this file not obey" is answered differently for each: two
exceptions files in disagreeing grammars (both empty), six Python constants in four shapes
(`ALLOWED`, `EXEMPT_PATTERNS`, `EXEMPT_PREFIXES`, `REACH_EXEMPT`), and hook-config
`files:`/`exclude:` that CI's bare runs ignore. No script honours a per-line suppression —
`check_comments.py` polices clang-tidy's `NOLINT`, it does not obey one. Shape: one loader, one
file format with path, rule and a mandatory reason, an inline `// LINT(<rule>): <reason>` held to
the `NOLINT` standard, and stale entries failing. Fourteen scripts have no exemption mechanism
and hold because of it — they keep having no entries until something real needs one.
