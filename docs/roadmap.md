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

---

## Handbook

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

Materials, boards, wiring and peripherals have pages, and the firmware section covers
toolchain, configure and flash. Still to write: frame and motors, power, smoke test, RC link,
bench test, and first flight.

Write each as its stage is reached during the real build, while the details are fresh and
the mistakes are still visible. A stage documented from memory six months later is the kind
that omits the step that actually caused the problem.

The pages that exist carry the same gaps where they need the built aircraft rather than a
second draft — the wiring page marks two, both waiting on photographs of real harnesses.

### #6 — Buzzer — 🟢 SUPPORTING

The buzzer is visible in the photographs on **The brain** and wired to `GPIO10`, but its
section on **Sensors and peripherals** is a `TBD(#6)` stub.

Needed: the part used and whether it is an active or passive type, how it is wired to
`GPIO10`, where it mounts, and what the firmware actually sounds it for.


---

## Safety and failsafe

### #15 — The failsafe conditions besides RC loss — 🎯 CRITICAL

RC loss is the only condition Sentinel answers. The others are not, and the rule that resolves
them against each other does not exist.

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

#### The other three flags

`kVehicleFailsafeFlagBattery` is still never set. A pack too flat to fly now refuses to arm, but
that is a ground interlock on the resting voltage; the in-flight condition is a different
reading — it sags with throttle and recovers when it drops — and what it should *do* is #50's
landing rather than a cut. `kVehicleFailsafeFlagImu` reaches the ground as a critical state and nothing acts on it: deciding
what an IMU failsafe does in the air is this item's job. `kVehicleFailsafeFlagGps` stays zero
until something actually navigates by GPS.

#### Arbitration is what none of the conditions owns

Conditions overlap and their preferred actions contradict, so what is needed is one ranked
resolution evaluated every pass rather than four handlers each acting alone. That resolution,
and the ranked action vocabulary it needs, is #50's — this item is the *policy* feeding it:
which conditions count and how fast. Where the policy lives and what enforces it is #16.

### #16 — Sentinel, one owner for arming and failsafe authority — 🎯 CRITICAL

Sentinel is the single owner of the arm decision and of every failsafe condition #15 defines.
It owns the arm path and the IMU conditions. Three things are still outside it: the remaining
failsafe conditions, FcLink peer loss, and the watchdog.

#### The conditions still live in a telemetry builder

`TelemetryPublisher::BuildSystemStatusMsg` derives GPS, battery and RC health from freshness, so
moving detection into Sentinel is *relocating* that computation rather than writing a second
copy of it. `Sentinel::Supervise` is the slot.

**It must live on the STM32**, because its whole purpose is to keep working when the ESP32 is
gone. That is also why FcLink peer loss is Sentinel's to detect on this side (#17 owns the
other): `FcLink` stamps every CRC-passing frame into `FcLinkData::timestamp_us`, and nothing
evaluates that stamp. An ESP32 failure while armed takes the GCS, the telemetry and every
annunciation the pilot has, and nothing on the board knows to say so.

#### What "high priority" means without an RTOS

The STM32 is a superloop, so priority is placement rather than a number: `Supervise` runs first
in `System::Poll` and has to stay there, since a failsafe skipped under load is a failsafe that
does not exist. Any future work-shedding under load inherits that constraint rather than
negotiating with it.

The shape to keep aiming at is an interlock that makes a state impossible rather than one that
polls for it — the former costs nothing per pass, the latter costs something forever.

#### Sentinel should own the watchdog

`Wdg().Kick()` sits unconditional in the superloop in `main.cpp`, so it proves only that the
superloop iterates. A wedged control loop under a healthy superloop is still fed — at 2048 Hz
against a ~681 ms worst-case IWDG, that is well over a thousand missed control cycles. The
stall is already reported, through `ControlLoopLoad::timestamp_us` and
`kSystemStatusFlagLoopAlive`, but nothing resets the board on it. Kicking only after Sentinel
has confirmed that stamp advanced turns the watchdog from "the loop turns" into "the control
loop is running".

Two constraints:

- **Three paths already kick outside the superloop, deliberately.** `EscBootloader` during ESC
  flashing, `Sdio`'s blocking waits, and `LogService::DrainFlush` at disarm. None of them has the
  control loop as its liveness criterion, so the kick policy has to be mode-aware or each of
  them resets the board mid-operation.
- **WWDG is unused and is not redundant.** The F407's second watchdog is a *window* watchdog
  off PCLK1: it resets when fed too early as well as too late, catching a loop running
  wrong-fast, which IWDG structurally cannot. Its ceiling is short — order of 50 ms at typical
  PCLK1 — which suits it to the control loop while IWDG stays on the superloop. Unlike IWDG it
  runs off the main clock, so it does not survive a clock failure.

### #17 — Doctor, health observation on the ESP32 — 🟢 SUPPORTING

The counterpart to #16, and the split between them is **authority, not location**. Sentinel
decides and acts; Doctor observes, correlates and reports, and has no authority at all. Draw
the line anywhere else and conditions like IMU health become ambiguous — with this line, Doctor
describes the IMU's condition and Sentinel decides whether that condition grounds the aircraft.
Doctor being wrong must be harmless, which is what allows it to live across a link that can die.

It does not duplicate `TelemetryPublisher`, which reports STM32-local health because only the STM32
can see it. Doctor adds what only the ESP32 can see — heap, task stack headroom at run time
rather than only through `esp32_stack_check.py` statically, WiFi, flash, transport link stats —
and correlates across FcLink. FreeRTOS makes it a task with a real priority, unlike #16.

#### The part worth doing before the service exists

`Mavlink::StartHeartbeatFrame` gates the armed state and the mode flags on `vehicle_fresh`
and the whole `MAV_STATE` ladder on `kMavlinkSystemStatusFreshMs`, so a silent STM32 reads as
`MAV_STATE_CRITICAL` rather than a stale vehicle. One read is deliberately ungated:
`vehicle_failsafe` tests `have_data` only, because a stale failsafe can only hold the state at
CRITICAL and a link that died with one raised is not the moment to stop saying so.

What is left is narrower than a service: nothing tells the GCS *why* the peer went quiet,
and #41 is the reason it goes quiet most often.

#### A condition it should carry: the card that left

`LogService` panics at boot on a missing or unusable card, then tolerates the same condition in
near silence afterwards. The only notice is a write failing, and writes only happen while armed,
so a card pulled on the bench produces nothing at all until the next arm and then a warning a few
tens of milliseconds into the flight. The aircraft spends that gap believing it is logging.

Nothing detects removal on its own. There is no card-detect line — the pin map carries only the
six bus signals — and `Sdio` has no ISR at all, so presence can only be established by asking over
the bus with CMD13. A periodic probe would put a bench-only concern on the flight path for the
sake of a question nobody is asking in the air, which is why the driver stays demand-driven. The
cheap trigger is `StartFlight`: it runs at the one moment the answer changes a decision, and costs
a single command with no data phase.

Doctor cannot ask the question itself, since only the STM32 touches the card. So the probe stays
on the STM32 and Doctor takes the reporting: the fact joins the logger counters `TelemetryPublisher`
already carries, and Doctor is what turns "mandatory at boot, absent now" into something a GCS
sees rather than a tone nobody is standing next to.

#### The same shape, on the other storage

A failed parameter save is as silent as a failed log write, and for a simpler reason: nothing
reads the answer. `EE::Read` and `EE::Write` return `bool`, `EeConfigStorage` propagates it, and
`RcReceiver::SaveCalibration` returns it -- to no caller anywhere in the tree. So an RC
calibration, an RC map or an accel calibration that did not reach the EEPROM is discovered on the
next boot, as settings that quietly reverted.

Card conditions and parameter-write conditions want the same answer, which is why they belong
together: a `bool` at the call that suffered it is more precise than any counter, and it reaches
nobody. Give the outcome a reader before adding any instrument beside it.

#### Constraint

Doctor reports to MAVLink and the logs, **not to the OLED**. The display stays a bench tool and
flight state stays off it.

### #20 — Sentinel watchdogs on the IMU and the control path — 🎯 CRITICAL

Sentinel weighs the IMU counters and answers per arm state. Every threshold it uses is asserted
rather than measured, and the one recovery it can order does not work in the case it exists for.

#### Every threshold is a guess

| Threshold | Value | Guards |
| --- | --- | --- |
| `imu_loss_threshold_samples` | 5 per mille of gyro ODR, 3 consecutive windows | unclaimed sample bursts |
| `imu_fault_threshold` | 3 path faults in 1 second | sample-path faults |
| `imu_stall_timeout_us` | 20 ms | sample-path silence |

What the loss rule counts is real: `PENDSVSET` is a bit, not a queue, so two bursts published
inside one control tick collapse to one PendSV run and the older is dropped. That is the
measurement that says whether the cascade fits its budget — **and the loop was raised to
2048 Hz without it**, argued from the aliasing corner and the DShot budget rather than measured.

The blackbox `imu_health` record already carries every counter to the card at 5 Hz, so the
reading needed is a bench one rather than a wire change: `missed_samples` across the four cases
that load the loop — disarmed idle, armed at motor idle, throttle sweeping, and every link
streaming at once.

#### The stall recovery cannot clear

A stall with `inflight_` stuck true — a DMA that started and never completed — survives
`RestartSampling`, because nothing clears it or the SPI driver's `busy_`. Aborting a live DMA is
real surgery and belongs here rather than in the driver.

### #50 — The four failsafe conditions, once there is somewhere to go — 🧊 DEFERRED

Item #15 decides the policy for an aircraft with no altitude source and no position: detect,
then disarm. This item is what those same four conditions become once autoland and return-to-home
exist, and it is deferred because the prerequisites are hard rather than because the policy is
unclear. Sentinel stays the owner (#16); what changes is what its conditions are allowed to ask
for, and #52 is where the asking lands -- the vehicle states a procedure needs in order to be
sequenced rather than run from inside the safety authority.

**Four things have to exist first, and all four are gates, not sequencing.** #46 for an altitude
source, without which a descent is a timed throttle and an invented constant. #27 for position
and velocity, without which there is no home vector to fly. #45 for heading, since yaw currently
wanders by design and nothing can hold a course. #49 for a fix worth trusting, because RTH
consumes GPS as a *flight input* rather than as a display field.

#### What each condition becomes

- **RC loss** is the case RTH was invented for: the operator is still there and the link may
  come back. Detection is unchanged -- Sentinel's timeout against `RcData::timestamp_us`, since
  CRSF carries no receiver-asserted failsafe bit. The action changes, and so does the recovery
  rule: a link that flaps must not toggle the aircraft between returning and manual, so handing
  control back wants Betaflight's shape -- sustained clean frames plus a deliberate pilot
  action, not the first good packet.
- **FcLink loss** is where RTH is most right and least helped: the GCS has no picture, and
  Sentinel has to run the whole manoeuvre on the STM32 with the companion gone. But it is only a
  failsafe *together with* RC loss. A live transmitter and a dead FcLink is a pilot with full
  authority, and treating that as an emergency takes the aircraft away from someone flying it.
- **GPS loss** is the difficult one, because it is the input the response runs on. Lost before a
  failsafe, RTH is simply unavailable and the fallback is landing where it stands. Lost *during*
  one, the manoeuvre has to degrade mid-flight rather than continue against a dead-reckoned
  position. This is also where #49's integrity messages stop being diagnostics: a fix that is
  absent is safe, and a fix that is *lying* flies the aircraft somewhere. A spoofed position
  reports excellent `hAcc`, so accuracy fields cannot detect it and NAV-STATUS and SEC-SIG
  become flight inputs.
- **Low battery** is the only one that is a continuous function rather than an edge, and the
  only one that must be able to *preempt* the others. Returning costs energy, so a threshold
  that triggers RTH too late strands the aircraft further from home than landing would have.
  That makes the return threshold a function of distance-to-home rather than a fixed voltage --
  a point-of-no-return calculation -- with a second, lower threshold that lands immediately
  wherever it is. `BatteryData::current` is an `optional`, so on a board without current sense
  there is no mAh integration and the estimate degrades to voltage under load, which sags with
  throttle and reads worst exactly while climbing.

#### Arbitration is the part none of the four items owns

Conditions overlap, and their preferred actions contradict: RC loss asks to return, low battery
asks to land now, GPS loss says returning is not possible. Four independent handlers each acting
on their own condition is the failure mode -- what is needed is one ranked resolution evaluated
every pass, which is how PX4 models it.

Ranking the actions by severity makes "most severe wins" a property of the type rather than a
chain of conditionals, and each condition reports what it *wants* rather than doing anything:

```cpp
enum class FailsafeAction : uint8_t { kNone, kWarn, kReturn, kLand, kDisarm };
```

The resolution has to latch downward. An aircraft that entered land-now must not be talked back
into returning by a battery reading that recovered when the throttle dropped -- which it will,
since that sag is what triggered it.

#### Two things it needs that do not exist

- **A mode Sentinel can command.** Autoland and RTH are flight modes, and the cascade in
  `states.cpp` is what flies the aircraft. If Sentinel executes the manoeuvre itself there are
  two things commanding motors, which is the duplication #19 exists to prevent. Sentinel selects
  the mode and holds the authority to; the mode flies.
- **A home position, captured at arm.** No home means no RTH regardless of which condition
  fired, so it is a pre-arm check rather than a runtime one. That is the point where #48 stops
  being a bench convenience: a degraded fix at arming time is currently harmless because nothing
  navigates, and the moment this item lands it decides whether the aircraft has anywhere to
  return to.

PX4's `_manual_control_lost_at_arming` rule, which Sentinel already applies -- RC absent at
arming does not count as a loss until RC has been seen once -- gains a sibling here, since the
same argument applies to a home position that was never captured.

### #52 — The vehicle states a failsafe procedure needs — 🧊 DEFERRED

Today the state machine has four states and only two of them fly: `Standby` and `Armed`, with
`EscConfig` and `Msc` as bench modes that suspend the cascade. That is enough while every
failsafe procedure is "disarm", because a disarm is not sequencing -- Sentinel writes
`armed_`, `ArmedState::OnStep` sees it and transitions to `Standby`, and no new state was
needed to express it.

`Sentinel::RcLinkPhase` is deliberately not a fifth state. Its three phases -- `kUp`, `kGuard`,
`kRecovering` -- change how much the RC input is believed, not what the vehicle does: `kGuard`
flies the pilot's last frame exactly as `kUp` does, and `kRecovering` is an arming interlock on
a vehicle already sitting disarmed. Conditions and hysteresis are Sentinel's (#16); behaviour is
the state machine's.

**That stops being enough the moment a procedure has somewhere to go.** A descent or a return is
sequencing by definition: it sources setpoints from something other than the pilot, for a
bounded time, with its own exit rules. Running that from inside Sentinel would make the safety
authority a flight-mode sequencer, which is the one thing #16 says it must not become.

#### The states, and what each one changes

- **`Failsafe`** -- entered on a condition Sentinel raises while armed, exited when the pilot
  takes the aircraft back. It is the parent, not a behaviour: what it *does* is whichever
  procedure the condition selected, and #50 decides that mapping per condition.
- **`ReturnHome`** -- setpoints from a navigator against the home vector. Gated on #27 for
  position and #45 for heading, since a course cannot be held by an estimator that lets yaw
  wander by design.
- **`Landing`** -- a controlled descent, which means closed-loop on altitude (#46). A fixed
  throttle and a timer is not a landing, and shipping it under that name is worse than having no
  descent at all.

#### What has to be decided when they land

**Who transitions.** Sentinel decides *that* a failsafe applies; the state machine decides what
running it looks like. The request has to be explicit -- a blackboard field the machine reads,
the way `armed_` already works -- rather than Sentinel calling `ReqTransition`, or the layering
inverts again.

**Where setpoints come from.** `ControlTickFlightLoop` reads `RcData` directly today. A
navigator-driven state needs that source switchable at one point, not patched per stick: a
procedure that reached some readers and not others would fly a blend of the pilot's last frame
and the procedure's.

**How the pilot takes it back.** Betaflight requires sustained clean frames *plus* a deliberate
stick or switch action, because a link that flaps must not toggle the aircraft between
returning and manual. `RcLinkPhase::kRecovering` already carries the first half.

**What `IControlTickState` means for them.** `Standby` and `Armed` implement it and the bench
states do not, which is what `IsControlLoopRunning()` reports. Every state here flies, so all
of them implement it -- and `kArmBlockNotStandby`, currently derived as "control loop running
and not armed", needs re-deriving once more than one flying state is disarmed-and-armable.

Deferred behind the same four gates as #50 -- #46, #27, #45, #49 -- because a procedure with no
altitude, position, heading or trusted fix has nothing to sequence.


---

## Motors and ESCs

### #10 — More than four motors — 🧊 DEFERRED

**TIM1 has four compare channels**, and that is the whole of what remains. DShot is one
burst-DMA transfer per bit, `DCR` configured with base `CCR1` and length 4, so all four motors
are driven from one stream and stay perfectly synchronised. Six motors needs TIM8 and a second
DMA stream with both bursts started together — two frames arriving at different times reads as
a yaw bias. `stm32_limits::kDshotChannelCount` is the ceiling every consumer already asserts
against, so a sixth motor is a driver problem rather than a scattered-constant problem.

Fewer than four motors is cheap — leave the unused channels configured and idle. Deferred
because no non-quad airframe is planned.

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

Both frame types are already listed in the `TODO(#11)` at `crsf_link_service.hpp`.

Deliberately not a panic: an ESC derating is a degraded aircraft the pilot may still want to
land, not a reason to stop the motors.

### #12 — Decide what a desync looks like from the flight controller — 🎯 CRITICAL

`stall_protection` (byte 29) and `stuck_rotor_protection` (byte 22) change what the ESC does
when a motor loses sync: whether it cuts, retries, or keeps trying to drive a rotor that is not
turning. Each choice presents differently in telemetry — eRPM collapsing to zero, current
spiking, or both recovering after a pause — and the flight controller interprets none of it. A
desync today reads as a motor that simply stopped producing thrust.

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
`[kThrottleMin, kThrottleMax]` — stop or forward, with no third case.

Two halves are already in the tree: `EscService::DshotCommand` declares `k3dModeOff` and
`k3dModeOn` and neither is ever sent, and `EscTelemetry::Info::bidirectional` is parsed from
settings byte 18.

What it needs: signed thrust `[-1, 1]` through the mixer and both controllers, a three-way
`ThrustToDshot` over the 3D split (`48–1047` reverse, `1048–2047` forward), `k3dModeOn` actually
issued, a configurator path to enable it, and a settings re-read to confirm the ESC took it.
`kEscDirectionReversed` has to be revisited in the same pass — with 3D on, which way a motor
turns stops being the fixed property that check assumes.

Deferred because no planned flight mode wants it. Autolevel and autonomous flight only ever ask
a motor for *less* lift, never for lift in the other direction; reverse thrust is an acro
capability, and this airframe is not being built for acro.

### #24 — Bidirectional DShot — 🟢 SUPPORTING

The RPM filter in #23 needs motor frequency every loop, and the KISS serial telemetry we poll
cannot give it: one motor at a time, tens of Hz, so a notch chasing a throttle punch would sit on
50 ms-stale RPM. Betaflight feeds its filter from bidirectional DShot instead — the ESC answers
each frame on the same wire.

Bigger than it sounds, but it does **not** cost the burst-DMA design. The trap to avoid is
Betaflight's timer-capture path: latching edge timestamps into CCR1-4 needs four edge-triggered
DMA streams, because four ESCs reply on their own schedules and a DMAR burst has exactly one
trigger — which is why their code makes burst and telemetry mutually exclusive. That is a
constraint of *capture*, not of receiving. Their own default (bit-bang) receives the way we
transmit: one DMA stream on a fixed cadence — pointed at `GPIOE->IDR`, where one 16-bit read
carries all four pins' levels at once. Cadence-sampling levels scales to four asynchronous
repliers; edge-latching timestamps never can.

So the transaction becomes: DMAR burst out of CCR1-4 (unchanged) → one MODER write flips
PE9/11/13/14 to input → the same stream retargeted at IDR samples at ~3× the GCR rate → flip
back. What it costs:

- **The signal inverts.** Bidir DShot idles high, ours idles low, so the output stage changes.
- **The budget is tight but fits.** At DSHOT600 the whole transaction is ~27 µs frame + ~30 µs
  turnaround + ~35 µs reply ≈ **92 µs**, against a 488 µs tick at 2048 Hz — 19%. It would be 75%
  of a 122 µs tick at 8192 Hz, which is why Betaflight forces `pid_process_denom >= 2` on F4
  with bidir. Any further `STM32_CONTROL_LOOP_HZ` increase and this constrain each other and
  want deciding together.
- **Undoing GCR is ~400 lines of software** (`dshot_bitbang_decode.c`): edge-find over the
  sample buffer, 21 bits at 5/4 the DShot rate, transition-decoded, 5-to-4 GCR lookup, CRC.

One thing falls our way: port sampling needs every motor on one GPIO port, and ours are PE9,
PE11, PE13 and PE14 — all GPIOE. AM32 supports the protocol (input type 4, EDT) and
`EscTelemetry::Info::bidirectional` already decodes the flag.

#### Where each piece lives

The IMU path already answers this, layer for layer:

- **Sampler and phase machine → `DShotTim1`.** The turnaround is the same transaction as
  transmit — same timer, same stream, same pins — and a phase machine split across files
  would give one DMA stream two owners.
- **GCR decode → a pure function beside `DShotCodec`**, encode's mirror: samples in, eRPM
  out, no hardware. Pure is what lets 400 lines of bit-twiddling be tested off-target against
  canned buffers.
- **Publish → the driver, from the RX-complete ISR**, exactly as `Icm42688p` parses and
  publishes `ImuHealth` from its DMA-done ISR. Into a **new** blackboard struct —
  `MotorRpmData { timestamp_us, erpm[4], valid_mask, crc_error_count }` — not into
  `EscTelemetryMotorData::rpm`, which has one writer (#19) and keeps it. KISS telemetry stays the
  volts/amps/temp source and its slow rpm becomes the cross-check on the fast one.
- **Consumers read the blackboard, nothing holds a `DShotTim1 *`.** The RPM filter (#23) in
  PendSV, the desync detector (#12) in Sentinel, `telemetry_publisher` if the wire wants it. The
  struct's timestamp is load-bearing from day one: it is what lets the filter fade a stale
  notch the way Betaflight's `rpm_filter_fade_range_hz` does.

#### Two files, split by purity rather than direction

A `dshot_tx` / `dshot_rx` pair is the one arrangement to avoid. They are not two things: the
turnaround is a single transaction over one timer, one DMA stream retargeted mid-flight, one
set of pins and one phase machine. Separate files give that stream two owners, or have one file
reach into the other's state.

- **`dshot_tim1.*` keeps the whole transaction**: burst out, MODER flip, IDR sampling, flip
  back, phase machine. Meaning-free throughout.
- **The GCR decoder gets its own file, not a half of `DShotCodec`.** ~400 lines onto a 133-line
  codec would leave the codec 80% decoder, and the two are not an encode/decode pair anyway:
  outbound is 16 bits (11 throttle + 1 telemetry request + 4 CRC), inbound is 21 bits of
  transition-encoded GCR carrying a 12-bit period, a 4-bit exponent and a CRC. Two protocols
  sharing a wire.

The decoder should be genuinely pure — `<array>`, `<cstdint>`, no hardware header — which
puts it in `libs/` rather than under `stm32/`, so anything host-side can feed it canned sample
buffers without dragging in STM32 headers.

Worth noting on the way past: `DShotCodec` is *not* pure today. `dshot_codec.cpp` includes
`dshot_tim1.hpp`, so the codec already reaches into the driver. That coupling is worth undoing
whether or not bidir ever lands.

It pays for itself twice. Besides #23, per-motor eRPM every loop is the desync detector #12 is
looking for.


---

## Sensors and estimation

### #23 — Nothing filters the gyro in software — 🟢 SUPPORTING

The entire gyro path is `gyro_accum / burst.count` in `Ahrs::Process`. **Nothing filters it** —
no notch, no lowpass, no RPM tracking, and no biquad anywhere in the repo. The first-order
lowpasses that exist sit on the PID's D-input and the yaw output, downstream of the sample.
The chip's own hardware notch exists and is configured `enabled = false`. Betaflight runs five
stages before its PID sees a sample.

Which of those stages earn their place is not obvious, and two of them do not:

- **Static notches default to off** (`gyro_notch1_hz = 0`), and have since the 3.4 defaults pass
  that introduced the dynamic notch. A fixed notch fights a peak that sweeps ~80–800 Hz with
  throttle, paying its phase cost for the whole flight to intersect the noise for a fraction of
  it. They survive for fixed frame resonances, found by hand from a log.
- **The RPM filter is the one that works**, and it is the one we are best placed to build: it
  needs per-motor frequency, and `EscTelemetryMotorData::rpm` is already on the blackboard.
  Four motors × three harmonics × three axes is 36 biquads; coefficient updates need sin/cos,
  which is why Betaflight uses polynomial approximations and staggers one motor per loop.
- **The dynamic notch** tracks peaks with a sliding DFT when no RPM reference exists. A real
  project, and largely redundant once the RPM filter runs.

The filter itself is trivial — one biquad struct, five multiplies and four adds per apply.

**Where the burst lets us beat the reference.** Betaflight filters one sample per interrupt and
runs its notches *after* decimation. Our burst gives the same per-sample rate at a quarter of
the interrupt cost and lets the notches run **before** the decimation. That still matters at
2048 Hz: the loop's Nyquist is 1024 Hz, while the 2nd and 3rd motor harmonics reach roughly
1600 and 2400 Hz at full throttle and fold into the loop band before any post-decimation filter
can see them. The pre-decimation stream is at 8192 Hz, so a notch there has 4096 Hz of Nyquist
to work in and catches all three harmonics while they are still real.

Wants #24 for an RPM source worth tracking.

### #25 — No calibration number has been checked on hardware — 🎯 CRITICAL

The six-pose fit runs from a GCS and the estimator applies what it produces. Every number it
turns on is still an assumption.

The pose bands and the hold time are PX4's numbers carried across, and a hand-held airframe
resting against a bench is not the jig they were chosen for. A run that classifies poses too
readily accepts a corner; one that classifies too reluctantly never advances, and both look the
same to an operator.

#### Calibration needs a GCS

`MAV_CMD_PREFLIGHT_CALIBRATION` is the only way in, so calibrating means a laptop and a link.
Both reference stacks are the same, and both are wrong about it for a bench: the ESP32 already
carries a display, a buzzer and a button, which is every input a six-pose routine needs. A
calibration page there would run the same `AccelCal` over FcLink with no GCS at all, and the
pose set is small enough to render — a name and a progress count. The wire already carries what
it would draw: `kCalStatus` reports the state and the captured-side mask on every edge.

### #27 — An estimator tier below the control loop — 🧊 DEFERRED

Autonomy needs a state estimate the rate loop does not: position, velocity, and an attitude that
stays consistent through aggressive manoeuvring. An invariant EKF is the interesting choice —
its error dynamics are trajectory-independent, which is exactly the regime a fast autonomous
craft lives in, and neither PX4 nor ArduPilot ships one.

The architecture is half-built already. The control loop iterates the burst once and produces two
things at different rates: filtered gyro for control (#23), and delta-angle / delta-velocity
increments accumulated per sample for the estimator, the way PX4's `ImuDownSampler` does with
coning and sculling corrections. Those increments are rate-decoupled by construction, and safe
on the blackboard precisely because an accumulator tolerates a missed read where a stateful
filter does not.

Two things are not the filter's algebra, and are where the time actually goes: the delayed-time
fusion shell that lets 100 ms-old GPS fuse correctly, and the innovation gating that decides
when a sensor is lying. EKF2's real value is that shell, not its equations.

One rule if it lands: **attitude gets a single owner.** Mahony and an IEKF both estimate it, and
both writing `EstimatorState::attitude_world_to_body` is the duplication this repo has spent
real effort removing. PX4 retires the complementary filter into an output predictor; ArduPilot
keeps DCM as an explicit fallback lane. Either is fine; two writers is not.

### #45 — The compass reads, and nothing trusts it yet — 🟢 SUPPORTING

The part is a QMC5883**P**, not the MMC5983MA this item was written around and not the
QMC5883L either: it is the compass half of the HGLRC M100-5883, whose other half is the M10
already on USART2. Fixed at I2C address 0x2C, chip ID 0x80, four field ranges from ±2G at
15000 LSB/G to ±30G at 1000. The driver publishes a body-frame vector in microtesla, and the
ESP32 turns it into the heading a ground station draws.

HGLRC has shipped both parts under the same 5883 badge and they share nothing but the name --
different address, different chip ID, data one register higher, status elsewhere, two ranges
against four. The driver probes 0x0D on a failed bring-up so a board carrying an L is told
which part it has; driving one would mean a second register map, and no aircraft here has one.

Three things stand between that and a heading anything may act on.

**The iron the motors add is not corrected.** The stored calibration is taken with the motors
off, so it holds the frame's iron and not the field four ESCs add when they switch tens of amps
a few centimetres away. PX4's answer is `CAL_MAG_COMP_TYP`: a second calibration that correlates
the field against battery current or throttle and subtracts the fitted share live. The battery
monitor reads current, so the input exists; the run and the term in the correction do not.

**The heading is magnetic, not true.** No declination is applied anywhere. PX4 takes it from a
World Magnetic Model table by GPS position; one airframe in one hemisphere is served as well by a
Kconfig offset measured once against a known bearing, and a mast glued a few degrees off the
nose is the same offset by another name, so one knob covers both.

**The estimator is deliberately not a consumer.** Yaw still bypasses the attitude loop, so a yaw
reference is #27's business, and the compass reaching it is a decision rather than a next step.

### #46 — Barometer, DPS310 — 🟢 SUPPORTING

There is no barometric altitude source, which is why #15 cannot offer a rescue descent and why
the airframe has no altitude hold. GPS carries an MSL figure, but nothing closes a loop on it.
The part is chosen (Infineon DPS310, pressure plus die temperature) and, like #45, shares the
I2C1 bus that is already built.

Needed: a driver and a blackboard fact. Calibration is the one place it does *not* follow #45 -- a baro's zero is a ground reference
re-established at every arm, not a stored constant, so it belongs with the estimator rather than
in `SensorCalService`. PX4 treats it the same way: `baro_calibration.cpp` is an EKF-driven bias
estimate, not a bench procedure.

Its own temperature reading matters more than it looks: pressure output is temperature-
compensated by coefficients read from the part at boot, so a driver that skips them reports
plausible nonsense rather than failing.

**Two CRSF frames wait on this and on #27**, and are worth landing with the driver rather than
after it, since the handset is the only display this aircraft has in flight. `0x09`
BARO_ALTITUDE carries altitude, which is the ground reference above -- a raw pressure reading
sent as altitude is wrong by the day's weather. `0x07` VARIO carries climb rate, which is not a
reading at all but a derivative, so it comes from the estimator or it comes from differentiating
noise. The encoders are a few lines each; the part they wait on is the number being meaningful.

### #48 — Decide whether GPS quality gates arming — 🟢 SUPPORTING

`hDOP` is plumbed end to end and read by nobody as a condition: `M10Service` publishes it,
`TelemetryPublisher` puts it on the wire, and `Mavlink` re-emits it as `eph`. Nothing compares it
against anything. Sentinel's arm path takes no view of GPS at all. It is also zero on every
build, because NAV-DOP is not enabled -- #49 has to land before any of this can be written.

The decision is not the comparison, it is what a bad number is allowed to do. A quad that
refuses to arm indoors because it cannot see satellites is broken for the bench, and this
aircraft spends most of its life there — so a hard gate is wrong on the current airframe, and
"warn, do not refuse" is the honest default while no mode navigates by GPS.

- **`num_sats` and `fix_type` are the coarse conditions**, and `hDOP` is the one that says the
  fix is *degraded* rather than absent. A gate written against DOP alone reads a good number
  from a receiver reporting no fix at all, because DOP describes satellite geometry, not
  whether a position was computed from it.
- **Which way it points depends on #27 and #45.** Nothing navigates today, so a poor fix costs
  nothing in the air; the moment an estimator consumes position, arming on a degraded fix stops
  being a bench convenience and starts being a flyaway.
- **The threshold cannot be picked from the datasheet.** DOP under an open sky and DOP beside a
  building differ by more than any published figure predicts, so this wants numbers off the
  actual card before a constant is written down — which is #33's job, since `hDOP` is among the
  18 `GpsData` fields the log does not record, and #49's, since it is among the fields the
  receiver is not asked to send.

Whatever it becomes, it is a Sentinel condition and not a check in the GPS driver: #16 owns the
arm decision, and a second component holding a veto is the shape #19 exists to prevent.

### #49 — Match PX4's UBX message set, and use it to decide the fix is trustworthy — 🟢 SUPPORTING

**NAV-DOP is disabled and four DOP fields are plumbed anyway.** `stm32_config.hpp` renders
`.nav_dop = false`, so `kIdNavDop` never dispatches, `dop_data_` is never written, and the
`gDOP`/`pDOP`/`hDOP`/`vDOP` that `BuildGpsData` copies out of it are structurally zero. MAVLink
survives it -- `mavlink_tx.cpp` sends `UINT16_MAX` for a zero DOP, which is the wire's word for
unknown. The log does not: `GpsRecord.hdop` writes `0` every 100 ms, and `0.00` reads as a
*perfect* fix in any viewer, which is worse than the field being absent. Turning the message on
is a one-line config change; everything downstream is already built for it.

**The enabled set is close to inverted against PX4's.** Comparison is against the `u_blox10`
path in `PX4-GPSDrivers/src/ubx.cpp`, which is the same receiver generation.

| Message | PX4 | 32Raven |
| --- | --- | --- |
| NAV-PVT | yes | yes |
| NAV-DOP | yes | **no** |
| NAV-STATUS | yes | no |
| MON-RF | yes | no |
| SEC-SIG | yes, non-fatal on NAK | no |
| RXM-COR | yes | no |
| NAV-SAT | only when satellite info is asked for | no |
| NAV-COV | **no** | yes |
| NAV-EOE | **no** | yes |

PX4 also explicitly writes zero to NAV-TIMEGPS and RXM-SFRBX, because another firmware may have
left them enabled in the receiver's non-volatile config. Receiver bandwidth is something it
reclaims, not merely something it declines to spend.

**The covariance goes.** `NAV_COV` does not appear anywhere in PX4's driver -- not unused,
absent -- and `sensor_gps` carries no covariance field for it to land in. EKF2 builds R from
three scalars instead: `pos_noise = max(hacc, EKF2_GPS_P_NOISE)` and
`vel_var = sq(max(sacc, EKF2_GPS_V_NOISE, 0.01f))`, one isotropic variance across all three
velocity axes. It computes `pdop` as `sqrt(hdop² + vdop²)` rather than reading the receiver's.
So the three `posCov*` floats `GpsData` carries are for a shape no reference estimator consumes,
at 640 B/s on a 11.5 kB/s line.

**What replaces them is already parsed and thrown away.** `M10PVTData` holds `sAcc`, `headAcc`
and `velN`/`velE`/`velD`; `GpsData` keeps none of the five. `sAcc` is exactly the field PX4's
velocity R comes from, and `vel`/`hdg` are derived from `gSpeed`/`headMot`, which are planar --
vertical velocity is not recoverable from what is kept, and an estimator fusing GPS velocity
needs it. Add those five plus `hAcc`/`vAcc`, drop `posCov*`, and the struct gets smaller while
saying more.

**NAV-EOE stays, and is ours on purpose.** PX4 sets `_use_nav_pvt` and publishes straight off
NAV-PVT, so it never needs an epoch barrier. `M10Service` joins several messages and uses EOE
plus a matching `iTOW` to publish the set atomically -- 12 B/epoch for a guarantee PX4 does not
need because it does not join. That reasoning holds only while the join has more than one
message in it: drop NAV-COV while NAV-DOP is still off and the epoch is NAV-PVT alone, at which
point EOE is pure overhead. The two changes are one change.

**The integrity half is the point.** NAV-STATUS, MON-RF and SEC-SIG are where PX4's jamming,
spoofing, AGC and noise fields come from, and 32Raven has no equivalent for any of them. They
describe whether the fix can be *believed*, which is a different question from the accuracy
fields describing how precise it claims to be -- a spoofed position reports excellent `hAcc`.
That distinction is what #48 needs and does not currently have: `num_sats` and `fix_type` say a
fix exists, DOP says the geometry is good, and only these say the signal is real. A receiver
under a jammer degrades in a way DOP alone will not show.

Budget, at the 100 ms measurement rate: the current PVT+COV+EOE set costs 1760 B/s of a
11.5 kB/s line. PVT+DOP+EOE costs 1380 B/s, leaving room for the integrity messages at a
divided rate -- PX4 runs NAV-SAT at every tenth epoch for the same reason.

Sequencing: the message-set and `GpsData` changes are independent of everything and can land
alone. #33 records the new fields once they exist. #48 cannot pick a threshold until #33 has put
real numbers on a card, and should be rewritten against integrity state rather than DOP alone
once the messages arrive. #27 and #45 are what eventually consume `sAcc` and the NED velocities.

### #51 — Per-cell voltage, sensed rather than divided — 🟢 SUPPORTING

`Battery::EstimatePercentage` divides pack voltage by `STM32_BATTERY_CELL_COUNT` and maps the
result linearly between the empty and full cell thresholds. That is the only per-cell figure the
aircraft has, it never leaves the function, and it is wrong in two independent ways.

**It cannot see imbalance.** One cell sagging is the failure that ruins packs and costs thrust,
and it is exactly the failure a divided average hides: five healthy cells carry the mean while
the sixth collapses. The number looks best when the pack is worst.

**The map is linear and the discharge curve is not.** A lithium cell is flat through the middle
of its range, so a straight line between two thresholds moves the percentage too slowly there
and too quickly at both ends. Under load it reads low as well, since the sag is current, not
charge -- `filtered_voltage_v_` smooths the noise, not the offset.

Sensing it needs a balance-lead tap: one divider per cell into an ADC network, or a dedicated
front end. That is a board change, not firmware, which is why the estimate stands in the
meantime rather than being deleted.

Three things unblock together when it lands:

- **CRSF `0x0E` CELLS**, which EdgeTX renders per cell. Sending the divided figure would draw a
  perfectly balanced pack however far one cell had gone, so it stays unsent until the reading is
  real -- an encoder is a few lines once it is.
- **A battery failsafe that trips on imbalance**, not only on pack voltage. #50 treats low
  battery as a continuous condition against distance-to-home; a single dying cell is a different
  condition with a different answer, and today nothing can express it.
- **A state of charge worth the name.** `EscTelemetryData::consumption_mah` already carries
  integrated charge wherever the ESCs have a shunt, which is a better basis than voltage under
  load; the two together beat either alone.


---

## Links and telemetry

### #9 — Share the FcLink frame parser — 🧊 DEFERRED

The byte-at-a-time receive state machine exists twice, in `stm32/Services/fc_link.cpp` and
`esp32/services/fc_link.cpp` — same states, same transitions, plus an `RxState` enum and an
`rx_pkt_internal_` struct declared identically in both headers.

The reason to fix it is not the line count. **CRC verification is implemented twice,
differently.** The STM32 rebuilds header and payload into one contiguous buffer and runs
`checksum::XModem` over it; the ESP32 feeds `XModemUpdate` byte by byte across magic, id, length
and payload. The two agree only for as long as `message::Header` stays packed as exactly
`{magic[2], id, len}`, and a divergence would not fail loudly — the link would simply stop
carrying packets.

What differs between the two sides is policy, not parsing. The STM32 resyncs silently and
dispatches inline; the ESP32 counts invalid frames, logs them, sounds the error tone, panics
past a threshold, and queues rather than dispatching.

- Add a policy-free parser to `libs/`: one byte in, one verdict out — need-more, bad magic,
  bad length, bad CRC, packet complete — so each firmware keeps its own reaction to each.
- Leave `Poll`, transmit, handshake, the ring buffers and the read budgets where they are.
  Those genuinely differ: an interrupt-fed byte ring on one side, block reads from the ESP-IDF
  driver on the other.

Deferred rather than supporting because it rewrites the receive hot path on both firmwares. Do
it once the ESC configurator work has been confirmed on hardware, so that a misbehaving bench
session has one candidate cause instead of two.

### #41 — The radio goes dark on every bench page — 🎯 CRITICAL

The Telem UART is the aircraft's MAVLink link: `TelemUartServer` on GPIO20/21 at 57600, the
SiK default, brought up by `ServingState` which is where `main.cpp` starts the machine. The
WiFi and USB MAVLink modes are bench transports reached from the menu, not the vehicle's link.

Four states call `Mavlink().SetTelemetryLink(false)` in `OnEnter`. One of them has a reason.

| State | STM32 | FcLink | Telem UART | Wanted |
| --- | --- | --- | --- | --- |
| Service, waiting for a host | Standby, running | free | free | on |
| WifiLog, waiting for a host | Standby, running | free | free | on |
| EscConfig | suspended | MSP relay | MAVLink, full | not-ready |
| UsbLog (MSC) | suspended | grant only | free | on, not-ready |
| LogPull, transferring | Standby, running | saturated | free | narrowed |
| Program, flashing | ROM bootloader | held by Programmer | free | dark |

`ServiceState` and `WifiLogState` never touch the flight controller — they start the network and
wait, indefinitely, while it runs normally. `ProgramState` is the only one with a physical
reason: BOOT0 is asserted, `Programmer` owns USART1, and no firmware is left to publish.
Recovery Service mode is a seventh case, where the STM32 is halted in its panic loop and the
ESP32 holds the only fact worth sending — which is #40's.

A dark link and a dead board are the same thing from the ground. That is the ambiguity #42
removes from the fields, and there is no point making the values honest while the link that
carries them disappears on the pages that make them interesting.

#### The page should pick a profile, not own the radio

`SetTelemetryLink(bool)` from `OnEnter` is a menu switching off a permanent fixture of the
aircraft. A page should narrow the stream instead — full, heartbeat-plus-status, or nothing,
with only Program picking nothing. That also answers the one real contention: `LogPullState`
saturates FcLink's 64 B/ms TX budget with chunks, so fresh SystemStatus competes with the
transfer while the Telem UART itself sits idle.

`SetTransport` has the same shape. The four states never call it, so they inherit whatever the
last page left — UDP after MavlinkWifi, CDC after MavlinkUsb. And `Mavlink().Poll()` is called
only from the four states that stream today, so raising the flag is not sufficient on its own.

#### boot_state is a readiness state, not a boot phase

On the bench pages `boot_state` reads `kBooting` — "starting up" — when the truth is "a
configurator holds the motor lines". `MAV_STATE_CALIBRATING` is the honest value, and PX4
sends exactly that for `in_esc_calibration_mode`. It needs a `kNotReady` enumerator in
`libs/message.hpp`, so it costs a dual flash and wants to travel with #42's wire additions.

Of the nine `MAV_STATE` values, four are ever sent: BOOT, STANDBY, ACTIVE, CRITICAL. UNINIT,
CALIBRATING, EMERGENCY, POWEROFF and FLIGHT_TERMINATION are unreachable. EMERGENCY is the
other absence worth closing — "lost control over parts or the whole airframe" is a real
distinction from CRITICAL's "can however still navigate", and #15 is what would decide it.

### #42 — SystemStatus reports values nothing produces — 🟢 SUPPORTING

Two fields the STM32 still fills with constants.

| Field | Today | Wanted |
| --- | --- | --- |
| `error_code` | `kOk`, always | Sentinel's latched fault code |
| `errors_count1..4` | four literal zeros in the pack call | four of the `ImuHealth` counters |

**`error_code` has a producer for the first time.** A panic halts the board, so a running
board had nothing to report and `kOk` was defensible. `Sentinel::imu_fault_latched_` ends
that — an aircraft flying on a deferred fault holds a real code in a private member and
nothing carries it. `failsafe_flags` says something is wrong; this says which.

Four of the IMU counters fit `errors_count1..4`. A wire change, so it wants #21's reset cause
and #41's `kNotReady` in the same flash.

Reading any of it needs #41 first.

### #43 — A current reading is trusted as far as it can saturate an int — 🟢 SUPPORTING

Blocked on a real sensor reaching `PC1`. When one does: `STM32_BATTERY_CAPACITY_MAH` and a
max-current knob inside the current-monitoring menu, a plausibility bound in the driver where
the negative clamp and deadband already live (a reading above what the pack can deliver is
provably false), and an over-current condition feeding #15's battery flag. The bound cannot
detect an absent sensor — a floating pin reads amps that fit any 6S budget — which is what
the build-time knob is for.

### #55 — A byte count is summed into the ESC fault total — 🟢 SUPPORTING

`EscTelemetryData::Total()` adds `rx_drop_bytes` to four event counters. Every other fault
struct — `UartFaults`, `SpiFaults`, `AdcFaults` — sums like with like, which is what lets
`TelemetryPublisher::UpdateFaultWindows` window them by counting events. Window this one the
same way and a single DMA drain that outran the ring reads as however many bytes were in
flight. Nothing does today, which makes this a trap rather than a defect: whoever reaches for
`Total()` next is the one who finds it.

`PublishIfChanged` reaches for it already, and stamps `timestamp_us` — the bus's heartbeat,
and what `ESC_STATUS` carries to the ground as its `time_usec` — on a dropped byte as though a
frame had moved.

The shape the other structs already have: count the overflowing drain as one event and put
that in `Total()`, keeping `rx_drop_bytes` as the magnitude beside it. The ULog
`system_health` record names `esc_rx_drop_bytes` and should keep it, so the new counter is a
field added to the schema — a wire change, wanting the same flash as #42's.

Reads zero on a bench with four ESCs answering, so nothing observes it until the ring is
actually pressed.

### #56 — RC calibration is a concept the link does not have — 🟢 SUPPORTING

CRSF carries channels as an 11-bit value over a range the protocol fixes, so there is nothing
per-airframe to measure. Endpoints, subtrim and reverse are already set in the transmitter
before the values reach the air, and `RcReceiver::ApplyCalibration` then maps them a second
time against a stored `min`/`trim`/`max`/`rev`. Two places hold the same setting and nothing
notices when they disagree. Betaflight and INAV omit the whole idea for this reason; PX4 keeps
it because it still supports PPM and analog receivers, where the range genuinely varies.

It is not a small thing to carry. The set is 64 of the roughly 101 parameters the bridge
serves, 112 of the 120 bytes `FcConfigCache` holds, the `RCC1` EEPROM record, and the only
real user of the cache's write-retry path.

- **STM32.** `ApplyCalibration` and `ScaleSegment` in `rc_receiver.cpp` collapse to one fixed
  map from the CRSF range onto `kCalibratedMinUs`/`kCalibratedMaxUs`, which is a constant
  rather than configuration. `LoadOrInitRcCalibration` and `SaveRcCalibration` go with them,
  and so does the `[stm32.rc.calibration]` record in `config/ee.toml`.
- **ESP32.** `RcCalField`, `RcCalibrationParamRef` and the three resolve/encode/set functions
  leave `mavlink_param.*`, which drops `ParamRef` back to one type instead of a variant.
  `FcConfigCache` loses a slot and `Record::kRcCalibration`. `RC_CHAN_CNT` is served out of
  `kRcCalibrationChannelCount` and dangles once that is gone.
- **Wire.** `RcCalibrationConfigMsg` and its three `MsgId`s leave `libs/message.hpp`, so the
  contract hash moves and both boards have to be flashed together. Two error enumerators go.
- **Generator.** `_RC_BLOCK_RE`, `_RC_FIELD_RE` and `_CHANNEL_COUNT_RE` in
  `generate_param_metadata.py` exist only to read the RC encoder, and are the reason that
  script parses a member function body rather than a table. Removing the set removes the cause.
- **GCS fork.** `PX4AutoPilotPlugin::vehicleComponents` appends `PX4RadioComponent`
  unconditionally. Leaving it once the parameters are gone gives a broken setup page rather
  than an absent one, so the two repositories have to land together.

The RC *map* is not part of this and stays. Which channel carries roll is per-airframe, and
the link has no opinion about it.

### #53 — The LR900-P replaces WiFi as the MAVLink link — 🧊 DEFERRED

The Telem UART is already the aircraft's link (#41), 57600 and SiK-shaped, and the MicoAir
LR900-P goes on it: 2.1 KB/s over the air by default, which MicoAir recommends for a flight
controller, 1.1 and 0.4 KB/s below that, and 3.2 KB/s downlink in its FHSS mode. The build
already checks the MAVLink ladder against `ESP32_MAVLINK_TX_LINK_AIR_RATE` — every periodic
message at its longest, against the declared air rate or the UART's line rate, whichever is
lower, with a 20 % margin — and the choice sits at UART-bound until a radio is on the port. So
landing the radio is setting that choice to the mode the radio is configured in, and reading
what the build says.

What it will say: the ladder at its defaults is ~1.57 KB/s, and RC_CHANNELS every 40 ms is
1.35 KB/s of it. At 2.1 KB/s the ladder fits with 7 % of the margin to spare; at 1.1 or
0.4 KB/s it does not, and RC_CHANNELS is the first knob to turn — deliberately, since 40 ms is
the rate the GCS's stick display was tuned to, not a default nobody chose.

Open with the radio, not before:

- The uplink is the narrower direction in FHSS mode (1.6 KB/s) and carries the GCS's commands
  and parameter traffic, which the check does not model.
- The bench pages in #41 should narrow the stream to a profile; with a budget declared, a
  profile is a budget.
- Whether the ladder should stretch to the link at runtime the way the CRSF one does, or stay
  a build-time check. A radio's air rate cannot change underneath the config the way a handset's
  ratio can, so a check is probably all it needs.


---

## Operator interface

### #36 — Notifications the display can carry — 🟢 SUPPORTING

Several conditions today are announced only by a warning tone and an `ESP_LOGW` nobody is
watching: the ESC port refusing to open because the vehicle is armed, the STM32 never granting
a session, the MAVLink TX queue dropping its oldest item. The beep says *something* happened
and the console says what — but the console needs a cable, which is the one thing the bridge
exists to avoid. The screen is right there and says nothing.

A notification is a **message drawn over whatever screen is up, with the state machine
untouched underneath**. That is the whole distinction from `AppState::kHardError`, which is
terminal and replaces everything: telemetry keeps flowing, the pull keeps transferring, and
dismissing it returns to exactly the screen that was there. The pieces already exist —
`Ui::LoadWidget` swaps widgets, `IWidget` is a two-method interface, and `NotifyUserActivity`
already wakes the panel for events worth seeing.

- **Dismissal is a hold**, which collides with the menu. `CycleOnButton` gives every state the
  same gesture pair, and a long press means "swap menus" everywhere. A notification has to
  consume the hold that dismisses it and not swap menus with the same press — one edit, in the
  one helper, now that the gesture lives in a single place.
- **More than one can be pending.** A queue with a depth and a drop policy, and dedupe for a
  condition that re-fires — armed-refusal holds for as long as the vehicle stays armed, and
  today it is edge-triggered precisely so it does not repeat.
- **Auto-expiry is a question, not a given.** A hold is a deliberate acknowledgement, which is
  right for something the pilot must see; a timeout is right for something merely informative.
  Deciding per notification rather than globally is what keeps both honest.
- **The STM32 already has a channel.** `kLog` packets arrive and are printed under the peer tag
  (`FcLink::kPeerLogTag`); a severity on that message is what turns the interesting ones into
  notifications without inventing a second wire format.

The natural first users are the sites that already play `kWarning`, plus #11's ESC derating
warning, which needs somewhere to be seen the moment it exists.

### #39 — The LED says less than it has states — 🟢 SUPPORTING

Six pages, three signals, and no scheme tying them together.

| Page | Pattern |
| --- | --- |
| Serving | breathe, 3 s |
| Service | blink, 400 ms |
| EscConfig, UsbLog, WifiLog | blink, 800 ms — identical on all three |
| MavlinkWifi, MavlinkUsb | nothing set |
| Program | off |

So the LED distinguishes "a tool page" from Service but not which tool page, and on the two MAVLink
pages it shows whatever the previous page happened to leave behind.

**A one-shot never gives the page back.** `SetPattern(..., repeat_count)` installs `kOffStep`
when the count exhausts rather than restoring the pattern underneath, so every transient use of
the LED permanently claims it. `mavlink_rx.cpp` fires a `kDoubleBlink` per received heartbeat,
which on a page with no pattern of its own reads as a link pulse — and on any page that has one
would silently end it. The mechanism wants a foreground/background split: a page installs
background, a transient plays over it and hands it back.

**The STM32 has no vocabulary at all.** `stm32/Drivers/led.hpp` exposes `Set`, `Toggle` and
`IsOn` — a pin, not a signal. That was fine while every annunciation went to the ESP32's buzzer
and screen. Now that the switch can arm without the bridge, an arm refused on the RC path with
the bridge dead has one LED, no tone and no display — a refusal the pilot has no way to hear,
which is its own failure mode.

Worth deciding what the LED *means* before adding patterns to it — page identity or link
liveness on the ESP32, armed on the STM32 — because it attempts all three with no priority
between them.


---

## Logging

### #26 — Blackbox logging — 🟢 SUPPORTING

Neither retrieval path has been exercised: the USB Log page mounting the card on a PC over MSC,
and `tools/pull_logs.py` returning a byte-identical copy over WiFi.

Content gaps are #31 and #33; formatting is #32.

### #31 — Tuning-grade log content — 🟢 SUPPORTING

The blackbox records what the vehicle *did*; a PID tune also needs what the controller *asked
for*. The rate setpoints, torque command and per-motor thrust flow rate_controller → mixer →
esc_service as locals and never touch the blackboard, so the logger cannot see them.

- **A `ControlOutputs` POD on the SharedState**, written by the control tick — one plain store,
  the same shape as `UpdateEstimate`. ~36 bytes at 2048 Hz adds ~80 KB/s to the stream.
- **A decimation knob** for the fast topics, once real flights show whether full rate is worth
  the file sizes.
- **Firmware identity in the ULog `I` messages.** The STM32 has no version constant (#18); when
  it gains one, stamp it so a log names the code that flew it.

### #32 — Format the card on the vehicle — 🟢 SUPPORTING

A card the firmware cannot mount is currently a trip to a PC, and a card over 32 GB is a trip
to a PC with third-party tooling, because Windows refuses to put FAT32 on one. `f_mkfs` with
`FM_FAT32` does both. The cost is small and measured: `FF_USE_MKFS=1` takes `ff.c` from 7,552
to 9,640 bytes of text — **+2,088 B of flash** — and no RAM at all, because `f_mkfs` takes an
explicit work buffer and `LogService::staging_[0]` is 4 KB that is provably idle whenever a
format could run.

**It must never be automatic.** `f_mount` failing does not mean the card is blank: it also
means a transient SDIO error, or a directory corrupted mid-write on a card whose log sectors
are perfectly readable. Formatting on mount failure would wire the destruction of flight data
to the exact symptom that says flight data is worth recovering — and would do it silently at
power-up, in the post-incident case, before anyone could object.

So it is a deliberate act, and the UI is what makes it one:

- **A hidden entry**, not a fourth stop in the config menu cycle. Reachable by a gesture that
  cannot be stumbled into and is documented only in the handbook, never on the screen.
- **Hold-to-confirm** with the card's capacity and volume label shown, so the operator is
  looking at what they are about to erase.
- **Refused while armed**, on the same interlock as the MSC grant — it is a card-owning
  operation, so it belongs in `MscState` after `LogService::ReleaseCard()`.
- **Bounded and fed.** A FAT32 mkfs writes both FAT copies — several MB on a large card. The
  blocking waits in `Sdio` already kick the watchdog, but `f_mkfs` itself must be checked
  against the ~700 ms window rather than assumed to fit.

Sequenced after the SD path has flown: the whole argument for a hidden format is that the
operator understands what they are erasing.

### #33 — The log records less than the vehicle already knows — 🟢 SUPPORTING

Distinct from #31, which needs new plumbing before anything can be recorded. Everything here is
already on the SharedState, already timestamped, and simply never written — so the work is
record fields and format strings, not a design change. The whole list costs a few KB/s against
a stream the raw IMU pair dominates at ~328 KB/s whenever it is enabled.

| Source | Not recorded |
| --- | --- |
| `GpsData` | `hAcc`, `vAcc`, `gDOP`, `pDOP`, `vDOP`, UTC date/time, `valid`, `tAcc`, `posCov*`, `velCovValid` — 18 of 27 fields |
| `EscTelemetryData` | the whole topic — recording is off, see below |
| SharedState | `flight_mode`, `IsArmed()` |
| SharedState | `uptime_ms`, `loop_counter` |
| `CrsfLinkData` | `active_antenna` |

**The `GpsData` row is a moving target.** #49 drops the `posCov*` floats and adds `sAcc`,
`headAcc` and the NED velocities, so the set worth recording changes shape before this lands.
Four of the DOP fields are also zero on every build until that item enables NAV-DOP, and
recording a zero DOP is worse than recording nothing.

**The estimate is not recorded at all.** The raw FIFO topics carry the full-rate truth, so what
is missing is PX4's pair, through the scheduler rather than pushed: `vehicle_angular_velocity`
at 20 ms and `vehicle_attitude` at 50 ms, in PX4's field order and units. Both live in
`msg/versioned/` with `MESSAGE_VERSION = 0`, so the shape has to be copied deliberately rather
than approximated — and ours would be the burst mean, where PX4's is calibration-corrected,
notch/LPF filtered and EKF-bias-subtracted, which the record cannot claim to be.

**ESC telemetry is recorded not at all, deliberately.** `STM32_LOG_TOPIC_ESC_TELEMETRY_ENABLED`
defaults off. The four motors answer their own requests and carry their own timestamps, so a
record with a single timestamp dates itself by whichever motor happened to be freshest, which
the reader cannot identify — and a motor falling silent has to be inferred from `valid_mask`
and a frozen RPM rather than seen. A log that quietly misattributes is worse than one that
says nothing, so the topic stays dark until the record shape is decided rather than inherited:
per-motor timestamps, the six bus counters (`crc_error_count`, `uart_error_count`,
`rx_dma_error_count`, `rx_drop_bytes`, `frame_count`, `unassigned_frame_count` — richer than
PX4's single per-ESC `esc_errorcount`), `consumption_mah` and `electrical_rpm`. Roughly 70 →
134 bytes and ~3 KB/s, so cost is not what is holding it.

**`error_code` is absent because nothing produces it**, not because the logger skips it: it is
hardcoded to `kOk` in `TelemetryPublisher`, so recording it today would write a constant.
`failsafe_flags` only carries the IMU bit until #15 defines the rest.

**The ULog facilities the writer does not use yet**, each of which turns data into something a
viewer renders rather than something a reader has to infer:

- **`'P'` parameters.** Byte-identical layout to the `'I'` info message already emitted, so it
  is a call, not a feature. Dumping the generated config means every log carries the tune that
  flew it — and Flight Review draws `IMU_GYRO_CUTOFF` onto the noise spectrum, which is the
  filter question answering itself.
- **`'L'` logged strings.** `FcLink::SendLog` already produces this stream; today it scrolls
  past on the ESP32 console and is gone. As ULog messages the same text lands on the plot
  timeline beside the anomaly it explains.
- **`'O'` dropout records.** A full ring increments `dropped_bytes` and moves on, so a gap
  reads as "nothing happened". Viewers draw a dropout; they cannot draw a counter.

**Topic naming is settled for the raw pair and open for everything else.** Flight Review is
hardcoded to PX4 topic names, so `sensor_gyro_fifo` / `sensor_accel_fifo` inherit its whole
analysis suite for free. Every topic above still carries a name of our own, which buys nothing
from any viewer. #44 works that check through for one of them, and the rule it argues for: take
PX4's name only where PX4's record is not the poorer of the two.

### #44 — `imu_health` is our name for `vehicle_imu_status` — 🟢 SUPPORTING

The die temperature is measured, scaled and published to the blackboard every second, and no
consumer reads it. PX4 carries the same value as `float32 temperature` in degrees Celsius on
`sensor_gyro` / `sensor_accel`, and as `temperature_accel` / `temperature_gyro` on
`vehicle_imu_status`, logged at 1000 ms — the rate `PublishTemperature` already limits itself
to. Adding it to the `imu_health` record is four bytes at 5 Hz.

The larger question is whether that record should be `vehicle_imu_status`. It is unversioned,
so it is the same stability class as the FIFO pair already matched field-for-field, and it is
what Flight Review reads its vibration metrics from. Eleven of its twenty fields we already
hold: both device ids, both error counts, both rates, both raw rates, both temperatures.

Against the rename, and the reason not to do it reflexively:

- **The nine missing fields summarise what we already log in full.** `add_raw_imu_gyro_fifo()`
  is opt-in in PX4, so `vehicle_imu_status` exists to stand in for a raw stream that is usually
  absent. `gyro_vibration_metric` is an EWMA of consecutive-sample difference magnitude — one
  number where we record the spectrum. `var_gyro` is recoverable offline from the same samples.
- **PX4's fault taxonomy is coarser than ours.** `overruns`, `dma_start_fails`,
  `spi_errors` and `parse_fails` all collapse into one `gyro_error_count`.

**Clipping is the exception, and wants doing whatever the topic ends up called.** A sample
pinned at the 20-bit rail is indistinguishable from a real reading once it is in the log, and
+/-16 g is reachable on a quad. `accel_clipping[3]` / `gyro_clipping[3]` are per-axis counts of
exactly that, and nothing here detects it -- `invalid_samples` counts the chip's no-fresh-data
sentinel, which is a different thing.

### #47 — Nothing starts a log without arming — 🟢 SUPPORTING

`ArmedState::OnEnter` is the only caller of `StartFlight`, so every measurement the card can hold
costs an arm. On a built aircraft that means spinning props for a capture that has nothing to do
with flying, and it puts the one interlock that matters between the operator and a number they
wanted on the bench.

Nothing else is missing. The control tick produces IMU bursts while disarmed, the scheduled topics
run off the main tick, and `StartFlight`/`StopFlight` already open, close and report a session --
only the second caller is absent.

- **A session command on the ctrl channel.** `LOG START` / `LOG STOP` where the host already
  speaks, forwarded over FcLink as one `MsgId`. Refused while armed, since arming owns the
  session -- the same refusal the calibration request takes.
- **A scalar result goes back as a log line.** `FcLink::SendLog` already reaches the host, and
  #33 already wants that stream mirrored into the ULog as `'L'` records -- so one call site
  gives both the terminal readout now and the same text on the plot timeline later.
- **A time series stays in the log.** Loop rate, jitter and tick load sampled over a minute are
  data rather than verdicts; they belong as recorded topics, where a viewer already draws them,
  and streaming them would fight FcLink's 64 B/ms TX budget for nothing.
- **The card benchmark must not write through the logger.** The logger's own writes are the
  load under test, so a result recorded through it changes the number it reports.
- **Chaining is the host's job.** One command opens a session, runs the sequence, closes it and
  pulls the file back; `tools/pull_logs.py` is already the retrieval half (#26).

Worth measuring once the session exists: card write throughput against the preallocation, the
control-loop rate and its jitter, tick load, and the bus counters #42 wants on the wire anyway.

A wire addition, so it travels with #21's reset cause and #42's counters rather than costing a
dual flash of its own.


---

## Diagnostics after the fact

### #21 — Know why the board restarted — 🟢 SUPPORTING

`RCC->CSR` records what caused the last reset — power-on, brownout, IWDG, window watchdog,
software, pin. `System` reads it at boot and clears `RMVF`, but `GetResetCause()` has no
consumer, so an in-flight watchdog reset still reaches the ground indistinguishable from a cable
glitch: the FC silently restarts and the GCS sees the handshake replay. #20 names one concrete
route to exactly that.

What remains is carrying it in `SystemStatusMsg`, which turns a silent restart into a reported
one. #42 wants the same field in the same flash.

#### A reset silently discards the gyro calibration

`Init` calls `ClearUserOffsets()` on every boot and nothing puts a value back, so a board that
restarts in flight keeps flying on zeroed offsets and says nothing. Calibration is operator-
triggered over MAVLink and refuses while armed, so it cannot re-run on its own — which leaves
the reported reset cause as the only thing that would explain why the bias came back.

#### In-flight restart recovery is a separate project — 🧊 DEFERRED

Persisting prior state is the easy half, and it belongs in a `.noinit` SRAM section rather than
on the ESP32: a reset does not clear SRAM, so the marker is readable microseconds in, with no
FcLink handshake to wait on and no second MCU to depend on. A brownout deep enough to lose that
RAM took the ESP32 with it anyway — they share a battery.

The hard half is that the attitude is gone. Gyro bias is a second-order term — `Ahrs::bias_`
already estimates it in flight, and a few degrees of drift is nothing beside a quaternion reset
to identity. In free fall the accelerometer reads about zero g in every direction, so it cannot
say which way is up: rate can be held, level cannot be recovered.

Auto-arming also inverts #16's charter. A bench reset would spin props on the table unless a
trustworthy airborne test gates it, and sustained near-zero-g is about the only honest one — a
signal that only arrives once things have already gone wrong. Fixing what causes the reset is
worth more than recovering from it.

### #40 — Faults that survive the battery being pulled — 🟢 SUPPORTING

An IMU fault raised in flight is held in `Sentinel::imu_fault_latched_` and answered at the
disarm edge. That latch lives in SRAM, so a pilot who lands and pulls the battery — rather than
disarming and letting the board sit — takes the only record of the fault with them, and the next
boot is clean with no one the wiser. Every fault this board can defer has the same hole.

Car ECUs answer it with diagnostic trouble codes: a fault writes a code to non-volatile storage,
the lamp stays lit across power cycles, and the code clears only when a tool reads it and is told
to. The value is not the storage, it is the refusal to forget without someone acknowledging.

`.noinit` SRAM is the wrong home, and #21 reaches the opposite conclusion for a different case:
it survives a *reset*, which is what an in-flight watchdog restart needs, but not a power cycle,
which is the case here. The ESP32 has NVS, already owns the display that would carry the
indicator, and is the thing the operator connects to on the bench.

#### Shape

A fault code and the flight index it was raised in, sent over FcLink when Sentinel raises it and
again on handshake so a fault raised while the link was down is not lost. The ESP32 appends to a
small NVS ring — bounded, oldest dropped — and the UI shows a pending-fault indicator until it
is cleared. Clearing is explicit and confirmed on the page that shows the faults; nothing
clears on boot, on read, or on a good flight.

`ErrorCode` is already the shared vocabulary, so the wire carries a code the ESP32 can name
through `error_code.cpp` rather than a second enum invented for the purpose.

#### The boot policy stays with Sentinel

A stored fault must not become a thing that refuses to boot. The ESP32 records and reports; what
a fault *means* — halt, refuse to arm, or warn — stays where #16 put it, and the STM32 must come
up fully with a card full of history. Otherwise a stale code from a fixed problem grounds an
airworthy aircraft, which is the failure mode ECUs are most criticised for.

Pairs with #17: Doctor is the natural reader, and a fault log is the first thing it should
present. #36 carries the notification.


---

## Codebase and tooling

### #7 — A failsafe state — 🟢 SUPPORTING

The state machine has no state for a tripped failsafe, because every procedure is still
"disarm". #15 decides what a trip does, and #52 works out which states that needs.

### #18 — Flash the two firmwares as one thing — 🟢 SUPPORTING

The flash targets rebuild and rewrite everything every time, and when two images disagree
neither can say which of them is the stale one. The ESP32 owns both of the STM32's flash paths,
WiFi and the bridge's own USB — `Programmer` drives BOOT0 and the shared FcLink UART — so one
device deciding what to flash is mostly wiring parts that are already there.

- Read both build identities, compare against the build, skip what already matches. The ESP32
  has `kMavlinkFlightSwVersion` and `kMavlinkGitHashShort` from `generate_esp32_config.py`; the
  STM32 has no identity at all, which is also what #31 wants for the ULog header.
- Identity is information, not a gate. What it buys over the handshake's refusal is knowing
  which image to flash.

Refusing to arm on a protocol mismatch is Sentinel's call, not the flasher's — see #16.

### #19 — Give every SharedState field an owner the compiler knows about — 🟢 SUPPORTING

`SharedState` is a const-correct store, not an access-controlled one. Readers get `const &` so
they cannot mutate shared state, but fourteen of its sixteen setters are **public**, so the
pattern governs *how* a write happens and never *who* performs it. `UpdateRc` is meant to be
`RcReceiver`'s alone; nothing says so and nothing checks.

`armed_` and `failsafe_flags_` are the two with a real owner — private, with
`friend class Sentinel` — because a second writer there spins motors.

#### Shape

The passkey idiom, one key per producer:

```cpp
class GpsKey { friend class M10Service; GpsKey() = default; };
void UpdateGps(const GpsData &data, GpsKey) { gps_ = data; }
```

`UpdateGps` stays public, but only `M10Service` can construct the key, so only `M10Service` can
call it. An empty class is elided entirely — identical generated code, no RAM, no indirection
in the control path. Roughly twenty lines in `shared_state.hpp` plus a `{}` at each call site.

It also narrows what exists today: `friend class Sentinel` opens *every* private in
`SharedState` to Sentinel, and the `Icm42688p`/`Ahrs` friendship on the sample mailbox is the
same shape. A key exposes exactly one function.

#### Why not the alternatives

- **A lint** counting call sites per setter measures a proxy. The property wanted is exclusive
  ownership — not how often the owner writes, but whether anything else can. C++ states that
  directly, and a compile-time guarantee needs no exceptions file.
- **A writer handle** (`SharedState::GpsWriter`, constructed once and stored by its producer) is
  the most principled: it is the only option giving real least privilege, since services today
  hold a whole `SharedState *`. Costs a pointer per writer, makes the single-owner property
  runtime rather than compile-time, and touches every service's `Init`. Revisit it if the replay
  harness needs to substitute a producer — a handle is trivially redirectable, a passkey is not.

### #22 — Nothing finds dead code — 🟢 SUPPORTING

`-Wunused` fires only for internal-linkage functions, so an unused public header-inline accessor
is invisible to every build: no TU odr-uses it, so no TU emits it, so it costs no flash and
raises no warning, so nothing in the build can tell an accessor nobody calls from one every
caller needs. Three tools each catch part of the gap, and they do not overlap:

- `-Wl,--print-gc-sections` — ground truth on the binary. Finds unused data, vtables and
  transitively dead code. Blind to never-emitted inline functions, which is most of what
  accumulates here.
- `cppcheck --enable=unusedFunction` — source-level, so it does see inline accessors. Weak on
  virtual and function-pointer dispatch, and needs the whole program in one pass.
- clang `-Wunused-private-field` — the only one that finds unused *data members*. Needs nothing
  but `-fsyntax-only`.

All three stay advisory. A whole-program "unused" verdict is only as good as its view of the
callers, and a public API has callers no single-tree pass can see — a list to review, never a
build gate.

### #34 — Linter exceptions are scattered, and no rule can be silenced on one line — 🟢 SUPPORTING

Twenty scripts in `scripts/lint/` gate this repo, and the answer to "why does this file not
have to obey" is in a different place for each of them:

- **Two read an exceptions file, in two grammars that disagree.** `comment_exceptions.txt` takes
  `<path>:<rule>`. `forbidden_exceptions.txt` takes a bare substring matched against either
  `path:line:col` *or* the offending source text, so one entry can excuse a construct everywhere
  rather than excuse a file — and it arrives through a `--exceptions` flag rather than a fixed
  path. Both files are empty today, which is the only reason the difference has cost nothing yet.
- **Four carry the list as Python constants, and not in the same shape.**
  `check_timer_access.py`'s `ALLOWED` (four paths) and `check_license.py`'s `EXEMPT_PATTERNS`
  (ten globs) excuse a path outright; `check_singleton_style.py`'s `ALLOWED` maps a path to a
  *set of rule names*, so it excuses per rule, and `check_config_reach.py`'s `ALLOWED` keys on a
  qualified name. All four already demand a written reason per entry, in a comment — the
  discipline is right, the storage is wrong. Two more stores are not even called `ALLOWED`:
  `check_comments.py`'s `EXEMPT_PREFIXES` and `check_forbidden.py`'s `REACH_EXEMPT`.
- **The hook config is a third location, and it is honoured in only one of the two runs.**
  `.pre-commit-config.yaml` scopes each hook with `files:`/`exclude:` and a shared `&not_ours`
  anchor, while `.github/workflows/lint.yml` runs fifteen of them bare over the whole tree. An
  exclusion that lives only in the hook config is silently absent from CI.

`check_tidy.py`'s `EXCLUDED` and `check_error_codes.py`'s `EXCLUDE_PATHS` look like the same
thing and are not: one is the rule set itself, the other is the enum's own two definition files.
Neither is an exception and neither should move.

**Nothing has a per-line escape.** No script honours an inline suppression for its own rules.
The single mention of one is `check_comments.py`'s `NOLINT_RE`, which *polices* clang-tidy's
`NOLINT` — rejecting any that carries neither a check name nor a reason — rather than obeying it.
Silencing `check_forbidden` for one honest line means editing the script.

Shape: one loader shared by every script, one file format carrying path, rule and a mandatory
reason, plus an inline `// LINT(<rule>): <reason>` for the single-line case, held to the same
standard `check_comments.py` already imposes on `NOLINT`. Stale entries should fail — a rule that
quietly stopped applying is worse than one that was never written.

The counter-pressure is real and belongs in the design: fourteen of the twenty have no
exemption mechanism at all, and that is why they hold. A shared escape hatch makes suppression
cheap for rules that currently cost an argument, so the inline form has to name the rule and the
reason in the diff the reviewer reads, and the fourteen keep having no entries until something
real needs one.
