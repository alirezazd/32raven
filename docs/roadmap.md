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

Materials, boards and wiring have pages, and the firmware section covers toolchain,
configure and flash. Still to write: frame and motors, power, smoke test, sensors, RC link,
bench test, and first flight.

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

### #7 — Two leftovers from the state split — 🟢 SUPPORTING

- **Rename `IdleState`.** It runs the full flight cascade — the name is left over from the
  single-state design and misdescribes the code.
- **A failsafe state**, once #15 decides what a trip does.

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

### #10 — Derive the motor count from the airframe — 🧊 DEFERRED

The airframe is one physical fact declared in two places that cannot see each other: the ESP32
fixes `kMavlinkSysAutostart` at 4001 and hardcodes `MAV_TYPE_QUADROTOR`; the STM32 references
`QuadX::kFactors` directly in `multirotor_mixer.cpp`.

No damage today, because neither side can be pointed at anything but an X quad. It becomes real
the moment either gains a geometry selection — two sources for one truth, on two MCUs that flash
independently. One airframe choice in Kconfig, with motor count *and* geometry derived from it,
rather than a standalone motor-count integer that can disagree with the frame it describes.

Three things resist a motor count that is not 4:

- **TIM1 has four compare channels.** DShot is one burst-DMA transfer per bit, `DCR` configured
  with base `CCR1` and length 4, so all four motors are driven from one stream and stay
  perfectly synchronised. Six motors needs TIM8 and a second DMA stream, with both bursts
  started together — two frames arriving at different times reads as a yaw bias.
- **`MotorThrust = std::array<float, 4>`** threads through `Mix`, `MixOutput`,
  `EscService::WriteMotorsThrust`, and back into the rate controller's anti-windup.
- **The literal 4 is written five times** with nothing checking that they agree:
  `dshot_codec.hpp`, `esc_telemetry.hpp`, `dshot_tim1.hpp` (`kMotors`), and
  `multirotor_mixer.hpp` twice.

Fewer than four motors is cheap — leave the unused channels configured and idle. Deferred
because no non-quad airframe is planned; unifying the declaration is the half worth doing first
if the mixer is being touched anyway.

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

Both frame types are already listed in the `TODO(crsf)` at `crsf_link_service.hpp`.

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

Until then the mode is refused rather than flown. `EscService::CheckEscFirmware` panics with
`kEsc3dModeEnabled` when a motor reports it, because an ESC in 3D reads everything below half
throttle as reverse — so the forward-only values `ThrustToDshot` produces would drive that
motor backwards across the bottom half of its range, on an aircraft whose mixer believes it is
commanding lift.

### #15 — Nothing acts on a lost link — 🎯 CRITICAL

`kVehicleFailsafeFlagRcLoss` reaches the wire and nothing reads it. No code anywhere changes
behaviour when a link goes down.

The control loop reads `rc.roll_us`, `rc.throttle_us` and the rest straight out of `SharedState`
with no freshness test, so RC loss means the last stick values are held and flown indefinitely.
`states.cpp` says so where the cascade starts — *"No tx_online check yet — disarmed mixer +
disarmed ESC means worst case is harmlessly computing zeros from stale RC."* That is true only
because arming is currently reachable solely through `kPrivilegedArm` over FcLink.

**The tripwire: stick arming must not land before this does.** The moment a stick gesture can
arm, holding stale sticks stops being harmless.

#### Detection is a timestamp away

Freshness is derived at read time from `RcData::timestamp_us`, and each consumer picks its own
bound. `StatPublisher` uses 1.5 s because it only has to answer "is the GCS being shown a live
link". The control path needs its own age test against the same field, not a new detector.

Betaflight triggers at 150 ms and PX4 at 500 ms, and recovery wants the opposite hysteresis
from an indicator — Betaflight demands 1 s of clean data plus low throttle before handing
control back, where a display should be quick to recover and slow to drop.

A timeout is the only detector available: CRSF carries no receiver-asserted failsafe bit, and
ExpressLRS signals loss by *stopping* RC frames rather than flagging them.

The battery condition inherits the same rule: `BatteryData` carries its own stamp and its
current as an optional, and a failsafe that reads the voltage without checking the stamp
trusts a frozen number — the freshness test lands with the detector, not after it.

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
already on the blackboard. `kVehicleFailsafeFlagImu` is the one flag Sentinel raises, and
nothing reads it either — deciding what an IMU failsafe *does* in the air is this item's job.
`kVehicleFailsafeFlagGps` stays zero until something actually navigates by GPS.

This item is the *policy*: which conditions count, how fast, and what each one does. Where that
policy lives and what enforces it is #16.

### #16 — Sentinel, one owner for arming and failsafe authority — 🎯 CRITICAL

Sentinel is the single owner of the arm decision and of every failsafe condition #15 defines.
It owns the arm path and the IMU conditions. Three things are still outside it: the remaining
failsafe conditions, FcLink peer loss, and the watchdog.

#### The conditions still live in a telemetry builder

`StatPublisher::BuildSystemStatusMsg` derives GPS, battery and RC health from freshness, so
moving detection into Sentinel is *relocating* that computation rather than writing a second
copy of it. `Sentinel::Supervise` is the slot, and carries the `TODO(fc)` naming the three.

**It must live on the STM32**, because its whole purpose is to keep working when the ESP32 is
gone. That is also why FcLink peer loss is Sentinel's to detect on this side (#17 owns the
other): `FcLink` exposes `Poll` and the `Send*` helpers and nothing that notices silence, and
since arming currently lives behind FcLink, an ESP32 failure while armed removes the disarm
path entirely and leaves the cascade running on the last RC values.

#### What "high priority" means without an RTOS

The STM32 is a superloop, so priority is placement rather than a number: `Supervise` runs first
in `MainTick` and has to stay there, since a failsafe skipped under load is a failsafe that does
not exist. Any future work-shedding under load inherits that constraint rather than negotiating
with it.

The shape to keep aiming at is an interlock that makes a state impossible rather than one that
polls for it — the former costs nothing per pass, the latter costs something forever.

#### Sentinel should own the watchdog

`Wdg().Kick()` sits unconditional in the superloop in `main.cpp`, so it proves only that the
superloop iterates. A wedged control loop under a healthy superloop is still fed — at 2048 Hz
against a ~681 ms worst-case IWDG, that is well over a thousand missed control cycles that
nothing detects. Kicking only after Sentinel has confirmed the control-loop counter advanced
turns the watchdog from "the loop turns" into "the control loop is running".

Two constraints:

- **`EscBootloader` also kicks, deliberately.** During ESC flashing the control loop is not the
  liveness criterion, so the kick policy has to be mode-aware or that path resets mid-write.
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

It does not duplicate `StatPublisher`, which reports STM32-local health because only the STM32
can see it. Doctor adds what only the ESP32 can see — heap, task stack headroom at run time
rather than only through `esp32_stack_check.py` statically, WiFi, flash, transport link stats —
and correlates across FcLink. FreeRTOS makes it a task with a real priority, unlike #16.

#### The part worth doing before the service exists

`Mavlink::StartHeartbeatFrame` gates the armed state and the mode flags on `vehicle_fresh`
and the whole `MAV_STATE` ladder on `kMavlinkSystemStatusFreshMs`, so a silent STM32 reads as
`MAV_STATE_CRITICAL` rather than a stale vehicle. One read is deliberately ungated:
`vehicle_failsafe` tests `have_data` only, because a stale failsafe can only hold the state at
CRITICAL and a link that died with one raised is not the moment to stop saying so.

What is left is narrower than a service: nothing tells the GCS *why* the peer went quiet, and
#41 is the reason it goes quiet most often.

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
on the STM32 and Doctor takes the reporting: the fact joins the logger counters `StatPublisher`
already carries, and Doctor is what turns "mandatory at boot, absent now" into something a GCS
sees rather than a tone nobody is standing next to.

#### Constraint

Doctor reports to MAVLink and the logs, **not to the OLED**. The display stays a bench tool and
flight state stays off it.

### #18 — Flash the two firmwares as one thing — 🟢 SUPPORTING

Both MCUs compile `libs/message.hpp`, so the FcLink wire format is a contract between two
images that are flashed separately. When they disagree, `IsPayloadLengthValid` rejects the
mismatched frames and the link degrades silently — no panic, no log, just a stream that stops
arriving. Nothing in either firmware detects it, and nothing in the build system prevents it.

The only protection today is remembering to flash both. That held while the wire format was
stable; narrowing `RcChannelsMsg` from 36 bytes to 20 is the first change that would have
punished forgetting, and `static_assert`s catch only the half of it that lives inside one build.

#### What already exists

- **The ESP32 owns the STM32's only flash path.** `Programmer` drives BOOT0 and the shared
  FcLink UART, so "one device flashes both" is mostly wiring parts that are already there.
- **The ESP32 already has a version** — `kMavlinkFlightSwVersion` and `kMavlinkGitHashShort`,
  emitted by `generate_esp32_config.py`.
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
- If it becomes one addressed binary, order matters: the ESP32 last means a failure leaves a
  matched STM32 and a stale ESP32, which is recoverable over WiFi. The reverse can leave a
  half-flashed ESP32 that can no longer program the STM32 it was meant to fix.

Refusing to arm on a protocol mismatch is Sentinel's call, not the flasher's — see #16. The
flasher's job ends at making the mismatch visible and easy to fix on the bench.

Supporting rather than critical only because the manual practice works. It stops being enough
the moment someone other than the author flashes this aircraft, or a wire change lands
alongside a flight-day rebuild.

### #35 — Flash the STM32 over the cable, not the air — 🟢 SUPPORTING

`make flash-wifi-stm32` is the only way to program the flight computer, so the AP has to be up
and joined before the STM32 can be fixed — including when the thing being debugged *is* the
WiFi. The bridge already has a USB cable attached for `make flash-esp32`; the same cable should
carry a `make flash-stm32`.

Everything below the host leg is already transport-agnostic. `DfuState` and `ProgramState`
drive `Programmer`, which pulses `BOOT0`/`NRST` and speaks the ROM bootloader over USART1;
none of that knows or cares how the image arrived. Only the host-to-ESP32 hop is TCP, and the
states touch it through about a dozen `TcpServer` calls — `Poll`, `PopEvent`, `SendCtrlLine`,
`SendData`, `StartDownload`, `StopDownload`, `GetStatus`, `SetStatus`, the bridge pair and
`Stop`. A USB transport that produces the same event stream and accepts the same writes drops
in underneath without the states noticing.

Two things make it more than a rename:

- **One pipe, two channels.** TCP hands out a line-oriented ctrl socket and a binary data
  socket; USB-Serial-JTAG is a single byte stream, so the two have to be multiplexed and
  framed. `libs/message.hpp` already defines a framed packet with a length and a CRC, and both
  firmwares compile it — reusing that shape is cheaper than inventing a second one.
- **The console shares the pipe.** `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, so `ESP_LOG` output
  interleaves with whatever else writes there — `UsbCdcServer::Init` says as much, and MAVLink
  survives it only because pymavlink resynchronises on the `0xFD` magic byte. A firmware image
  needs the same resync property from its framing, or the console silenced for the duration of
  a transfer.

Host side, `tools/esp32_client.py` opens two sockets and resolves an IP; it would open one
serial port and demultiplex, with port discovery replacing `resolve_target_ip`. The existing
`flash`/`flash_esp` command shapes and the `BEGIN size=… crc=…` handshake stay as they are.

The one-USB-at-a-time rule in `docs/flashing.md` is unaffected: this uses the bridge's own
port, the one already carrying `make flash-esp32`, and never the flight computer's.

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

### #38 — Put the destructive pages behind a service menu — 🟢 SUPPORTING

DFU is one long-press from the operational cycle. `CycleOnButton` walks Serving, MavlinkWifi,
WifiLog, UsbLog, MavlinkUsb and back, with Dfu and EscConfig hanging off the long press, so
detaching the STM32's firmware is as reachable as switching telemetry transports. #32 adds
erasing the card to the same neighbourhood.

#32 already answers this for formatting alone — a hidden entry, a gesture that cannot be
stumbled into, documented in the handbook and never on the screen. The point here is that the
answer should be one gate rather than a second mechanism per dangerous page. Everything that
reprograms an MCU or destroys stored data lives behind it; the cycle carries only what an
operator uses in normal service.

#### The recovery path must not go behind the gate

`EnterRecoveryDfuMode` reaches DFU directly from the panic loop, not through the menu, and that
has to stay true. A gesture-gated DFU that a panicking board cannot reach turns a bricked STM32
into a cable job — the exact situation the WiFi DFU path exists to avoid.

Lands after #32, since formatting is the second occupant that makes a shared gate worth
building rather than a one-off.

### #39 — The LED says less than it has states — 🟢 SUPPORTING

Six pages, three signals, and no scheme tying them together.

| Page | Pattern |
| --- | --- |
| Serving | breathe, 3 s |
| Dfu | blink, 400 ms |
| EscConfig, UsbLog, WifiLog | blink, 800 ms — identical on all three |
| MavlinkWifi, MavlinkUsb | nothing set |
| Program | off |

So the LED distinguishes "a tool page" from DFU but not which tool page, and on the two MAVLink
pages it shows whatever the previous page happened to leave behind.

**A one-shot never gives the page back.** `SetPattern(..., repeat_count)` installs `kOffStep`
when the count exhausts rather than restoring the pattern underneath, so every transient use of
the LED permanently claims it. `mavlink_rx.cpp` fires a `kDoubleBlink` per received heartbeat,
which on a page with no pattern of its own reads as a link pulse — and on any page that has one
would silently end it. The mechanism wants a foreground/background split: a page installs
background, a transient plays over it and hands it back.

**The STM32 has no vocabulary at all.** `stm32/Drivers/led.hpp` exposes `Set(bool)` and nothing
else. That is fine while every annunciation goes to the ESP32's buzzer and screen, and stops
being fine the moment #28 lands: an arm refused on the RC path with the bridge dead has one
LED, no tone and no display, which is the silent refusal #28 names as its own failure mode.

Worth deciding what the LED *means* before adding patterns to it — page identity, link
liveness, or fault — because it currently attempts all three with no priority between them.

### #19 — Give every SharedState field an owner the compiler knows about — 🟢 SUPPORTING

`SharedState` is a const-correct store, not an access-controlled one. Readers get `const &` so
they cannot mutate shared state, but eleven of its thirteen setters are **public**, so the
pattern governs *how* a write happens and never *who* performs it. `UpdateEstimate` is meant to
be the AHRS path's alone and `UpdateRc` `RcReceiver`'s alone; nothing says so and nothing checks.

`armed_` and `failsafe_flags_` are the two with a real owner — private, with
`friend class Sentinel` — because a second writer there spins motors.

#### The failure it prevents

Two components each holding their own copy of one fact, kept in step only by every writer
remembering to update both. On `armed` that shape put the ESCs disarmed while the mixer kept
producing torque, and only `friend class Sentinel` closes it. Every other field is still open to
it, and the next occurrence will not announce itself either — the compiler has nothing to say
about a setter anyone may call.

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

### #20 — Sentinel watchdogs on the IMU and the control path — 🎯 CRITICAL

Sentinel weighs the IMU counters and answers per arm state. Every threshold it uses is asserted
rather than measured, and the failure that matters most — a control loop that has stopped
running — has no detector at all.

#### Nothing watches whether the control loop runs

`StatPublisher::BuildSystemStatusMsg` sets `msg.flags = kSystemStatusFlagLoopAlive`
unconditionally, and the ESP32 gates `MAV_STATE_CRITICAL` on that bit. A SystemStatus frame
needs only the main tick, and `msg.loop_counter` is the main tick's own count — so a wedged
control loop leaves every indicator on the ground green. The IWDG misses it too: the main loop
kicks it, and the main loop is fine, which is one concrete route to an in-flight reset (#21).

`Sentinel::RecoverStalledImu` covers the neighbouring failure, not this one — it watches
`ImuHealth::timestamp_us`, which the *sample interrupt* stamps. Liveness is the opposite case:
the interrupt publishing normally while the control loop fails to consume. It needs a stamp the
control loop writes, which is also what makes the flag mean its name, and what #16 needs before
the watchdog kick can mean anything.

#### Every threshold is a guess

| Threshold | Value | Guards |
| --- | --- | --- |
| `kLossPerSecThreshold` | 0.5% of gyro ODR, 3 consecutive seconds | unclaimed sample bursts |
| `overrun_threshold` | 3 path faults in 1 second | sample-path faults |
| `kImuStallTimeoutUs` | 20 ms | sample-path silence |

What the loss rule counts is real: `PENDSVSET` is a bit, not a queue, so two bursts published
inside one control tick collapse to one PendSV run and the older is dropped. That is the
measurement that says whether the cascade fits its budget — **and the loop was raised to
2048 Hz without it**, argued from the aliasing corner and the DShot budget rather than measured.

**The numbers cannot leave the board.** `ImuHealth` carries ten counters and every consumer
reduces them to a boolean — an LED blink, a health bit, a threshold comparison. `SystemStatusMsg`
has no field for any of them. An honest threshold needs at least `missed_samples` on the wire,
read across the four cases that load the loop: disarmed idle, armed at motor idle, throttle
sweeping, and every link streaming at once. Worth landing with #21 — both are `SystemStatusMsg`
additions, and one wire break costs one dual-flash.

#### The stall recovery cannot clear

A stall with `inflight_` stuck true — a DMA that started and never completed — survives
`RestartSampling`, because nothing clears it or the SPI driver's `busy_`. Aborting a live DMA is
real surgery and belongs here rather than in the driver.

### #21 — Know why the board restarted — 🟢 SUPPORTING

`RCC->CSR` records what caused the last reset — power-on, brownout, IWDG, window watchdog,
software, pin — and holds it until `RMVF` is written. Nothing reads it. So an in-flight
watchdog reset is indistinguishable from a cable glitch: the FC silently restarts and the GCS
sees the handshake replay. #20 names one concrete route to exactly that.

Capture it at boot, clear it, and carry it in `SystemStatusMsg`. Three lines, and it turns a
silent restart into a reported one. Worth landing alongside #20's counters, since both are
`SystemStatusMsg` additions and one wire break costs one dual-flash.

#### It is a prerequisite for gyro calibration, not a nicety

`Init` zeroes `OFFSET_USER` on every boot and nothing puts a value back, so #25's MAVLink
calibration work will add a trigger for `CalibrateGyro`. Running that after a *non*-power-on
reset either trips `kImuCalibrationMotionDetected` or writes the motion permanently into the
chip's offset registers. "Only calibrate on a power-on reset" is the check that prevents it,
and it needs the cause to exist first.

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
is cleared. Clearing is explicit and belongs behind #38's service menu; nothing clears on boot,
on read, or on a good flight.

`ErrorCode` is already the shared vocabulary, so the wire carries a code the ESP32 can name
through `error_code.cpp` rather than a second enum invented for the purpose.

#### The boot policy stays with Sentinel

A stored fault must not become a thing that refuses to boot. The ESP32 records and reports; what
a fault *means* — halt, refuse to arm, or warn — stays where #16 put it, and the STM32 must come
up fully with a card full of history. Otherwise a stale code from a fixed problem grounds an
airworthy aircraft, which is the failure mode ECUs are most criticised for.

Pairs with #17: Doctor is the natural reader, and a fault log is the first thing it should
present. #36 carries the notification.

### #41 — The radio goes dark on every bench page — 🎯 CRITICAL

The Telem UART is the aircraft's MAVLink link: `TelemUartServer` on GPIO20/21 at 57600, the
SiK default, brought up by `ServingState` which is where `main.cpp` starts the machine. The
WiFi and USB MAVLink pages are bench transports reached from the menu, not the vehicle's link.

Four states call `Mavlink().SetTelemetryLink(false)` in `OnEnter`. One of them has a reason.

| State | STM32 | FcLink | Telem UART | Wanted |
| --- | --- | --- | --- | --- |
| Dfu, waiting for a host | Idle, running | free | free | on |
| WifiLog, waiting for a host | Idle, running | free | free | on |
| EscConfig | suspended | MSP relay | MAVLink, full | not-ready |
| UsbLog (MSC) | suspended | grant only | free | on, not-ready |
| LogPull, transferring | Idle, running | saturated | free | narrowed |
| Program, flashing | ROM bootloader | held by Programmer | free | dark |

`DfuState` and `WifiLogState` never touch the flight controller — they start the network and
wait, indefinitely, while it runs normally. `ProgramState` is the only one with a physical
reason: BOOT0 is asserted, `Programmer` owns USART1, and no firmware is left to publish.
Recovery DFU is a seventh case, where the STM32 is halted in its panic loop and the ESP32
holds the only fact worth sending — which is #40's.

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

Two fields the STM32 fills with constants, and one whose health bits latch. #20 owns
`kSystemStatusFlagLoopAlive`; these are the rest.

| Field | Today | Wanted |
| --- | --- | --- |
| `sensor_health_flags` | gated on since-boot counters being zero | a windowed delta |
| `error_code` | `kOk`, always | Sentinel's latched fault code |
| `errors_count1..4` | four literal zeros in the pack call | four of the ten `ImuHealth` counters |

**The health bits latch red and never recover.** `path_faults`, `rx_dma_error_count` and
`uart_error_count` are cumulative since boot, so one transient fault ever leaves that sensor
unhealthy for the rest of the session while the freshness test beside it reads fine. The
windowed delta Sentinel already uses fixes it, with no wire change. It became easier to reach
once the path-fault panic left the driver: those faults now accumulate instead of halting.

**`error_code` has a producer for the first time.** A panic halts the board, so a running
board had nothing to report and `kOk` was defensible. `Sentinel::imu_fault_latched_` ends
that — an aircraft flying on a deferred fault holds a real code in a private member and
nothing carries it. `failsafe_flags` says something is wrong; this says which.

**The counters cannot leave the board**, which is #20's blocker for setting any threshold
honestly. Four of the ten fit `errors_count1..4`, and `missed_samples` is the one #20 needs
first. A wire change, so it wants #21's reset cause and #41's `kNotReady` in the same flash.

Reading any of it needs #41 first.

### #43 — A current reading is trusted as far as it can saturate an int — 🟢 SUPPORTING

Blocked on a real sensor reaching `PC1`. When one does: `STM32_BATTERY_CAPACITY_MAH` and a
max-current knob inside the current-monitoring menu, a plausibility bound in the driver where
the negative clamp and deadband already live (a reading above what the pack can deliver is
provably false), and an over-current condition feeding #15's battery flag. The bound cannot
detect an absent sensor — a floating pin reads amps that fit any 6S budget — which is what
the build-time knob is for.

### #22 — Nothing finds dead code — 🟢 SUPPORTING

`-Wunused` fires only for internal-linkage functions, so an unused public header-inline accessor
is invisible to every build: no TU odr-uses it, so no TU emits it, so it costs no flash and
raises no warning. `Ahrs::Current()` survived that way until it was found by reading; so did
the CRSF direct-send trio, private and unreachable for months after StatPublisher took their
job. Three tools each catch part of the gap, and they do not overlap:

- `-Wl,--print-gc-sections` — ground truth on the binary. Finds unused data, vtables and
  transitively dead code. Blind to never-emitted inline functions, which is most of what
  accumulates here.
- `cppcheck --enable=unusedFunction` — source-level, so it does see inline accessors. Weak on
  virtual and function-pointer dispatch, and needs the whole program in one pass.
- clang `-Wunused-private-field` — the only one that finds unused *data members*. Needs nothing
  but `-fsyntax-only`.

All three stay advisory. The confidential SIL consumes this repo's public API, so a whole-program
"unused" verdict from inside the repo is the exact false positive that rule exists to catch —
a list to review, never a build gate.

### #34 — Linter exceptions are scattered, and no rule can be silenced on one line — 🟢 SUPPORTING

Seventeen scripts in `scripts/lint/` gate this repo, and the answer to "why does this file not
have to obey" is in a different place for each of them:

- **Two read an exceptions file, in two grammars that disagree.** `comment_exceptions.txt` takes
  `<path>:<rule>`. `forbidden_exceptions.txt` takes a bare substring matched against either
  `path:line:col` *or* the offending source text, so one entry can excuse a construct everywhere
  rather than excuse a file — and it arrives through a `--exceptions` flag rather than a fixed
  path. Both files are empty today, which is the only reason the difference has cost nothing yet.
- **Two carry the list as Python constants.** `check_timer_access.py`'s `ALLOWED` (three paths)
  and `check_license.py`'s `EXEMPT_PATTERNS` (fourteen globs). Both already demand a written
  reason per entry, in a comment — the discipline is right, the storage is wrong.
- **The hook config is a third location, and it is honoured in only one of the two runs.**
  `.pre-commit-config.yaml` scopes each hook with `files:`/`exclude:` and a shared `&not_ours`
  anchor, while `.github/workflows/lint.yml` runs every script bare over the whole tree. An
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

The counter-pressure is real and belongs in the design: thirteen of the seventeen have no
exemption mechanism at all, and that is why they hold. A shared escape hatch makes suppression
cheap for rules that currently cost an argument, so the inline form has to name the rule and the
reason in the diff the reviewer reads, and the thirteen keep having no entries until something
real needs one.

### #23 — Nothing filters the gyro in software — 🟢 SUPPORTING

The entire gyro path is `gyro_accum / batch.count` in `Ahrs::Process`. **No software filter
exists at all** — no notch, no lowpass, no RPM tracking, and no biquad anywhere in the repo.
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

### #24 — Bidirectional DShot — 🟢 SUPPORTING

#23's RPM filter needs motor frequency every loop, and the KISS serial telemetry we poll cannot
give it: one motor at a time, tens of Hz, so a notch chasing a throttle punch would sit on
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
  out, no hardware. Pure is what lets the SIL test 400 lines of bit-twiddling off-target with
  canned buffers.
- **Publish → the driver, from the RX-complete ISR**, exactly as `Icm42688p` parses and
  publishes `ImuHealth` from its DMA-done ISR. Into a **new** blackboard struct —
  `MotorRpmData { timestamp_us, erpm[4], valid_mask, crc_error_count }` — not into
  `EscTelemetryData::rpm`, which has one writer (#19) and keeps it. KISS telemetry stays the
  volts/amps/temp source and its slow rpm becomes the cross-check on the fast one.
- **Consumers read the blackboard, nothing holds a `DShotTim1 *`.** The RPM filter (#23) in
  PendSV, the desync detector (#12) in Sentinel, `stat_publisher` if the wire wants it. The
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
puts it in `libs/` rather than under `stm32/`, where the SIL reaches it without dragging in
STM32 headers to feed it canned sample buffers.

Worth noting on the way past: `DShotCodec` is *not* pure today. `dshot_codec.cpp` includes
`dshot_tim1.hpp`, so the codec already reaches into the driver. That coupling is worth undoing
whether or not bidir ever lands.

It pays for itself twice. Besides #23, per-motor eRPM every loop is the desync detector #12 is
looking for.

### #25 — Calibration is stored, never applied, and would reset the board — 🎯 CRITICAL

Two unrelated halves, both armed the moment a MAVLink trigger is wired.

**Accel calibration does nothing.** `accel_calibration_` is loaded from EE at `Init` and written
back by `SaveAccelCalibration`, and nothing on the sample path reads it. It round-trips through
storage and never reaches a sample. The `EeConfigStorage` path around it is correct end to end —
the value simply has no consumer. Note there is no longer a float conversion inside the driver
to fold it into: the burst leaves in counts, so applying it means either an integer offset in
`MapAxes` or a correction in the estimator, and the two differ in whether the log carries a
corrected signal.

**Gyro calibration would trip the watchdog.** `CalibrateGyro` collects for `gyro_duration_s` of
2 with a `gyro_timeout_s` ceiling of 5, and there is no `Watchdog::Kick` anywhere in that
driver. The IWDG's worst-case period is ~681 ms, so the call resets the board three times over
— seven at the timeout. It has no caller today, which is the only reason it has never fired.

The gyro half is incomplete beyond that: `ClearUserOffsets()` wipes the chip's offsets every
boot, there is no EE record for them the way there is for accel, and #21 explains why the
trigger has to check the reset cause as well.

**The trigger is a decision, not just missing plumbing.** Operator-triggered over MAVLink is one
shape, automatic at boot is the other, and they need different things underneath:

- *Recalibrate every boot, persist nothing.* Consistent with the `ClearUserOffsets()` that
  already runs unconditionally, needs no EE record, and requires the craft still at power-on.
- *Calibrate on command, persist, restore at boot.* Needs a gyro record beside
  `ImuAccelCalibration`, and a reason for `ClearUserOffsets()` to stop wiping.

Today's code is neither — it wipes at boot and stores nothing.

Two things gate that choice rather than follow from it. The AHRS already carries a Mahony PI
bias term (`ki_bias`), and it has never executed: the accel-trust band is +/-0.020 g full and
+/-0.050 g zero, so a 4 g reading held the gate shut for the life of the board. With the
sensitivities corrected it runs, and what `bias_` converges to decides whether a hardware offset
is needed at all -- the residual to beat is 0.910 dps. Second, automation changes what motion
should do: `Panic(kImuCalibrationMotionDetected)` is defensible for an operator-triggered bench
step and hostile as a boot step that bricks the aircraft because someone leaned on the bench.
The stillness gate itself -- 64 raw counts, scale-invariant, so the sensitivity fix did not move
it -- has never been measured against a still board.

### #26 — Blackbox logging — 🟢 SUPPORTING

What the bench pass never reached is the two retrieval paths: the `SdCard` menu entry mounting
on a PC over MSC, and `tools/pull_logs.py` returning a byte-identical copy over WiFi. Both are
still unexercised.

Content gaps are #31 and #33; a card that fills is #37; formatting is #32.

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
  operation, so it belongs in the `SdCard` state after `LogService::ReleaseCard()`.
- **Bounded and fed.** A FAT32 mkfs writes both FAT copies — several MB on a large card. The
  blocking waits in `Sdio` already kick the watchdog, but `f_mkfs` itself must be checked
  against the ~700 ms window rather than assumed to fit.

Worth doing when the SD path has flown and its failure modes are known, not before: the whole
argument for a hidden format is that the operator understands what they are erasing, and right
now nobody has lost a log yet.

### #37 — Reuse the card when it fills, rather than refusing to boot — 🟢 SUPPORTING

A full card is not a fault, it is where the feature ends up. Every flight consumes one whole
preallocation whatever its length, so a 30-second hover costs the same 256 MB as a full pack:
124 flights on a 32 GB card, 62 on a 16 GB one. That is a season, not a lifetime.

What happens at the end of it is a panic. `f_expand` returns `FR_DENIED`, `PrepareNextFile`
raises `kSdCardFull`, and because that call sits in `Init` as well as `StopFlight`, the board
stops booting. The card filling on the last flight of a day panics *at disarm, after landing*,
and the vehicle then refuses to start until a PC has been found.

#### The ring is cheap here

`PrepareNextFile` already walks the root for `LOGnnnnn.ULG` tracking the highest index; the
lowest comes out of the same loop. `f_unlink` the oldest, retry `f_expand`. Uniform file sizes
are what make this trivial — one deletion always frees exactly one preallocation, so none of
the fragmentation reasoning that usually makes ring recording awkward applies.

#### It is a knob, and the default keeps today's behaviour

Overwriting flight data has to be something the operator chose, on #32's reasoning: the card
that fills is also the card holding the incident nobody has looked at yet. So a Kconfig knob,
default off.

- **Off:** `kSdCardFull` panics, exactly as now.
- **On:** the oldest log is reclaimed and the condition is announced instead — which is what
  makes this wait for #36. A panic is a poor way to say "the card wrapped"; a notification at
  startup is the right one, and before #36 there is no way to say it that does not need a
  cable.
- **Never the current flight's own log.** The file `StartFlight` is recording into is the one
  the operator most likely wants, so the victim is the oldest *closed* log, and a card holding
  only one log is full rather than wrapping.

Pairs with #32: both are card-owning operations that destroy data deliberately, and both are
worth doing only once the SD path has flown and somebody has an opinion about which logs they
would rather lose.

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
| `ImuHealth` | `last_bad_header` |

**The estimate is not recorded at all.** The raw FIFO topics carry the full-rate truth, so what
is missing is PX4's pair, through the scheduler rather than pushed: `vehicle_angular_velocity`
at 20 ms and `vehicle_attitude` at 50 ms, in PX4's field order and units. Both live in
`msg/versioned/` with `MESSAGE_VERSION = 0`, so the shape has to be copied deliberately rather
than approximated — and ours would be the burst mean, where PX4's is calibration-corrected,
notch/LPF filtered and EKF-bias-subtracted, which the record cannot claim to be.

**ESC telemetry is recorded not at all, deliberately.** `STM32_LOG_TOPIC_ESC_TELEMETRY_PERIOD_MS`
defaults to 0. The four motors answer their own requests and carry their own timestamps, so a
record with a single timestamp dates itself by whichever motor happened to be freshest, which
the reader cannot identify — and a motor falling silent has to be inferred from `valid_mask`
and a frozen RPM rather than seen. A log that quietly misattributes is worse than one that
says nothing, so the topic stays dark until the record shape is decided rather than inherited:
per-motor timestamps, the six bus counters (`crc_error_count`, `uart_error_count`,
`rx_dma_error_count`, `rx_drop_bytes`, `frame_count`, `unassigned_frame_count` — richer than
PX4's single per-ESC `esc_errorcount`), `consumption_mah` and `electrical_rpm`. Roughly 70 →
134 bytes and ~3 KB/s, so cost is not what is holding it.

**`error_code` is absent because nothing produces it**, not because the logger skips it: it is
hardcoded to `kOk` in `StatPublisher`, so recording it today would write a constant.
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
- **PX4's fault taxonomy is coarser than ours.** `true_overruns`, `dma_start_fails`,
  `spi_errors` and `parse_fails` all collapse into one `gyro_error_count`.

**Clipping is the exception, and wants doing whatever the topic ends up called.** A sample
pinned at the 20-bit rail is indistinguishable from a real reading once it is in the log, and
+/-16 g is reachable on a quad. `accel_clipping[3]` / `gyro_clipping[3]` are per-axis counts of
exactly that, and nothing here detects it -- `invalid_samples` counts the chip's no-fresh-data
sentinel, which is a different thing.

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

### #28 — The vehicle cannot be armed without the ESP32 — 🎯 CRITICAL

`CommandHandler::OnPrivilegedArm` is the only caller of `Sentinel::RequestArm` on the board, and
it is reachable only by `kPrivilegedArm` over FcLink. So arming *and disarming* both require the
second MCU alive and a GCS talking to it. An ESP32 failure while armed removes the disarm path
entirely and leaves the cascade running on the last RC values.

An arm switch is the fix, and Sentinel is already shaped for it — `RequestArm` is the single
entry point, so this is a second caller, not a second mechanism.

#### The switch is the easy part

`RcReceiver::Config` carries roll, pitch, yaw and throttle and nothing else, so an aux channel
needs a Kconfig knob, a calibration schema slot, and a wider `RcMapConfigMsg` — a wire change,
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

### #30 — Name the IMU orientations rather than configuring a triple — 🟢 SUPPORTING

`axis_map` is hardcoded identity in `stm32_config.hpp.j2`, with a comment saying to promote it
to Kconfig when a board rotates the chip. Promote it as a **named choice**, not as the signed
permutation the struct stores.

A signed permutation has 6 orderings × 8 sign combinations = 48 settings, and only 24 are
rotations. The other 24 have determinant −1: reflections no rigid mount can produce.
`AxisMapIsPermutation` does not catch them — it asserts the ordering is a permutation of
{0,1,2} and never looks at the signs — so `x_from=1, y_from=0, z_from=2` all-positive passes,
swaps X and Y, and hands the AHRS a left-handed frame.

Betaflight's set is the right size: 8 orientations, 4 yaw × {upright, flipped}
(`common/sensor_alignment.h`), plus a sentinel for the driver default and one custom escape.
It covers every mount anyone builds on a board they designed, stays a signed permutation so
`MapAxes` remains a table lookup, and structurally cannot express a reflection. PX4 carries
41 rotations behind an Euler table and a DCM multiply, including 45-degree steps and one
`ROTATION_ROLL_90_PITCH_68_YAW_293`, because it runs on airframes somebody else laid out. That
generality is a liability here, not a feature.

Shape: a Kconfig choice, expanded by the generator into the permutation and signs the driver
already consumes, with a determinant check in the generator so a bad expansion fails the build
instead of the flight. Nothing in the control path changes.

**The calibration hazard is handled but stays load-bearing.** `OFFSET_USER` is per chip axis, so
`CalibrateGyro` runs its body-frame mean back through `Icm42688p::ChipFromBody` before writing.
That inversion is exact only because the map is a signed permutation — one chip axis per body
axis — and a general rotation would need a transpose instead. The write is permanent, silent
when wrong, and shows up as drift on an axis that was never calibrated, so any change to how the
map is expressed has to keep `ChipFromBody` its exact inverse. Naming the orientations narrows
that risk rather than removing it.

Pairs with #25: a rotated mount invalidates the stored accel calibration as well, so the two
land together or not at all.
