#!/usr/bin/env python3
"""Raven Dashboard - Real-time telemetry monitor for 32Raven."""

# /// script
# dependencies = [
#     "textual>=0.47.1",
#     "textual_plotext>=0.1.0",
# ]
# ///

from __future__ import annotations

import argparse
import asyncio
import time
from collections import deque
from typing import Dict, Optional, Tuple

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.reactive import reactive
from textual.widgets import Footer, Header, Input, Label, Log, Static
from textual_plotext import PlotextPlot

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CTRL_PORT = 9000
DATA_PORT = 9001
PLOT_HISTORY = 200  # samples to display
PLOT_FPS = 10       # plot refresh rate


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def parse_kv(line: str, prefix: str) -> Dict[str, str]:
    """Parse 'PREFIX: k1=v1 k2=v2 ...' into a dict."""
    body = line[len(prefix):].strip() if line.startswith(prefix) else line
    out: Dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            out[k] = v
    return out


# ---------------------------------------------------------------------------
# Status Bar
# ---------------------------------------------------------------------------
class StatusBar(Static):
    """Connection status + live packet rate."""

    status = reactive("Disconnected")
    color = reactive("red")
    imu_hz = reactive(0.0)
    gps_hz = reactive(0.0)

    def render(self) -> str:
        rates = f"  IMU [cyan]{self.imu_hz:.0f}[/] Hz  GPS [cyan]{self.gps_hz:.0f}[/] Hz"
        return f"[bold {self.color}]● {self.status}[/]{rates}"


# ---------------------------------------------------------------------------
# GPS Widget
# ---------------------------------------------------------------------------
class GpsWidget(Static):
    """Compact GPS telemetry display."""

    fix = reactive(0)
    sats = reactive(0)
    utc_time = reactive("--:--:--")
    lat = reactive(0)
    lon = reactive(0)
    alt = reactive(0)
    vel = reactive(0)
    hdg = reactive(0)
    h_acc = reactive(0)
    drift = reactive(0)
    synced = reactive(False)

    def render(self) -> str:
        fix_s = f"[bold green]{self.fix}D[/]" if self.fix > 0 else "[red]No Fix[/]"
        sync_s = "[green]●[/]" if self.synced else "[red]●[/]"
        return (
            f"[b]GPS[/b]  {fix_s}  {self.sats} sats  {self.utc_time}\n"
            f"  {self.lat / 1e7:+.6f}°  {self.lon / 1e7:+.6f}°  "
            f"Alt {self.alt / 1000:.1f}m\n"
            f"  Vel {self.vel}cm/s  Hdg {self.hdg / 100:.0f}°  "
            f"HAcc {self.h_acc / 1000:.1f}m  "
            f"Sync {sync_s} {self.drift}µs"
        )


# ---------------------------------------------------------------------------
# RC Widget
# ---------------------------------------------------------------------------
class RcWidget(Static):
    """RC channel display."""

    ch_a = reactive(1500)
    ch_e = reactive(1500)
    ch_t = reactive(1500)
    ch_r = reactive(1500)

    def render(self) -> str:
        def bar(val: int) -> str:
            pct = max(0, min(100, (val - 1000) * 100 // 1000))
            filled = pct // 5
            return f"[cyan]{'█' * filled}{'░' * (20 - filled)}[/] {val}"

        return (
            f"[b]RC Channels[/b]\n"
            f"  A {bar(self.ch_a)}  E {bar(self.ch_e)}\n"
            f"  T {bar(self.ch_t)}  R {bar(self.ch_r)}"
        )


# ---------------------------------------------------------------------------
# IMU Text Widget
# ---------------------------------------------------------------------------
class ImuTextWidget(Static):
    """Numeric IMU readout."""

    ax = reactive(0.0)
    ay = reactive(0.0)
    az = reactive(0.0)
    gx = reactive(0.0)
    gy = reactive(0.0)
    gz = reactive(0.0)

    def render(self) -> str:
        return (
            f"[b]IMU[/b]\n"
            f"  [bold cyan]Accel[/] "
            f"X[red]{self.ax:+8.2f}[/]  "
            f"Y[green]{self.ay:+8.2f}[/]  "
            f"Z[blue]{self.az:+8.2f}[/]  m/s²\n"
            f"  [bold cyan]Gyro [/] "
            f"X[red]{self.gx:+8.2f}[/]  "
            f"Y[green]{self.gy:+8.2f}[/]  "
            f"Z[blue]{self.gz:+8.2f}[/]  rad/s"
        )


# ---------------------------------------------------------------------------
# Combined 3-Axis Plot
# ---------------------------------------------------------------------------
class TriAxisPlot(Static):
    """Single plot with three overlaid X/Y/Z traces."""

    LABELS = ("X", "Y", "Z")
    COLORS = ("red", "green", "blue")

    def __init__(
        self,
        title: str = "Sensor",
        y_range: Tuple[float, float] = (-1.0, 1.0),
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._title = title
        self._y_range = y_range
        self._bufs: list[deque] = [
            deque([0.0] * PLOT_HISTORY, maxlen=PLOT_HISTORY) for _ in range(3)
        ]
        self._sample_idx = 0
        self._dirty = False

    def compose(self) -> ComposeResult:
        yield PlotextPlot(id=f"{self.id}_plot")

    def on_mount(self) -> None:
        self._plot = self.query_one(f"#{self.id}_plot", PlotextPlot)
        self._plot.plt.theme("dark")
        self._plot.plt.title(self._title)
        self._plot.plt.ylim(*self._y_range)
        self.set_interval(1 / PLOT_FPS, self._refresh)

    def push_sample(self, x: float, y: float, z: float) -> None:
        for buf, val in zip(self._bufs, (x, y, z)):
            buf.append(val)
        self._sample_idx += 1
        self._dirty = True

    def _refresh(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        plt = self._plot.plt
        plt.clear_data()
        start = self._sample_idx
        xs = list(range(start, start + PLOT_HISTORY))
        for i in range(3):
            plt.plot(
                xs,
                list(self._bufs[i]),
                label=self.LABELS[i],
                color=self.COLORS[i],
            )
        self._plot.refresh()


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
class DashboardApp(App):
    """32Raven Telemetry Dashboard."""

    TITLE = "32Raven Dashboard"

    CSS = """
    Screen {
        layout: grid;
        grid-size: 2;
        grid-columns: 1fr 2fr;
    }

    /* Left pane: logs */
    #left-pane {
        height: 100%;
        border: solid $primary;
    }
    #left-pane Label {
        text-style: bold;
        padding: 0 1;
        color: $text;
    }
    #log {
        scrollbar-size: 1 1;
    }
    Input {
        dock: bottom;
    }

    /* Right pane: telemetry */
    #right-pane {
        height: 100%;
    }

    /* Top telemetry strip */
    #telem-strip {
        height: auto;
        max-height: 12;
        border-bottom: solid $primary;
    }
    #gps {
        width: 2fr;
        padding: 0 1;
        border-right: solid $primary;
    }
    #rc {
        width: 1fr;
        padding: 0 1;
        border-right: solid $primary;
    }
    #imu_text {
        width: 1fr;
        padding: 0 1;
    }

    /* Plot area */
    #plot-area {
        height: 1fr;
    }
    #accel_plot {
        height: 1fr;
        border-bottom: solid $surface-darken-2;
    }
    #gyro_plot {
        height: 1fr;
    }
    #accel_plot PlotextPlot,
    #gyro_plot PlotextPlot {
        height: 100%;
    }

    /* Status bar */
    #status {
        height: 1;
        dock: bottom;
        background: $surface;
        padding: 0 1;
    }
    """

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("c", "connect", "Connect"),
        ("d", "disconnect", "Disconnect"),
        ("l", "clear_log", "Clear Log"),
    ]

    def __init__(self, ip: str = "192.168.4.1"):
        super().__init__()
        self.target_ip = ip
        self._writer: Optional[asyncio.StreamWriter] = None
        self._reader: Optional[asyncio.StreamReader] = None
        self._ctrl_writer: Optional[asyncio.StreamWriter] = None
        self._ctrl_reader: Optional[asyncio.StreamReader] = None
        self._connected = False
        # Rate counters
        self._imu_count = 0
        self._gps_count = 0
        self._rate_ts = time.monotonic()

    # ── Layout ────────────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header()

        with Vertical(id="left-pane"):
            yield Label("System Log")
            yield Log(id="log")
            yield Input(placeholder="Send command…", id="input")

        with Vertical(id="right-pane"):
            with Horizontal(id="telem-strip"):
                yield GpsWidget(id="gps")
                yield RcWidget(id="rc")
                yield ImuTextWidget(id="imu_text")

            with Vertical(id="plot-area"):
                yield TriAxisPlot(
                    title="Accelerometer  (m/s²)",
                    y_range=(-20.0, 20.0),
                    id="accel_plot",
                )
                yield TriAxisPlot(
                    title="Gyroscope  (rad/s)",
                    y_range=(-40.0, 40.0),
                    id="gyro_plot",
                )

            yield StatusBar(id="status")

        yield Footer()

    # ── Lifecycle ─────────────────────────────────────────────────────────

    async def on_mount(self) -> None:
        self.log_msg(f"Target: {self.target_ip}")
        self.set_interval(1.0, self._update_rates)
        if self.target_ip:
            asyncio.create_task(self._connect_and_monitor())

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        cmd = event.value.strip()
        if cmd and self._ctrl_writer:
            self._ctrl_writer.write(f"{cmd}\n".encode())
            await self._ctrl_writer.drain()
            self.log_msg(f"> {cmd}")
            event.input.value = ""

    # ── Actions ───────────────────────────────────────────────────────────

    async def action_connect(self) -> None:
        if not self._connected:
            asyncio.create_task(self._connect_and_monitor())

    async def action_disconnect(self) -> None:
        if self._connected:
            await self._disconnect()

    async def action_clear_log(self) -> None:
        self.query_one("#log", Log).clear()

    # ── Connection ────────────────────────────────────────────────────────

    def log_msg(self, msg: str) -> None:
        self.query_one("#log", Log).write(msg)

    def _set_status(self, text: str, color: str) -> None:
        bar = self.query_one("#status", StatusBar)
        bar.status = text
        bar.color = color

    async def _disconnect(self) -> None:
        self._connected = False
        for w in (self._writer, self._ctrl_writer):
            if w:
                w.close()
                try:
                    await w.wait_closed()
                except Exception:
                    pass
        self._writer = self._reader = None
        self._ctrl_writer = self._ctrl_reader = None
        self._set_status("Disconnected", "red")
        self.log_msg("Disconnected.")

    async def _connect_and_monitor(self) -> None:
        if self._connected:
            return

        self._set_status("Connecting…", "yellow")

        try:
            self.log_msg(f"Ctrl → {self.target_ip}:{CTRL_PORT}")
            self._ctrl_reader, self._ctrl_writer = await asyncio.open_connection(
                self.target_ip, CTRL_PORT
            )

            self.log_msg("BRIDGE mode…")
            self._ctrl_writer.write(b"BRIDGE\n")
            await self._ctrl_writer.drain()

            resp = await self._ctrl_reader.readuntil(b"\n")
            if b"OK" not in resp:
                self.log_msg(f"Bridge failed: {resp}")
                await self._disconnect()
                return

            self.log_msg(f"Data → {self.target_ip}:{DATA_PORT}")
            self._reader, self._writer = await asyncio.open_connection(
                self.target_ip, DATA_PORT
            )

            self._connected = True
            self._set_status("Connected", "green")
            self.log_msg("Monitor active.")

            while self._connected:
                data = await self._reader.read(4096)
                if not data:
                    break
                text = data.decode("utf-8", errors="ignore")
                for line in text.split("\n"):
                    line = line.strip()
                    if line:
                        self._process_line(line)

        except Exception as exc:
            self.log_msg(f"Error: {exc}")

        await self._disconnect()

    # ── Telemetry Dispatch ────────────────────────────────────────────────

    def _process_line(self, line: str) -> None:
        if line.startswith("GPS:"):
            self._on_gps(line)
        elif line.startswith("IMU:"):
            self._on_imu(line)
        elif line.startswith("RC") and "A=" in line:
            self._on_rc(line)
        elif line.startswith("[TimeSync]"):
            self._on_timesync(line)
        elif any(c.isalnum() for c in line):
            self.log_msg(line)

    # ── GPS ───────────────────────────────────────────────────────────────

    def _on_gps(self, line: str) -> None:
        self._gps_count += 1
        kv = parse_kv(line, "GPS:")
        w = self.query_one("#gps", GpsWidget)
        try:
            if "fix" in kv:
                w.fix = int(kv["fix"])
            if "sats" in kv:
                w.sats = int(kv["sats"])
            if "lat" in kv:
                w.lat = int(kv["lat"])
            if "lon" in kv:
                w.lon = int(kv["lon"])
            if "alt" in kv:
                w.alt = int(kv["alt"])
            if "vel" in kv:
                w.vel = int(kv["vel"])
            if "hdg" in kv:
                w.hdg = int(kv["hdg"])
            if "hac" in kv:
                w.h_acc = int(kv["hac"])
            if "t" in kv:
                w.utc_time = kv["t"]
        except (ValueError, KeyError):
            pass

    # ── RC ────────────────────────────────────────────────────────────────

    def _on_rc(self, line: str) -> None:
        kv = parse_kv(line, "RC:")
        w = self.query_one("#rc", RcWidget)
        try:
            for ch, attr in (("A", "ch_a"), ("E", "ch_e"), ("T", "ch_t"), ("R", "ch_r")):
                if ch in kv:
                    setattr(w, attr, int(kv[ch]))
        except (ValueError, KeyError):
            pass

    # ── TimeSync ──────────────────────────────────────────────────────────

    def _on_timesync(self, line: str) -> None:
        kv = parse_kv(line, "[TimeSync]")
        w = self.query_one("#gps", GpsWidget)
        try:
            if "drift" in kv:
                w.drift = int(kv["drift"])
            if "synced" in kv:
                w.synced = kv["synced"] == "1"
        except (ValueError, KeyError):
            pass

    # ── IMU ───────────────────────────────────────────────────────────────

    def _on_imu(self, line: str) -> None:
        self._imu_count += 1
        kv = parse_kv(line, "IMU:")
        try:
            ax = float(kv.get("ax", 0))
            ay = float(kv.get("ay", 0))
            az = float(kv.get("az", 0))
            gx = float(kv.get("gx", 0))
            gy = float(kv.get("gy", 0))
            gz = float(kv.get("gz", 0))
        except ValueError:
            return

        imu = self.query_one("#imu_text", ImuTextWidget)
        imu.ax, imu.ay, imu.az = ax, ay, az
        imu.gx, imu.gy, imu.gz = gx, gy, gz

        self.query_one("#accel_plot", TriAxisPlot).push_sample(ax, ay, az)
        self.query_one("#gyro_plot", TriAxisPlot).push_sample(gx, gy, gz)

    # ── Rate Counter ──────────────────────────────────────────────────────

    def _update_rates(self) -> None:
        now = time.monotonic()
        dt = now - self._rate_ts
        if dt > 0:
            bar = self.query_one("#status", StatusBar)
            bar.imu_hz = self._imu_count / dt
            bar.gps_hz = self._gps_count / dt
        self._imu_count = 0
        self._gps_count = 0
        self._rate_ts = now


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="32Raven Dashboard")
    parser.add_argument("ip", nargs="?", default="192.168.4.1", help="ESP32 IP")
    args = parser.parse_args()
    DashboardApp(ip=args.ip).run()


if __name__ == "__main__":
    main()
