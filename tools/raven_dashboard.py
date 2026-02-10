#!/usr/bin/env python3
"""Raven Dashboard - Real-time telemetry monitor for 32raven."""

# /// script
# dependencies = [
#     "textual>=0.47.1",
#     "textual_plotext>=0.1.0",
# ]
# ///

import argparse
import asyncio
import random

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.reactive import reactive
from textual.widgets import Footer, Header, Input, Label, Log, Static
from textual_plotext import PlotextPlot

# Constants
CTRL_PORT = 9000
DATA_PORT = 9001


class ConnectionStatus(Static):
    """Displays connection status."""

    status = reactive("Disconnected")
    color = reactive("red")

    def render(self) -> str:
        return f"[bold {self.color}]{self.status}[/]"


class GpsLeftCol(Static):
    """GPS Left Column: Fix, UTC, Sync."""

    fix = reactive(0)
    sats = reactive(0)
    utc_time = reactive("--:--:--")
    drift = reactive(0)
    synced = reactive(False)

    def render(self) -> str:
        fix_str = f"[bold green]{self.fix}[/]" if self.fix > 0 else "[red]No Fix[/]"
        sync_icon = "✅" if self.synced else "❌"
        return (
            f"[i]Fix:[/]  {fix_str} ({self.sats} sats)\n"
            f"[i]UTC:[/]  {self.utc_time}\n"
            f"[i]Sync:[/] {sync_icon} {self.drift}µs"
        )


class GpsRightCol(Static):
    """GPS Right Column: Pos, Alt, Hdg, Acc."""

    lat = reactive(0)
    lon = reactive(0)
    alt = reactive(0)
    vel = reactive(0)
    hdg = reactive(0)
    h_acc = reactive(0)
    v_acc = reactive(0)

    def render(self) -> str:
        return (
            f"[i]Pos:[/]  {self.lat / 1e7:.5f}°, {self.lon / 1e7:.5f}°\n"
            f"[i]Alt:[/]  {self.alt / 1000:.1f}m  [i]Hdg:[/] {self.hdg / 100:.1f}°\n"
            f"[i]Vel:[/]  {self.vel}cm/s  [i]Acc:[/] H:{self.h_acc / 1000:.1f}m"
        )


class RcWidget(Static):
    """Displays RC Channels."""

    ch_a = reactive(1500)
    ch_e = reactive(1500)
    ch_t = reactive(1500)
    ch_r = reactive(1500)

    def render(self) -> str:
        return (
            f"[b]RC Channels[/b]\n"
            f"A: {self.ch_a}  E: {self.ch_e}\n"
            f"T: {self.ch_t}  R: {self.ch_r}"
        )


class AccelWidget(Static):
    """Real-time Accelerometer Plot (X, Y, Z)."""

    def compose(self) -> ComposeResult:
        with Horizontal():
            yield PlotextPlot(id="plot_x", classes="plot_axis")
            yield PlotextPlot(id="plot_y", classes="plot_axis")
            yield PlotextPlot(id="plot_z", classes="plot_axis")

    def on_mount(self) -> None:
        self.plot_x = self.query_one("#plot_x", PlotextPlot)
        self.plot_y = self.query_one("#plot_y", PlotextPlot)
        self.plot_z = self.query_one("#plot_z", PlotextPlot)

        for p, title in [
            (self.plot_x, "Accel X"),
            (self.plot_y, "Accel Y"),
            (self.plot_z, "Accel Z"),
        ]:
            p.plt.title(title)
            p.plt.ylim(0, 1)
            p.plt.xlabel("Time")
            p.plt.theme("dark")

        # Data buffers
        self.data_x = list(range(100))
        self.data_y_x = [0.0] * 100
        self.data_y_y = [0.0] * 100
        self.data_y_z = [0.0] * 100
        self.idx = 0

        # 25Hz Update Timer
        self.set_interval(1 / 25, self.update_plot)

    def update_plot(self) -> None:
        # Shift data (TODO: Replace with real IMU data)
        self.data_y_x.pop(0)
        self.data_y_y.pop(0)
        self.data_y_z.pop(0)

        self.data_y_x.append(random.random())
        self.data_y_y.append(random.random())
        self.data_y_z.append(random.random())

        self.idx += 1
        self.data_x = list(range(self.idx, self.idx + 100))

        # Redraw plots
        for plot, data, color in [
            (self.plot_x, self.data_y_x, "red"),
            (self.plot_y, self.data_y_y, "green"),
            (self.plot_z, self.data_y_z, "blue"),
        ]:
            plot.plt.clear_data()
            plot.plt.plot(self.data_x, data, marker="dot", color=color)
            plot.refresh()


class DashboardApp(App):
    """Raven Dashboard TUI."""

    CSS = """
    Screen {
        layout: grid;
        grid-size: 2;
        grid-columns: 2fr 1fr;
    }

    #left-pane {
        width: 100%;
        height: 100%;
        border: solid green;
    }

    #status {
        height: 2;
        background: $surface;
        border-top: solid $primary;
        text-align: left;
        padding-left: 1;
    }

    #right-pane {
        width: 100%;
        height: 100%;
        border: solid blue;
        dock: right;
    }

    .telemetry {
        height: 10;
        background: $surface;
        border-bottom: solid $primary;
        padding: 1;
    }

    #gps_container {
        width: 2fr;
        height: 100%;
        border: solid $primary;
    }

    #gps_left {
        width: 1fr;
        height: 100%;
    }

    #gps_right {
        width: 1fr;
        height: 100%;
    }

    #rc {
        width: 1fr;
        height: 100%;
        border: solid $primary;
    }

    .plot_container {
        height: 15;
        border-bottom: solid $primary;
        padding: 0;
    }

    .plot_axis {
        width: 1fr;
        height: 100%;
        border: solid gray;
    }

    .spacer {
        height: 1fr;
    }

    Input {
        dock: bottom;
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
        self.writer = None
        self.reader = None
        self.ctrl_writer = None
        self.ctrl_reader = None
        self.connected = False

    def compose(self) -> ComposeResult:
        yield Header()

        # Left Pane: Logs + Input
        with Vertical(id="left-pane"):
            yield Label("[b]System Logs[/b]")
            yield Log(id="log")
            yield Input(placeholder="Send command...", id="input")

        # Right Pane: Telemetry + Plots
        with Vertical(id="right-pane"):
            yield Horizontal(
                Vertical(
                    Label("[b]GPS & Time[/b]"),
                    Horizontal(
                        GpsLeftCol(id="gps_left"),
                        GpsRightCol(id="gps_right"),
                    ),
                    id="gps_container",
                ),
                RcWidget(id="rc"),
                classes="telemetry",
            )
            yield Vertical(AccelWidget(id="accel"), classes="plot_container")
            yield Static(classes="spacer")
            yield ConnectionStatus(id="status")

        yield Footer()

    async def on_mount(self) -> None:
        self.query_one("#log").write(f"Target: {self.target_ip}")
        if self.target_ip:
            asyncio.create_task(self.connect_and_monitor())

    async def action_connect(self) -> None:
        if not self.connected:
            asyncio.create_task(self.connect_and_monitor())

    async def action_disconnect(self) -> None:
        if self.connected:
            await self.disconnect()

    async def action_clear_log(self) -> None:
        self.query_one("#log").clear()

    async def disconnect(self):
        self.connected = False
        for w in [self.writer, self.ctrl_writer]:
            if w:
                w.close()
                try:
                    await w.wait_closed()
                except Exception:
                    pass

        self.writer = self.reader = self.ctrl_writer = self.ctrl_reader = None

        self.query_one("#status").status = "Disconnected"
        self.query_one("#status").color = "red"
        self.query_one("#log").write("Disconnected.")

    async def connect_and_monitor(self):
        log = self.query_one("#log")
        status = self.query_one("#status")

        if self.connected:
            log.write("Already connected.")
            return

        status.status = "Connecting..."
        status.color = "yellow"

        try:
            # 1. Connect Control
            log.write(f"Ctrl → {self.target_ip}:{CTRL_PORT}")
            self.ctrl_reader, self.ctrl_writer = await asyncio.open_connection(
                self.target_ip, CTRL_PORT
            )

            # 2. Enable Bridge
            log.write("BRIDGE mode...")
            self.ctrl_writer.write(b"BRIDGE\n")
            await self.ctrl_writer.drain()

            resp = await self.ctrl_reader.readuntil(b"\n")
            if b"OK" not in resp:
                log.write(f"Bridge Failed: {resp}")
                await self.disconnect()
                return

            # 3. Connect Data
            log.write(f"Data → {self.target_ip}:{DATA_PORT}")
            self.reader, self.writer = await asyncio.open_connection(
                self.target_ip, DATA_PORT
            )

            self.connected = True
            status.status = "Connected"
            status.color = "green"
            log.write("Monitor Active.")

            # 4. Read Loop
            while self.connected:
                data = await self.reader.read(1024)
                if not data:
                    break

                text = data.decode("utf-8", errors="ignore")
                for line in text.split("\n"):
                    line = line.strip()
                    if line:
                        self.process_line(line)

        except Exception as e:
            log.write(f"Error: {e}")
            await self.disconnect()

    def process_line(self, line: str):
        if line.startswith("GPS:"):
            self.update_gps(line)
        elif line.startswith("RC") and "A=" in line:
            self.update_rc(line)
        elif line.startswith("[TimeSync]"):
            self.update_timesync(line)
        elif any(c.isalnum() for c in line):
            self.query_one("#log").write(line)

    def update_gps(self, line: str):
        left = self.query_one("#gps_left")
        right = self.query_one("#gps_right")
        try:
            for p in line.replace("GPS:", "").split():
                if "=" in p:
                    k, v = p.split("=", 1)
                    if k == "fix":
                        left.fix = int(v)
                    elif k == "sats":
                        left.sats = int(v)
                    elif k == "lat":
                        right.lat = int(v)
                    elif k == "lon":
                        right.lon = int(v)
                    elif k == "alt":
                        right.alt = int(v)
                    elif k == "vel":
                        right.vel = int(v)
                    elif k == "hdg":
                        right.hdg = int(v)
                    elif k == "hac":
                        right.h_acc = int(v)
                    elif k == "vac":
                        right.v_acc = int(v)
                    elif k == "t":
                        left.utc_time = v
        except Exception:
            pass

    def update_rc(self, line: str):
        w = self.query_one("#rc")
        try:
            content = line.split(":", 1)[-1] if ":" in line else line
            for item in content.split():
                if "=" in item:
                    k, v = item.split("=", 1)
                    if v.isdigit():
                        val = int(v)
                        if k == "A":
                            w.ch_a = val
                        elif k == "E":
                            w.ch_e = val
                        elif k == "T":
                            w.ch_t = val
                        elif k == "R":
                            w.ch_r = val
        except Exception:
            pass

    def update_timesync(self, line: str):
        w = self.query_one("#gps_left")
        try:
            for p in line.replace("[TimeSync]", "").split():
                if "=" in p:
                    k, v = p.split("=", 1)
                    if k == "drift":
                        w.drift = int(v)
                    elif k == "synced":
                        w.synced = v == "1"
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Raven Dashboard")
    parser.add_argument("ip", nargs="?", default="192.168.4.1", help="ESP32 IP")
    args = parser.parse_args()

    app = DashboardApp(ip=args.ip)
    app.run()


if __name__ == "__main__":
    main()
