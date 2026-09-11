#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
#     "pyserial",
#     "rich",
# ]
# ///

import argparse
import cmd
import fcntl
import os
import re
import select
import shutil
import socket
import subprocess
import sys
import time
import zlib

import serial
from rich.console import Console
from rich.progress import (
    BarColumn,
    DownloadColumn,
    Progress,
    TimeRemainingColumn,
    TransferSpeedColumn,
)

# Constants
CTRL_PORT = 9000
DATA_PORT = 9001
DEFAULT_SSID = "32Raven"
DEFAULT_PASS = "32Raven@1234"
DEFAULT_FALLBACK_IP = "192.168.4.1"
CHUNK_SIZE = 4096
SERVICE_WAIT_SECONDS = 60
FLASH_STATUS_POLL_S = 0.5
FLASH_STATUS_TIMEOUT_S = 120
SERVICE_POLL_SECONDS = 1.0
WIFI_WAIT_SECONDS = 15

# STATUS? state values, mirroring HostLink::Status in host_link.hpp.
STATE_DONE = 1
STATE_VERIFYING = 2

# Mirrors UsbHostLink::kChunkBytes: the bridge acknowledges every chunk,
# and its USB driver drops what it cannot hold, so nothing more is in flight.
USB_CHUNK_SIZE = 512
USB_REPLY_TIMEOUT_S = 2.0
# The bridge's console shares its USB port; these are the lines that are not
# console.
PROTOCOL_PREFIXES = ("OK", "ERR", "STATUS")
SERVICE_HINT = (
    "  Put it in Service mode: press the button until the OLED reads Service."
)

# Flash-path output only; the interactive shell keeps plain prints.
console = Console()


class AutoConnector:
    @staticmethod
    def get_wifi_state():
        """Returns (state, ssid, device) for the wifi interface.

        state is nmcli's word -- "connected", "connecting (...)",
        "disconnected" -- or "" when there is no wifi device or no nmcli.
        A connected device wins over a connecting one.
        """
        if not shutil.which("nmcli"):
            return "", None, None

        try:
            # -t: terse (escaped), -f: fields
            out = subprocess.check_output(
                ["nmcli", "-t", "-f", "TYPE,STATE,CONNECTION,DEVICE", "device"],
                stderr=subprocess.DEVNULL,
            ).decode("utf-8")
        except Exception:
            return "", None, None

        best = ("", None, None)
        for line in out.splitlines():
            # wifi:connected:32Raven:wlan0
            parts = line.strip().split(":")
            if len(parts) < 4 or parts[0] != "wifi":
                continue
            state, ssid, dev = parts[1], parts[2] or None, parts[3]
            if state == "connected":
                return state, ssid, dev
            if state.startswith("connecting") and not best[0]:
                best = (state, ssid, dev)
        return best

    @staticmethod
    def wait_for_wifi(timeout_s):
        """Polls until the wifi lands somewhere, connected or given up."""
        deadline = time.monotonic() + timeout_s
        while True:
            triple = AutoConnector.get_wifi_state()
            if triple[0] == "connected" or time.monotonic() > deadline:
                return triple
            time.sleep(0.5)

    @staticmethod
    def connect_to_ssid(ssid, password):
        """Attempts to connect to the SSID using nmcli."""
        if not shutil.which("nmcli"):
            print("Error: nmcli not found. Cannot auto-connect.")
            return False

        print(f"Attempting to connect to WiFi '{ssid}'...")
        try:
            subprocess.check_call(
                [
                    "nmcli",
                    "device",
                    "wifi",
                    "connect",
                    ssid,
                    "password",
                    password,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return True
        except subprocess.CalledProcessError:
            print("Failed to connect via nmcli.")
            return False

    @staticmethod
    def get_gateway_ip(device):
        """Gets the default gateway IP for the given interface."""
        if not shutil.which("ip"):
            return None

        try:
            # ip route show dev <dev>
            out = subprocess.check_output(
                ["ip", "route", "show", "dev", device],
                stderr=subprocess.DEVNULL,
            ).decode("utf-8")

            # match "default via <IP>"
            for line in out.splitlines():
                if "default via" in line:
                    match = re.search(r"default via ([\d\.]+)", line)
                    if match:
                        return match.group(1)

            # fallback: match "src <IP>" -> assume .1
            for line in out.splitlines():
                if "src" in line:
                    match = re.search(r"src ([\d\.]+)", line)
                    if match:
                        ip = match.group(1)
                        # Assume gateway is x.x.x.1
                        parts = ip.split(".")
                        parts[-1] = "1"
                        return ".".join(parts)

        except Exception:
            pass
        return None

    @staticmethod
    def wait_gateway_ip(device, timeout_s):
        """Polls for the gateway; no route exists until DHCP lands."""
        deadline = time.monotonic() + timeout_s
        while True:
            ip = AutoConnector.get_gateway_ip(device)
            if ip or time.monotonic() > deadline:
                return ip
            time.sleep(0.5)


def await_service(probe, retry, refusal):
    """probe() until it answers True, waiting out the wrong mode if retry.

    refusal is what the first failed probe prints, before the Service hint.
    """
    deadline = time.time() + SERVICE_WAIT_SECONDS
    announced = False
    while True:
        if probe():
            if announced:
                print("\nService mode detected.")
            return True
        if not announced:
            print()
            for line in refusal:
                print(f"  {line}")
            print(SERVICE_HINT)
            if not retry:
                print()
                return False
            print(f"  Waiting {SERVICE_WAIT_SECONDS}s...  ^C aborts.")
            print()
            announced = True
        if time.time() > deadline:
            print("Gave up waiting for Service mode.")
            return False
        time.sleep(SERVICE_POLL_SECONDS)


class CtrlLines:
    """Line-splitter over the ctrl socket."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def readline(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while b"\n" not in self.buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("ctrl socket: no line")
            self.sock.settimeout(remaining)
            chunk = self.sock.recv(256)
            if not chunk:
                raise ConnectionError("ctrl socket closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode(errors="replace").strip()


class TcpLink:
    """The WiFi transport: a ctrl socket for lines, a data socket for bytes."""

    chunk_size = CHUNK_SIZE

    def __init__(self, ip, timeout):
        self.ip = ip
        self.timeout = timeout
        self.ctrl = None
        self.lines = None
        self.data = None

    def describe(self):
        return self.ip

    def open(self, retry):
        """Open the ctrl socket, explaining a refusal, not echoing errno.

        ECONNREFUSED here is not ambiguous: MavlinkWifiState calls Tcp().Stop()
        but leaves the AP up, so the host still associates and routes. A closed
        port 9000 with a reachable host means the ESP32 is running, just not in
        Service mode. Anything else -- timeout, unreachable -- is a network
        fault and is reported as-is.
        """

        def connect():
            try:
                self.ctrl = socket.create_connection(
                    (self.ip, CTRL_PORT), timeout=self.timeout
                )
            except ConnectionRefusedError:
                return False
            self.lines = CtrlLines(self.ctrl)
            return True

        try:
            return await_service(
                connect,
                retry,
                [
                    f"CONNECTION REFUSED   {self.ip}:{CTRL_PORT}",
                    "ESP32 is up, Service server is not.",
                ],
            )
        except OSError as e:
            print(f"Connection failed: {e}")
            return False

    def request(self, cmd):
        """One line out, one line back; None once the socket is gone."""
        self.ctrl.sendall((cmd.strip() + "\n").encode("ascii"))
        try:
            return self.lines.readline(self.timeout)
        except ConnectionError:
            return None

    def open_data(self):
        try:
            self.data = socket.create_connection(
                (self.ip, DATA_PORT), timeout=self.timeout
            )
            return True
        except OSError as e:
            print(f"Error connecting to data port: {e}")
            return False

    def send_chunk(self, chunk):
        self.data.sendall(chunk)

    def alive(self):
        try:
            self.ctrl.send(b"\n")
            return True
        except OSError:
            return False

    def close(self):
        for sock in (self.ctrl, self.data):
            if sock:
                try:
                    sock.close()
                except OSError:
                    pass
        self.ctrl = None
        self.data = None


class SerialLink:
    """The USB transport: one byte stream, shared with the bridge's console."""

    chunk_size = USB_CHUNK_SIZE
    # No raw socket to hand the shell.
    ctrl = None

    def __init__(self, port, timeout):
        self.port = port
        self.timeout = timeout
        self.ser = None

    def describe(self):
        return self.port

    def open(self, retry):
        ser = serial.Serial()
        ser.port = self.port
        ser.timeout = USB_REPLY_TIMEOUT_S
        # The USB-Serial-JTAG peripheral resets the chip on RTS high with DTR
        # low. The kernel raises both on open and pyserial then applies DTR
        # before RTS, which is exactly that state; so RTS is dropped first
        # with DTR left up, and DTR follows once the port is open.
        ser.rts = False
        try:
            ser.open()
        except serial.SerialException as e:
            print(f"Could not open {self.port}: {e}")
            return False
        ser.dtr = False
        self.ser = ser
        # Nothing refuses a serial port, and only Service polls this link, so
        # the wrong mode is a STATUS? that goes unanswered.
        if await_service(
            lambda: self.request("STATUS?") is not None,
            retry,
            [f"NO ANSWER   {self.port}", "The port opened; Service is not on."],
        ):
            return True
        self.close()
        return False

    def _reply(self, timeout_s=USB_REPLY_TIMEOUT_S):
        """The next protocol line; console lines pass through."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith(PROTOCOL_PREFIXES):
                return line
            if line and line.isprintable():
                print(line)
        return None

    def request(self, cmd):
        self.ser.write((cmd.strip() + "\n").encode("ascii"))
        return self._reply()

    def open_data(self):
        return True

    def send_chunk(self, chunk):
        self.ser.write(chunk)
        # The bridge erases the target's sectors before it takes the first
        # chunk, seconds with its tick held; the WiFi link rides the same
        # stall on its socket timeout.
        reply = self._reply(self.timeout)
        if reply != "OK":
            raise OSError(f"chunk not acknowledged: {reply}")

    def alive(self):
        return self.ser is not None and self.ser.is_open

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            self.ser = None


class Esp32Shell(cmd.Cmd):
    intro = (
        "Welcome to the ESP32/32raven Shell. Type help or ? to list commands.\n"
    )
    prompt = "(disconnected) > "

    def __init__(self, ip=None, port=None, timeout=10, wait_for_service=False):
        super().__init__()
        self.target_ip = ip
        self.port = port
        self.timeout = timeout
        # Only the one-shot flash waits: an interactive session has a human
        # who can read the message and retry.
        self.wait_for_service = wait_for_service
        self.link = None
        self.failed = False

        # If no IP provided, just prompt updates
        if self.port or self.target_ip:
            self.prompt = f"({self.port or self.target_ip}) > "

    def do_connect(self, arg):
        """Connect to the ESP32. Usage: connect [ip]."""
        if self.link:
            print(f"Already connected to {self.link.describe()}")
            return

        if self.port:
            link = SerialLink(self.port, self.timeout)
        else:
            ip = arg if arg else self.target_ip
            if not ip:
                print("No IP specified and auto-connect not yet run.")
                return
            self.target_ip = ip
            link = TcpLink(ip, self.timeout)

        print(f"Connecting to {link.describe()}...")
        if link.open(retry=self.wait_for_service):
            self.link = link
            self.prompt = f"({link.describe()}) > "
            print("Connected.")

    def do_disconnect(self, arg):
        """Disconnect from the ESP32."""
        if self.link:
            self.link.close()
            self.link = None
        self.prompt = "(disconnected) > "
        print("Disconnected.")

    def do_status(self, arg):
        """Get the current status of the ESP32."""
        if not self._ensure_connected():
            return

        resp = self._send_ctrl("STATUS?")
        if resp:
            print(f"Remote: {resp}")

    def do_reboot(self, arg):
        """Reboot the ESP32 (and STM32)."""
        if not self._ensure_connected():
            return

        print("Sending REBOOT command...")
        self._send_ctrl("RESET")
        print("Target is rebooting. Connection closed.")
        self.do_disconnect(None)

    def do_abort(self, arg):
        """Send ABORT command to ESP32."""
        if not self._ensure_connected():
            return

        print("Sending ABORT command...")
        resp = self._send_ctrl("ABORT")
        print(f"Response: {resp}")
        self.do_disconnect(None)

    def _upload(self, filename, begin_extra):
        """BEGIN on ctrl, open the data path on OK, and stream the file."""
        with open(filename, "rb") as f:
            image = f.read()
        filesize = len(image)
        crc = zlib.crc32(image) & 0xFFFFFFFF

        print(f"Handshake (BEGIN{begin_extra})...")
        begin = f"BEGIN size={filesize} crc={crc}{begin_extra}"
        resp = self._send_ctrl(begin)
        if resp != "OK":
            if resp is None:
                # The connect succeeded, so something is listening; silence
                # means nothing is consuming commands. The claim stops there:
                # this path has covered a wedged server as well as the wrong
                # mode, and a guessed diagnosis reads as fact.
                console.print(
                    "[red]Connected, but nothing answered BEGIN.[/red] The "
                    "ESP32 is not serving flashing right now -- not on the "
                    "Service mode, or its command loop is stuck. Check the "
                    "serial log."
                )
            elif "wrong_mode" in resp:
                console.print(
                    "[yellow]Wrong mode:[/yellow] put the ESP32 on Service "
                    "and retry."
                )
            else:
                console.print(f"[red]Target refused handshake:[/red] {resp}")
            return False

        if not self.link.open_data():
            # The target armed a transfer on BEGIN; tell it the stream is
            # not coming rather than leaving it waiting for one.
            self._send_ctrl("ABORT")
            return False

        total_sent = 0
        start_time = time.time()
        progress = Progress(
            "[progress.description]{task.description}",
            BarColumn(),
            DownloadColumn(),
            TransferSpeedColumn(),
            TimeRemainingColumn(),
            console=console,
            transient=True,
        )
        try:
            with progress:
                task = progress.add_task("upload", total=filesize)
                for offset in range(0, filesize, self.link.chunk_size):
                    chunk = image[offset : offset + self.link.chunk_size]
                    self.link.send_chunk(chunk)
                    total_sent += len(chunk)
                    progress.update(task, completed=total_sent)
        except Exception as e:
            console.print(f"[red]Error sending data:[/red] {e}")
            self.do_disconnect(None)
            return False

        duration = time.time() - start_time
        rate_kb = total_sent / duration / 1024
        console.print(
            f"[green]Uploaded[/green] {total_sent} bytes in {duration:.2f}s "
            f"({rate_kb:.1f} KB/s)"
        )
        return True

    def _await_flash(self, expect_done=False):
        """Poll STATUS? until the target finishes, refuses, or the wait ends.

        A dropped connection is the ordinary success signal -- the target
        reboots into the new image mid-poll. An ERR line is a refusal the
        firmware could only make after the handshake, so it ends the wait
        rather than being counted as progress.
        """
        deadline = time.monotonic() + FLASH_STATUS_TIMEOUT_S
        progress = Progress(
            "[progress.description]{task.description}",
            BarColumn(),
            DownloadColumn(),
            TimeRemainingColumn(),
            console=console,
            transient=True,
        )
        status_re = re.compile(r"rx=(\d+) total=(\d+) state=(\d+)")
        with progress:
            task = progress.add_task("write", total=None)
            verifying = False
            while time.monotonic() < deadline:
                # One lost reply is not a halted bridge; a halted bridge is
                # silent for the retry as well.
                resp = self._send_ctrl("STATUS?") or self._send_ctrl("STATUS?")
                if not resp:
                    progress.stop()
                    if expect_done:
                        # The bridge halts on a failed flash rather than
                        # reporting it, so silence before done is the failure.
                        console.print(
                            "[red]Target went quiet before reporting "
                            "done.[/red] The bridge's display and log have "
                            "the reason."
                        )
                        return False
                    # The target reboots into the new image mid-poll; the
                    # drop is the ordinary success signal.
                    console.print(
                        "[green]Flash success[/green] (target rebooted)"
                    )
                    return True
                if resp.startswith("ERR"):
                    progress.stop()
                    console.print(
                        f"[red]Target refused the flash:[/red] {resp}"
                    )
                    return False
                if match := status_re.search(resp):
                    rx = int(match.group(1))
                    total = int(match.group(2))
                    state = int(match.group(3))
                    # rx restarts from zero when the target switches to
                    # verifying, so the bar has to restart with it.
                    if state == STATE_VERIFYING and not verifying:
                        verifying = True
                        progress.update(task, description="verify")
                    progress.update(
                        task, completed=rx, total=total if total else None
                    )
                    if expect_done and state == STATE_DONE:
                        progress.stop()
                        console.print("[green]Flash success[/green]")
                        return True
                time.sleep(FLASH_STATUS_POLL_S)
        console.print(
            f"[red]Target never reported done after "
            f"{FLASH_STATUS_TIMEOUT_S}s.[/red] "
            "Check which mode the ESP32 is in."
        )
        return False

    def do_flash(self, arg):
        """Flash a binary file. Usage: flash <path_to_bin>."""
        self.failed = True
        if not arg:
            print("Error: Usage: flash <path_to_bin>")
            return

        filename = arg.strip()
        if not os.path.exists(filename):
            print(f"Error: File '{filename}' not found.")
            return

        if not self._ensure_connected():
            return

        console.print(
            f"Flashing [bold]{filename}[/bold] "
            f"({os.path.getsize(filename)} bytes)"
        )
        if not self._upload(filename, ""):
            return

        # STATUS rx=... total=... state=1 err=0
        if not self._await_flash(expect_done=True):
            return
        self.failed = False

        # STM32 keeps the connection up; an ESP32 flash drops it.
        if self._ensure_connected_silent():
            print("Session active.")
        else:
            self.do_disconnect(None)

    def _ensure_connected_silent(self):
        """Check if connected without auto-reconnect or prints."""
        return self.link is not None and self.link.alive()

    def do_flash_esp(self, arg):
        """Flash ESP32 firmware. Usage: flash_esp <path_to_bin>."""
        self.failed = True
        if not arg:
            print("Error: Usage: flash_esp <path_to_bin>")
            return

        filename = arg.strip()
        if not os.path.exists(filename):
            print(f"Error: File '{filename}' not found.")
            return

        if not self._ensure_connected():
            return

        console.print(
            f"Flashing ESP32 [bold]{filename}[/bold] "
            f"({os.path.getsize(filename)} bytes)"
        )
        if not self._upload(filename, " target=esp32"):
            return

        # The ESP32 reboots into the new image; the drop is the only ack.
        if not self._await_flash():
            return
        self.failed = False

        self.do_disconnect(None)

    def do_exit(self, arg):
        """Exit the shell."""
        self.do_disconnect(None)
        print("Bye!")
        return True

    def do_quit(self, arg):
        return self.do_exit(arg)

    def do_shell(self, arg):
        """Enter raw interactive shell mode; Ctrl+C exits."""
        if not self._ensure_connected():
            return

        sock = self.link.ctrl
        if sock is None:
            print("The shell needs the WiFi link; USB has make monitor-esp32.")
            return

        print(f"--- Entering Interactive Shell ({self.link.describe()}) ---")
        print("Type commands directly. Ctrl+C to exit.")

        prompt = "32Raven> "
        sys.stdout.write(prompt)
        sys.stdout.flush()

        try:
            while self.link:
                # Wait for input from stdin or data from socket
                r, _, _ = select.select([sys.stdin, sock], [], [])

                if sys.stdin in r:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    sock.sendall(line.encode("ascii"))
                    # Don't print prompt here, expect response

                if sock in r:
                    data = sock.recv(1024)
                    if not data:
                        print("\nDisconnected by remote.")
                        self.do_disconnect(None)
                        break
                    text = data.decode("ascii", errors="replace")
                    sys.stdout.write(text)
                    if text.endswith("\n"):
                        sys.stdout.write(prompt)
                    sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nExiting shell mode.")
        except OSError as e:
            print(f"\nSocket error: {e}")
            self.do_disconnect(None)

    def default(self, line):
        """Send unknown commands directly to ESP32."""
        if not self.link:
            self.do_connect(self.target_ip)
            if not self.link:
                return

        sock = self.link.ctrl
        if sock is None:
            print(f"> {line}")
            print(self._send_ctrl(line) or "")
            return

        try:
            print(f"> {line}")
            sock.sendall((line + "\n").encode("ascii"))
            # Wait briefly for response (pseudo-shell)
            sock.settimeout(0.5)
            try:
                while True:
                    data = sock.recv(1024)
                    if not data:
                        break
                    sys.stdout.write(data.decode("ascii", errors="replace"))
            except TimeoutError:
                pass
            sock.settimeout(self.timeout)
            print()
        except OSError as e:
            print(f"Error: {e}")
            self.do_disconnect(None)

    # --- Helpers ---

    def _ensure_connected(self):
        if not self.link:
            print("Not connected. Trying auto-connect...")
            self.do_connect(self.target_ip)
        return self.link is not None

    def _send_ctrl(self, cmd):
        if not self.link:
            return None
        try:
            return self.link.request(cmd)
        except OSError as e:
            print(f"Link error: {e}")
            self.do_disconnect(None)
            return None


def resolve_target_ip():
    """The AP's gateway address, joining the network first if needed."""
    state, ssid, dev = AutoConnector.get_wifi_state()

    # The OS may be mid-association -- a second nmcli connect would abort
    # that attempt, so wait for it to land before judging where we are.
    if state.startswith("connecting"):
        print(f"WiFi is {state}, waiting for it to settle...")
        state, ssid, dev = AutoConnector.wait_for_wifi(WIFI_WAIT_SECONDS)

    if state == "connected" and ssid == DEFAULT_SSID:
        print(f"Already connected to {DEFAULT_SSID} on {dev}.")
    else:
        print(
            f"Not connected to {DEFAULT_SSID} (Currently: {ssid}). "
            "Attempting auto-connect..."
        )
        if not AutoConnector.connect_to_ssid(DEFAULT_SSID, DEFAULT_PASS):
            print(
                "Auto-connect failed. "
                f"Falling back to default {DEFAULT_FALLBACK_IP}."
            )
            return DEFAULT_FALLBACK_IP
        state, ssid, dev = AutoConnector.wait_for_wifi(WIFI_WAIT_SECONDS)
        if state != "connected":
            print(f"WiFi did not come up, using default {DEFAULT_FALLBACK_IP}")
            return DEFAULT_FALLBACK_IP

    ip = AutoConnector.wait_gateway_ip(dev, WIFI_WAIT_SECONDS)
    if ip:
        print(f"Detected Gateway IP: {ip}")
        return ip
    print(f"Could not resolve Gateway IP, using default {DEFAULT_FALLBACK_IP}")
    return DEFAULT_FALLBACK_IP


def main():
    parser = argparse.ArgumentParser(description="ESP32/32raven Shell Client")
    parser.add_argument(
        "ip", nargs="?", help="Target IP Address (Empty for auto-connect)"
    )
    parser.add_argument(
        "command", nargs="*", help="Run single command and exit"
    )
    parser.add_argument(
        "--port",
        help="Serial port of the bridge's USB; replaces the IP and WiFi",
    )

    args = parser.parse_args()
    # With a port there is no address, so the first positional is the verb.
    if args.port and args.ip:
        args.command = [args.ip] + args.command
        args.ip = None

    # Exclusive access check
    # We keep the file open until the process exits
    lock_file = open("/tmp/esp32_client.lock", "a+")

    def acquire_lock():
        try:
            lock_file.seek(0)
            fcntl.lockf(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # We got the lock, write our PID
            lock_file.seek(0)
            lock_file.truncate()
            lock_file.write(str(os.getpid()))
            lock_file.flush()
            return True
        except OSError:
            return False

    if not acquire_lock():
        # Read the PID of the locking process
        lock_file.seek(0)
        try:
            pid_str = lock_file.read().strip()
            if pid_str:
                pid = int(pid_str)
                print(f"Previous instance running (PID {pid}). Killing it...")
                try:
                    os.kill(pid, 15)  # SIGTERM
                    time.sleep(1)
                except OSError:
                    pass
        except ValueError:
            pass

        # Retry lock
        if not acquire_lock():
            print("Error: Could not acquire lock even after kill attempt.")
            sys.exit(1)

    target_ip = args.ip

    # Auto-Connect Logic if IP not provided
    if not target_ip and not args.port:
        target_ip = resolve_target_ip()

    # Check if a command is provided (one or more arguments)
    is_batch_mode = len(args.command) > 0

    shell = Esp32Shell(
        ip=target_ip, port=args.port, wait_for_service=is_batch_mode
    )

    if is_batch_mode:
        line = " ".join(args.command)
        shell.onecmd("connect")
        if not shell.link:
            sys.exit(1)
        shell.onecmd(line)
        sys.exit(1 if shell.failed else 0)
    else:
        try:
            shell.cmdloop()
        except KeyboardInterrupt:
            print("\nInterrupted.")
            shell.do_disconnect(None)


if __name__ == "__main__":
    main()
