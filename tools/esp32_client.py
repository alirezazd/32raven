#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
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

# STATUS? state values, mirroring TcpServer::Status in tcp_server.hpp.
STATE_DONE = 1
STATE_VERIFYING = 2

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


class Esp32Shell(cmd.Cmd):
    intro = (
        "Welcome to the ESP32/32raven Shell. Type help or ? to list commands.\n"
    )
    prompt = "(disconnected) > "

    def __init__(self, ip=None, timeout=10, wait_for_service=False):
        super().__init__()
        self.target_ip = ip
        self.timeout = timeout
        # Only the one-shot flash waits: an interactive session has a human
        # who can read the message and retry.
        self.wait_for_service = wait_for_service
        self.ctrl_sock = None
        self.data_sock = None
        self.connected = False
        self.failed = False

        # If no IP provided, just prompt updates
        if self.target_ip:
            self.prompt = f"({self.target_ip}) > "

    def do_connect(self, arg):
        """Connect to the ESP32. Usage: connect [ip]."""
        ip = arg if arg else self.target_ip

        if not ip:
            print("No IP specified and auto-connect not yet run.")
            return

        self.target_ip = ip
        if self.connected:
            print(f"Already connected to {self.target_ip}")
            return

        print(f"Connecting to {self.target_ip}...")
        if self._open_ctrl(retry=self.wait_for_service):
            self.connected = True
            self.prompt = f"({self.target_ip}) > "
            print("Connected.")

    def _open_ctrl(self, retry=False):
        """Open the ctrl socket, explaining a refusal, not echoing errno.

        ECONNREFUSED here is not ambiguous: MavlinkWifiState calls Tcp().Stop()
        but leaves the AP up, so the host still associates and routes. A closed
        port 9000 with a reachable host means the ESP32 is running, just not in
        Service mode. Anything else -- timeout, unreachable -- is a network
        fault and is reported as-is.
        """
        deadline = time.time() + SERVICE_WAIT_SECONDS
        announced = False
        while True:
            try:
                self.ctrl_sock = socket.create_connection(
                    (self.target_ip, CTRL_PORT), timeout=self.timeout
                )
                if announced:
                    print("\nService mode detected.")
                return True
            except ConnectionRefusedError:
                if not announced:
                    print()
                    print(
                        f"  CONNECTION REFUSED   {self.target_ip}:{CTRL_PORT}"
                    )
                    print("  ESP32 is up, Service server is not.")
                    print(
                        "  Put it in Service mode: press the button until "
                        "the OLED reads Service."
                    )
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
            except OSError as e:
                print(f"Connection failed: {e}")
                return False

    def do_disconnect(self, arg):
        """Disconnect from the ESP32."""
        self._close_sockets()
        self.connected = False
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
        """BEGIN on ctrl, open the data socket on OK, and stream the file."""
        filesize = os.path.getsize(filename)

        print(f"Handshake (BEGIN{begin_extra})...")
        resp = self._send_ctrl(f"BEGIN size={filesize} crc=0{begin_extra}")
        if resp != "OK":
            if resp is None:
                # The connect succeeded, so something is listening; silence
                # means nothing is consuming commands. The claim stops there:
                # this path has covered a wedged server as well as the wrong
                # page, and a guessed diagnosis reads as fact.
                console.print(
                    "[red]Connected, but nothing answered BEGIN.[/red] The "
                    "ESP32 is not serving flashing right now -- not on the "
                    "Service page, or its command loop is stuck. Check the "
                    "serial log."
                )
            elif "wrong_page" in resp:
                console.print(
                    "[yellow]Wrong page:[/yellow] put the ESP32 on Service "
                    "and retry."
                )
            else:
                console.print(f"[red]Target refused handshake:[/red] {resp}")
            return False

        try:
            self.data_sock = socket.create_connection(
                (self.target_ip, DATA_PORT), timeout=self.timeout
            )
        except OSError as e:
            # The target armed a transfer on BEGIN; tell it the stream is
            # not coming rather than leaving it waiting for one.
            print(f"Error connecting to data port: {e}")
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
            with open(filename, "rb") as f, progress:
                task = progress.add_task("upload", total=filesize)
                while True:
                    chunk = f.read(CHUNK_SIZE)
                    if not chunk:
                        break
                    self.data_sock.sendall(chunk)
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
                try:
                    resp = self._send_ctrl("STATUS?")
                except (OSError, BrokenPipeError):
                    resp = None
                if not resp:
                    # The target reboots into the new image mid-poll; the
                    # drop is the ordinary success signal.
                    progress.stop()
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
            "Check which page the ESP32 is on."
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
            print("Session active. You can run 'monitor' or other commands.")
        else:
            self.do_disconnect(None)

    def _ensure_connected_silent(self):
        """Check if connected without auto-reconnect or prints."""
        if not self.connected:
            return False
        try:
            # check socket health?
            self.ctrl_sock.send(b"\n")
            return True
        except OSError:
            return False

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

        print(f"--- Entering Interactive Shell ({self.target_ip}) ---")
        print("Type commands directly. Ctrl+C to exit.")

        prompt = "32Raven> "
        sys.stdout.write(prompt)
        sys.stdout.flush()

        try:
            while self.connected:
                # Wait for input from stdin or data from socket
                r, _, _ = select.select([sys.stdin, self.ctrl_sock], [], [])

                if sys.stdin in r:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    self.ctrl_sock.sendall(line.encode("ascii"))
                    # Don't print prompt here, expect response

                if self.ctrl_sock in r:
                    data = self.ctrl_sock.recv(1024)
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
        if not self.connected:
            self.do_connect(self.target_ip)
            if not self.connected:
                return

        try:
            print(f"> {line}")
            self.ctrl_sock.sendall((line + "\n").encode("ascii"))
            # Wait briefly for response (pseudo-shell)
            self.ctrl_sock.settimeout(0.5)
            try:
                while True:
                    data = self.ctrl_sock.recv(1024)
                    if not data:
                        break
                    sys.stdout.write(data.decode("ascii", errors="replace"))
            except TimeoutError:
                pass
            self.ctrl_sock.settimeout(self.timeout)
            print()
        except OSError as e:
            print(f"Error: {e}")
            self.do_disconnect(None)

    # --- Helpers ---

    def _ensure_connected(self):
        if not self.connected:
            print("Not connected. Trying auto-connect...")
            self.do_connect(self.target_ip)
        return self.connected

    def _close_sockets(self):
        if self.ctrl_sock:
            try:
                self.ctrl_sock.close()
            except OSError:
                pass
            self.ctrl_sock = None
        if self.data_sock:
            try:
                self.data_sock.close()
            except OSError:
                pass
            self.data_sock = None

    def _send_ctrl(self, cmd):
        if not self.ctrl_sock:
            return None
        try:
            msg = cmd.strip() + "\n"
            self.ctrl_sock.sendall(msg.encode("ascii"))

            # Read response
            resp = b""
            while True:
                chunk = self.ctrl_sock.recv(1)
                if not chunk:
                    return None
                resp += chunk
                if chunk == b"\n":
                    break
            return resp.decode("ascii").strip()
        except OSError as e:
            print(f"Socket error: {e}")
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

    args = parser.parse_args()

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
    if not target_ip:
        target_ip = resolve_target_ip()

    # Check if a command is provided (one or more arguments)
    is_batch_mode = len(args.command) > 0

    shell = Esp32Shell(ip=target_ip, wait_for_service=is_batch_mode)

    if is_batch_mode:
        line = " ".join(args.command)
        shell.onecmd("connect")
        if not shell.connected:
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
