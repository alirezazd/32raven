#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Pull blackbox logs off the aircraft over WiFi.

The ESP32 must be on the WiFi Log page (short-press past MAVLink in the main
menu) with its AP up; any other page answers the LOG verbs with a redirect.
Listing and transfer ride the same ctrl/data socket pair the firmware
programmer uses; the ESP32 relays FcLink frames from the STM32's SD card.

  ./tools/pull_logs.py list
  ./tools/pull_logs.py get LOG00042.ULG [-o out.ulg]

Throughput is FcLink-bound (~50 KB/s) -- this is the cable-free peek. Bulk
retrieval is USB mass storage from the USB Log page, one press further.
"""

# /// script
# dependencies = [
#     "prompt_toolkit",
#     "rich",
# ]
# ///

import argparse
import hashlib
import html
import os
import pathlib
import select
import shutil
import socket
import subprocess
import sys
import termios
import time
import tty

from esp32_client import resolve_target_ip
from prompt_toolkit import prompt
from prompt_toolkit.completion import PathCompleter
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.key_binding import KeyBindings
from rich.console import Console
from rich.progress import (
    BarColumn,
    DownloadColumn,
    Progress,
    TimeRemainingColumn,
    TransferSpeedColumn,
)
from rich.table import Table

CTRL_PORT = 9000
DATA_PORT = 9001

# At or above this, USB mass storage is the faster route.
BULK_HINT_BYTES = 8 * 1024 * 1024

WRONG_PAGE = "wrong page -- put the ESP32 on WiFi Log and retry"

console = Console()


class CancelledError(Exception):
    """Esc, at any prompt."""


def _escape_bindings() -> KeyBindings:
    keys = KeyBindings()

    @keys.add("escape", eager=True)
    def _(event):
        event.app.exit(exception=CancelledError)

    return keys


ESCAPE_KEYS = _escape_bindings()


class EscapeWatch:
    """Esc during a transfer, where there is no prompt to bind it to.

    Puts the terminal in cbreak so a lone keypress arrives without Enter, and
    is inert when stdin is not a terminal (piped, or a CI runner).
    """

    def __enter__(self):
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        if self.fd is not None:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def pressed(self) -> bool:
        if self.fd is None:
            return False
        while select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.read(1) == "\x1b":
                return True
        return False


def ask(label: str, **kwargs) -> str:
    """A prompt that treats Esc and Ctrl-C alike."""
    try:
        return prompt(label, key_bindings=ESCAPE_KEYS, **kwargs).strip()
    except (KeyboardInterrupt, EOFError) as exc:
        raise CancelledError from exc


def download_dir() -> pathlib.Path:
    """Where the desktop puts downloads, or ~/Downloads when it has no say."""
    env = os.environ.get("XDG_DOWNLOAD_DIR")
    if env:
        return pathlib.Path(os.path.expandvars(env)).expanduser()
    if shutil.which("xdg-user-dir"):
        try:
            out = subprocess.check_output(
                ["xdg-user-dir", "DOWNLOAD"], stderr=subprocess.DEVNULL
            ).decode()
            path = out.strip()
            # xdg-user-dir answers with $HOME when nothing is configured.
            if path and pathlib.Path(path) != pathlib.Path.home():
                return pathlib.Path(path)
        except (OSError, subprocess.CalledProcessError):
            pass
    return pathlib.Path.home() / "Downloads"


def human_bytes(size: int) -> str:
    value = float(size)
    for unit in ("B", "KB", "MB", "GB"):
        if value < 1024 or unit == "GB":
            if unit == "B":
                return f"{value:.0f} B"
            return f"{value:.1f} {unit}"
        value /= 1024
    return f"{value:.1f} GB"


def ask_path(label: str, default: pathlib.Path) -> pathlib.Path:
    """A path field that shows the default greyed out until you type."""
    ghost = HTML(
        f'<style fg="#6b6b6b">default: {html.escape(str(default))}</style>'
    )
    answer = ask(
        label,
        placeholder=ghost,
        completer=PathCompleter(expanduser=True),
        complete_while_typing=False,
    )
    return pathlib.Path(answer or default).expanduser()


class CtrlLines:
    """Line-splitter over the ctrl socket."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buf = b""

    def readline(self, timeout_s: float) -> str:
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


def fetch_list(ip: str) -> list[tuple[str, int]]:
    """Every closed log on the card, as (name, size)."""
    entries: list[tuple[str, int]] = []
    with socket.create_connection((ip, CTRL_PORT), timeout=5) as ctrl:
        lines = CtrlLines(ctrl)
        ctrl.sendall(b"LOG LIST\n")
        reply = lines.readline(5)
        if reply != "OK":
            raise RuntimeError(WRONG_PAGE if "wrong_page" in reply else reply)
        while True:
            line = lines.readline(10)
            if line.startswith("DONE"):
                return entries
            if line.startswith("ERR"):
                raise RuntimeError(line)
            parts = line.split()
            if len(parts) == 3 and parts[0] == "LOG":
                entries.append((parts[1], int(parts[2])))


def choose_log(entries: list[tuple[str, int]]) -> tuple[str, int] | None:
    table = Table(title="Logs on the card", title_justify="left")
    table.add_column("#", justify="right", style="cyan")
    table.add_column("Name")
    table.add_column("Size", justify="right")
    table.add_column("")
    for index, (name, size) in enumerate(entries, start=1):
        hint = "USB is faster" if size >= BULK_HINT_BYTES else ""
        table.add_row(str(index), name, human_bytes(size), hint)
    console.print(table)

    raw = ask(f"Log to pull [1-{len(entries)}, Esc cancels]: ")
    if not raw:
        return None
    try:
        pick = int(raw)
    except ValueError:
        console.print("[red]not a number[/red]")
        return None
    if not 1 <= pick <= len(entries):
        console.print("[red]out of range[/red]")
        return None
    return entries[pick - 1]


def cmd_browse(ip: str) -> int:
    try:
        return browse(ip)
    except CancelledError:
        console.print("[yellow]cancelled[/yellow]")
        return 130


def browse(ip: str) -> int:
    try:
        entries = fetch_list(ip)
    except (RuntimeError, OSError, TimeoutError) as exc:
        console.print(f"[red]error:[/red] {exc}")
        return 1
    if not entries:
        console.print(
            "No logs on the card yet -- one is written per armed flight."
        )
        return 0

    chosen = choose_log(entries)
    if chosen is None:
        return 0
    name, size = chosen

    out = ask_path("Save to: ", download_dir() / name)
    if out.is_dir():
        out = out / name
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        if ask(f"{out} exists, overwrite? [y/N]: ").lower() != "y":
            return 0

    console.print(f"{name} ({human_bytes(size)}) -> [bold]{out}[/bold]")
    return cmd_get(ip, name, str(out), expected_size=size)


def cmd_list(ip: str) -> int:
    with socket.create_connection((ip, CTRL_PORT), timeout=5) as ctrl:
        lines = CtrlLines(ctrl)
        ctrl.sendall(b"LOG LIST\n")
        reply = lines.readline(5)
        if reply != "OK":
            print(f"error: {reply or WRONG_PAGE}", file=sys.stderr)
            return 1
        while True:
            line = lines.readline(10)
            if line.startswith("DONE"):
                print(line)
                return 0
            if line.startswith("ERR"):
                print(f"error: {line}", file=sys.stderr)
                return 1
            print(line)


def cmd_get(
    ip: str, name: str, out_path: str, expected_size: int | None = None
) -> int:
    sha = hashlib.sha256()
    received = 0
    with socket.create_connection((ip, CTRL_PORT), timeout=5) as ctrl:
        lines = CtrlLines(ctrl)
        with socket.create_connection((ip, DATA_PORT), timeout=5) as data:
            ctrl.sendall(f"LOG GET {name}\n".encode())
            reply = lines.readline(5)
            if reply != "OK":
                print(f"error: {reply or WRONG_PAGE}", file=sys.stderr)
                return 1
            data.settimeout(10)
            progress = Progress(
                "[progress.description]{task.description}",
                BarColumn(),
                DownloadColumn(),
                TransferSpeedColumn(),
                TimeRemainingColumn(),
                console=console,
                transient=True,
            )
            with open(out_path, "wb") as out, progress, EscapeWatch() as esc:
                task = progress.add_task(name, total=expected_size)
                done_line = None
                try:
                    while done_line is None:
                        if esc.pressed():
                            raise KeyboardInterrupt
                        # The DONE line races the data tail; drain data first.
                        try:
                            chunk = data.recv(4096)
                            if chunk:
                                out.write(chunk)
                                sha.update(chunk)
                                received += len(chunk)
                                progress.update(task, completed=received)
                                continue
                        except TimeoutError:
                            pass
                        done_line = lines.readline(30)
                    data.settimeout(2)
                    while True:
                        try:
                            chunk = data.recv(4096)
                        except TimeoutError:
                            break
                        if not chunk:
                            break
                        out.write(chunk)
                        sha.update(chunk)
                        received += len(chunk)
                        progress.update(task, completed=received)
                except KeyboardInterrupt:
                    progress.stop()
                    # Closing the sockets is the abort: the device's send fails.
                    got = human_bytes(received)
                    console.print(
                        f"[yellow]aborted[/yellow] after {got} -- "
                        f"partial file at {out_path}"
                    )
                    return 130

    if done_line.startswith("ERR"):
        print(f"error: {done_line}", file=sys.stderr)
        return 1
    fields = dict(kv.split("=", 1) for kv in done_line.split()[1:] if "=" in kv)
    expected_size = int(fields.get("size", "-1"))
    expected_sha = fields.get("sha256", "")
    if received != expected_size or sha.hexdigest() != expected_sha:
        print(
            f"error: integrity mismatch (got {received} bytes, "
            f"sha {sha.hexdigest()[:16]}...; device said {expected_size}, "
            f"{expected_sha[:16]}...)",
            file=sys.stderr,
        )
        return 1
    print(f"{out_path}: {received} bytes, sha256 verified")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ip",
        default=None,
        help="Target IP address (default: the same AP gateway the "
        "programmer resolves)",
    )
    sub = parser.add_subparsers(dest="cmd")
    sub.add_parser("list")
    get = sub.add_parser("get")
    get.add_argument("name")
    get.add_argument("-o", "--out", default=None)
    args = parser.parse_args()

    ip = args.ip or resolve_target_ip()
    if args.cmd is None:
        return cmd_browse(ip)
    if args.cmd == "list":
        return cmd_list(ip)
    return cmd_get(ip, args.name, args.out or args.name)


if __name__ == "__main__":
    sys.exit(main())
