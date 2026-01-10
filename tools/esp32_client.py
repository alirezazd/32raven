#!/usr/bin/env python3

import socket
import sys
import argparse
import time
import os
import cmd
import subprocess
import shutil
import re

# Constants
CTRL_PORT = 9000
DATA_PORT = 9001
DEFAULT_SSID = "32Raven"
DEFAULT_PASS = "32Raven@1234"
DEFAULT_FALLBACK_IP = "192.168.4.1"
CHUNK_SIZE = 4096

class AutoConnector:
    @staticmethod
    def get_current_wifi():
        """Returns (ssid, device) if connected to wifi, else (None, None)."""
        if not shutil.which("nmcli"):
            return None, None
        
        try:
            # -t: terse (escaped), -f: fields
            out = subprocess.check_output(
                ["nmcli", "-t", "-f", "TYPE,STATE,CONNECTION,DEVICE", "device"],
                stderr=subprocess.DEVNULL
            ).decode("utf-8")
            
            for line in out.splitlines():
                # wifi:connected:32Raven:wlan0
                parts = line.strip().split(":")
                if len(parts) >= 4 and parts[0] == "wifi" and parts[1] == "connected":
                    return parts[2], parts[3]
        except Exception:
            pass
        return None, None

    @staticmethod
    def connect_to_ssid(ssid, password):
        """Attempts to connect to the SSID using nmcli."""
        if not shutil.which("nmcli"):
            print("Error: nmcli not found. Cannot auto-connect.")
            return False

        print(f"Attempting to connect to WiFi '{ssid}'...")
        try:
            subprocess.check_call(
                ["nmcli", "device", "wifi", "connect", ssid, "password", password],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
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
                stderr=subprocess.DEVNULL
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
                        parts = ip.split('.')
                        parts[-1] = '1'
                        return ".".join(parts)
                        
        except Exception:
            pass
        return None

class Esp32Shell(cmd.Cmd):
    intro = 'Welcome to the ESP32/32raven Shell. Type help or ? to list commands.\n'
    prompt = '(disconnected) > '

    def __init__(self, ip=None, timeout=5.0):
        super().__init__()
        self.target_ip = ip
        self.timeout = timeout
        self.ctrl_sock = None
        self.data_sock = None
        self.connected = False
        
        # If no IP provided, just prompt updates
        if self.target_ip:
            self.prompt = f"({self.target_ip}) > "

    def do_connect(self, arg):
        """Connect to the ESP32. Usage: connect [ip]"""
        ip = arg if arg else self.target_ip
        
        if not ip:
            print("No IP specified and auto-connect not yet run.")
            return

        self.target_ip = ip
        if self.connected:
            print(f"Already connected to {self.target_ip}")
            return

        print(f"Connecting to {self.target_ip}...")
        try:
            self.ctrl_sock = socket.create_connection((self.target_ip, CTRL_PORT), timeout=self.timeout)
            self.connected = True
            self.prompt = f"({self.target_ip}) > "
            print("Connected.")
        except socket.error as e:
            print(f"Connection failed: {e}")

    def do_disconnect(self, arg):
        """Disconnect from the ESP32."""
        self._close_sockets()
        self.connected = False
        self.prompt = '(disconnected) > '
        print("Disconnected.")

    def do_status(self, arg):
        """Get the current status of the ESP32."""
        if not self._ensure_connected(): return
        
        resp = self._send_ctrl("STATUS?")
        if resp:
            print(f"Remote: {resp}")

    def do_reboot(self, arg):
        """Reboot the ESP32 (and STM32)."""
        if not self._ensure_connected(): return

        print("Sending REBOOT command...")
        self._send_ctrl("RESET")
        print("Target is rebooting. Connection closed.")
        self.do_disconnect(None)

    def do_abort(self, arg):
        """Send ABORT command to ESP32."""
        if not self._ensure_connected(): return

        print("Sending ABORT command...")
        resp = self._send_ctrl("ABORT")
        print(f"Response: {resp}")
        self.do_disconnect(None)

    def do_flash(self, arg):
        """Flash a binary file. Usage: flash <path_to_bin>"""
        if not arg:
            print("Error: Usage: flash <path_to_bin>")
            return
        
        filename = arg.strip()
        if not os.path.exists(filename):
            print(f"Error: File '{filename}' not found.")
            return

        if not self._ensure_connected(): return

        filesize = os.path.getsize(filename)
        print(f"Prepare to flash '{filename}' ({filesize} bytes)...")

        # 1. Open Data Socket
        try:
            self.data_sock = socket.create_connection((self.target_ip, DATA_PORT), timeout=self.timeout)
        except socket.error as e:
            print(f"Error connecting to data port: {e}")
            return

        # 2. Send BEGIN
        print("Handshake (BEGIN)...")
        resp = self._send_ctrl(f"BEGIN size={filesize} crc=0")
        if resp != "OK":
            print(f"Target refused handshake: {resp}")
            self.data_sock.close()
            self.data_sock = None
            return

        # 3. Stream Data
        print("Streaming firmware...")
        total_sent = 0
        start_time = time.time()
        
        try:
            with open(filename, "rb") as f:
                while True:
                    chunk = f.read(CHUNK_SIZE)
                    if not chunk:
                        break
                    self.data_sock.sendall(chunk)
                    total_sent += len(chunk)
                    # Simple progress bar
                    percent = int(total_sent * 100 / filesize)
                    sys.stdout.write(f"\rProgress: [{('=' * int(percent/5)).ljust(20)}] {percent}%")
                    sys.stdout.flush()
        except Exception as e:
            print(f"\nError sending data: {e}")
            self.do_disconnect(None)
            return

        duration = time.time() - start_time
        print(f"\nUpload complete. {total_sent} bytes in {duration:.2f}s ({total_sent/duration/1024:.2f} KB/s)")

        # 4. Monitor completion
        print("Verifying and Flashing...")
        while True:
            try:
                # Short monitoring loop
                resp = self._send_ctrl("STATUS?")
                # print(f"DEBUG status: {resp}")
                if not resp:
                    print("\nConnection closed by remote (Success).")
                    break
                time.sleep(0.5)
            except (socket.error, BrokenPipeError):
                print("\nConnection closed by remote (Success/Reboot).")
                break
        
        self.do_disconnect(None)

    def do_flash_esp(self, arg):
        """Flash ESP32 firmware. Usage: flash_esp <path_to_bin>"""
        if not arg:
            print("Error: Usage: flash_esp <path_to_bin>")
            return
        
        filename = arg.strip()
        if not os.path.exists(filename):
            print(f"Error: File '{filename}' not found.")
            return

        if not self._ensure_connected(): return

        filesize = os.path.getsize(filename)
        print(f"Prepare to flash ESP32 '{filename}' ({filesize} bytes)...")

        # 1. Open Data Socket
        try:
            self.data_sock = socket.create_connection((self.target_ip, DATA_PORT), timeout=self.timeout)
        except socket.error as e:
            print(f"Error connecting to data port: {e}")
            return

        # 2. Send BEGIN
        print("Handshake (BEGIN target=esp32)...")
        resp = self._send_ctrl(f"BEGIN size={filesize} crc=0 target=esp32")
        if resp != "OK":
            print(f"Target refused handshake: {resp}")
            self.data_sock.close()
            self.data_sock = None
            return

        # 3. Stream Data
        print("Streaming ESP32 firmware...")
        total_sent = 0
        start_time = time.time()
        
        try:
            with open(filename, "rb") as f:
                while True:
                    chunk = f.read(CHUNK_SIZE)
                    if not chunk:
                        break
                    self.data_sock.sendall(chunk)
                    total_sent += len(chunk)
                    # Simple progress bar
                    percent = int(total_sent * 100 / filesize)
                    sys.stdout.write(f"\rProgress: [{('=' * int(percent/5)).ljust(20)}] {percent}%")
                    sys.stdout.flush()
        except Exception as e:
            print(f"\nError sending data: {e}")
            self.do_disconnect(None)
            return

        duration = time.time() - start_time
        print(f"\nUpload complete. {total_sent} bytes in {duration:.2f}s ({total_sent/duration/1024:.2f} KB/s)")

        # 4. Monitor completion
        print("Verifying and Flashing...")
        while True:
            try:
                # Short monitoring loop
                resp = self._send_ctrl("STATUS?")
                # print(f"DEBUG status: {resp}")
                if not resp:
                    print("\nConnection closed by remote (Success).")
                    break
                time.sleep(0.5)
            except (socket.error, BrokenPipeError):
                print("\nConnection closed by remote (Success/Reboot).")
                break
        
        self.do_disconnect(None)

    def do_exit(self, arg):
        """Exit the shell."""
        self.do_disconnect(None)
        print("Bye!")
        return True

    def do_quit(self, arg):
        return self.do_exit(arg)

    # --- Helpers ---

    def do_monitor(self, arg):
        """Monitor data port (STM32 UART bridge). Ctrl+C to exit."""
        if not self._ensure_connected(): return

        print("Opening monitor on DATA port... (Ctrl+C to exit)")
        
        # 1. Open Data Socket
        try:
            self.data_sock = socket.create_connection((self.target_ip, DATA_PORT), timeout=self.timeout)
            self.data_sock.setblocking(False)
        except socket.error as e:
            print(f"Error connecting to data port: {e}")
            return
            
        import threading
        # 2. Start reader thread
        stop_event = threading.Event()
        
        def reader_thread():
            while not stop_event.is_set():
                try:
                    ready = select.select([self.data_sock], [], [], 0.1)
                    if ready[0]:
                        data = self.data_sock.recv(1024)
                        if not data:
                            break # Closed
                        sys.stdout.buffer.write(data)
                        sys.stdout.buffer.flush()
                except (socket.error, ValueError):
                    break
        
        import select
        t = threading.Thread(target=reader_thread, daemon=True)
        t.start()
        
        # 3. Writer loop (stdin)
        print("--- Monitor Active ---")
        try:
            while True:
                # Check for stdin input (non-blocking if possible, but python stdin is tricky)
                # We'll use select on stdin if on linux
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if r:
                    line = sys.stdin.readline()
                    if not line: break
                    self.data_sock.sendall(line.encode("utf-8"))
        except KeyboardInterrupt:
            print("\nExiting monitor...")
        except socket.error:
            print("\nSocket connection lost.")
        
        stop_event.set()
        t.join(timeout=1.0)
        self.data_sock.close()
        self.data_sock = None

    def _ensure_connected(self):
        if not self.connected:
            print("Not connected. Trying auto-connect...")
            self.do_connect(self.target_ip)
        return self.connected

    def _close_sockets(self):
        if self.ctrl_sock:
            try: self.ctrl_sock.close()
            except: pass
            self.ctrl_sock = None
        if self.data_sock:
            try: self.data_sock.close()
            except: pass
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
                    return None # Closed
                resp += chunk
                if chunk == b'\n':
                    break
            return resp.decode("ascii").strip()
        except socket.error as e:
            print(f"Socket error: {e}")
            self.do_disconnect(None)
            return None

def main():
    parser = argparse.ArgumentParser(description="ESP32/32raven Shell Client")
    parser.add_argument("ip", nargs='?', help="Target IP Address (Empty for auto-connect)")
    parser.add_argument("command", nargs='*', help="Run single command and exit")
    
    args = parser.parse_args()
    
    target_ip = args.ip
    
    # Auto-Connect Logic if IP not provided
    if not target_ip:
        # print("No IP provided. Checking WiFi...")
        ssid, dev = AutoConnector.get_current_wifi()
        
        if ssid == DEFAULT_SSID:
            print(f"Already connected to {DEFAULT_SSID} on {dev}.")
            ip = AutoConnector.get_gateway_ip(dev)
            if ip:
                # print(f"Detected Gateway IP: {ip}")
                target_ip = ip
            else:
                print(f"Could not failover Gateway IP, using default {DEFAULT_FALLBACK_IP}")
                target_ip = DEFAULT_FALLBACK_IP
        else:
            print(f"Not connected to {DEFAULT_SSID} (Currently: {ssid}). Attempting auto-connect...")
            if AutoConnector.connect_to_ssid(DEFAULT_SSID, DEFAULT_PASS):
                print("Connected! resolving IP...")
                # Re-check device and IP
                ssid, dev = AutoConnector.get_current_wifi()
                if dev:
                    time.sleep(2) # DHCP wait
                    ip = AutoConnector.get_gateway_ip(dev)
                    if ip:
                        print(f"Detected Gateway IP: {ip}")
                        target_ip = ip
                    else:
                        target_ip = DEFAULT_FALLBACK_IP
            else:
                print(f"Auto-connect failed. Falling back to default {DEFAULT_FALLBACK_IP}.")
                target_ip = DEFAULT_FALLBACK_IP

    # Check if a command is provided (one or more arguments)
    is_batch_mode = len(args.command) > 0
    
    shell = Esp32Shell(ip=target_ip)

    if is_batch_mode:
        line = " ".join(args.command)
        shell.onecmd("connect")
        if shell.connected:
            shell.onecmd(line)
        else:
            sys.exit(1)
    else:
        try:
            shell.cmdloop()
        except KeyboardInterrupt:
            print("\nInterrupted.")
            shell.do_disconnect(None)

if __name__ == "__main__":
    main()
