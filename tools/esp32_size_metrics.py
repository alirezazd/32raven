#!/usr/bin/env python3
from __future__ import annotations
import argparse, pathlib, re, struct, sys

PATTERNS = {
    "diram": r"^\s*dram0_0_seg\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+\S+\s*$",
    "iram": r"^\.iram0\.text\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s*$",
    "data": r"^\.dram0\.data\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s*$",
    "bss": r"^\.dram0\.bss\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s*$",
    "rtc_ram": r"^\s*rtc_iram_seg\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+\S+\s*$",
    "rtc_reserved": r"^\s*rtc_reserved_seg\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+\S+\s*$",
    "rtc_slow": r"^\s*0x([0-9a-fA-F]+)\s+_rtc_slow_length\b",
    "rtc_fast": r"^\s*0x([0-9a-fA-F]+)\s+_rtc_fast_length\b",
    "rtc_used_reserved": r"^\s*0x([0-9a-fA-F]+)\s+_rtc_reserved_length\b",
}

def main() -> int:
    root = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description="Print ESP32 RAM, flash-app, and RTC usage.")
    ap.add_argument("--build-dir", default=str(root / "build/Ninja/esp32"))
    build = pathlib.Path(ap.parse_args().build_dir).resolve()
    maps = sorted(build.glob("*.map"))
    if len(maps) != 1: raise SystemExit(f"expected exactly one .map in {build}")
    map_text = maps[0].read_text(encoding="utf-8", errors="replace")
    get = lambda key: int(re.search(PATTERNS[key], map_text, re.MULTILINE).group(1), 16)
    dram = (get("iram") + get("data") + get("bss"), get("diram"))
    rtc = (get("rtc_slow") + get("rtc_fast") + get("rtc_used_reserved"), get("rtc_ram") + get("rtc_reserved"))
    app_bin = maps[0].with_suffix(".bin")
    raw = (build / "partition_table/partition-table.bin").read_bytes()
    app_sizes = []
    for i in range(0, len(raw), 32):
        entry = raw[i : i + 32]
        if entry == b"\xff" * 32: break
        if entry[:2] == b"\xeb\xeb": continue
        magic, part_type, _subtype, _offset, size, _name, _flags = struct.unpack("<2sBBLL16sL", entry)
        if magic != b"\xaa\x50": raise SystemExit("invalid partition table entry")
        if part_type == 0: app_sizes.append(size)
    flash = (app_bin.stat().st_size, min(app_sizes))
    pct = lambda used, total: f"{used * 100.0 / total:.2f}%"
    print()
    print(f"ESP RAM Usage: {dram[0]} bytes / {dram[1]} bytes {pct(*dram)}")
    print(f"ESP Flash-App Usage: {flash[0]} bytes / {flash[1]} bytes {pct(*flash)}")
    print(f"ESP RTC-RAM Usage: {rtc[0]} bytes / {rtc[1]} bytes {pct(*rtc)}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
