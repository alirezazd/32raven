#!/usr/bin/env python3
import sys
import os
import subprocess
import re

def main():
    if len(sys.argv) < 3:
        print("Usage: esp32_size.py <map_file> <bin_file>")
        sys.exit(1)
    
    map_file = sys.argv[1]
    bin_file = sys.argv[2]
    
    # Get binary size
    bin_size = os.path.getsize(bin_file)
    
    # Run esp_idf_size and parse output
    try:
        result = subprocess.run(
            [sys.executable, "-m", "esp_idf_size", map_file],
            capture_output=True,
            text=True
        )
        output = result.stdout
        
        # Parse: "Used stat D/IRAM:  139966 bytes ( 181330 remain, 43.6% used)"
        dram_match = re.search(r"Used stat D/IRAM:\s+(\d+) bytes.*?(\d+) remain.*?([\d.]+)% used", output)
        
        if dram_match:
            dram_used = int(dram_match.group(1))
            dram_remain = int(dram_match.group(2))
            dram_total = dram_used + dram_remain
            dram_pct = float(dram_match.group(3))
        else:
            dram_used = 0
            dram_total = 327680
            dram_pct = 0.0
        
        # Flash: partition is ~1.8MB for app
        flash_max = 1945600  # 0x1db000 from partition table
        flash_pct = (bin_size / flash_max) * 100
        
        # Print table
        print("\n" + "="*65)
        print(f"{'Memory Region':<15} {'Used':>12} {'Total':>12} {'Percent':>10}")
        print("="*65)
        print(f"{'FLASH':<15} {bin_size:>12,} {flash_max:>12,} {flash_pct:>9.1f}%")
        print(f"{'RAM (DRAM)':<15} {dram_used:>12,} {dram_total:>12,} {dram_pct:>9.1f}%")
        print("="*65 + "\n")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
