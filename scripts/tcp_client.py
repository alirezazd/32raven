
import socket
import argparse
import struct
import time
import sys
import zlib

CTRL_PORT = 9000
DATA_PORT = 9001

def main():
    parser = argparse.ArgumentParser(description='TCP Flash Client')
    parser.add_argument('ip', help='ESP32 IP address')
    parser.add_argument('file', help='Binary file to upload')
    parser.add_argument('--chunk-size', type=int, default=1024, help='Upload chunk size')
    args = parser.parse_args()

    # Read file
    with open(args.file, 'rb') as f:
        data = f.read()
    
    total_size = len(data)
    crc = zlib.crc32(data) & 0xFFFFFFFF
    
    print(f"File: {args.file}, Size: {total_size}, CRC32: 0x{crc:08X}")

    try:
        # 1. Connect Control
        print(f"Connecting to {args.ip}:{CTRL_PORT} (CTRL)...")
        sk_ctrl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sk_ctrl.settimeout(5.0)
        sk_ctrl.connect((args.ip, CTRL_PORT))

        # 2. Connect Data
        print(f"Connecting to {args.ip}:{DATA_PORT} (DATA)...")
        sk_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sk_data.settimeout(5.0)
        sk_data.connect((args.ip, DATA_PORT))

        # 3. Send BEGIN
        cmd = f"BEGIN size={total_size} crc={crc}\n"
        print(f"Sending: {cmd.strip()}")
        sk_ctrl.sendall(cmd.encode('ascii'))

        # 4. Wait for OK
        resp = sk_ctrl.recv(1024).decode('ascii').strip()
        print(f"Response: {resp}")
        if resp != "OK":
            print("Error: Expected OK")
            return

        # 5. Send Data
        print("Sending data...")
        sent = 0
        start_time = time.time()
        
        while sent < total_size:
            chunk = data[sent : sent + args.chunk_size]
            sk_data.sendall(chunk)
            sent += len(chunk)
            
            # Optional: Check status every now and then?
            # For now just blast it.
            sys.stdout.write(f"\rProgress: {sent}/{total_size} ({sent/total_size*100:.1f}%)")
            sys.stdout.flush()

        duration = time.time() - start_time
        print(f"\nUpload complete in {duration:.2f}s ({total_size/duration/1024:.1f} KB/s)")
        
        # 6. Check status in a loop
        print("Waiting for server to process...")
        while True:
            sk_ctrl.sendall(b"STATUS?\n")
            resp = sk_ctrl.recv(1024).decode('ascii').strip()
            # Parse state=X
            # STATUS rx=0 total=15900 state=0 err=0
            # We want to see rx == total
            print(f"\r{resp}      ", end='')
            
            parts = dict(x.split('=') for x in resp.split()[1:])
            rx = int(parts.get('rx', 0))
            total = int(parts.get('total', 1))
            
            if rx >= total:
                print(f"\nSuccess! All bytes processed.")
                break
            
            time.sleep(0.5)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'sk_ctrl' in locals(): sk_ctrl.close()
        if 'sk_data' in locals(): sk_data.close()

if __name__ == '__main__':
    main()
