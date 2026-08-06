#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AD15 UART OTA upgrade test (fast ACK)."""
from __future__ import annotations
import argparse, struct, sys, time
from pathlib import Path
try:
    import serial
except ImportError:
    print("need pyserial"); sys.exit(1)

FLAG = 0x4658
HEAD_LEN = 16
CHUNK = 512
OTA_INIT, OTA_RECIVE, OTA_FAIL, OTA_SUCCESS = 0x01, 0x02, 0x03, 0x04
OTA_READ_VERSION = 0x05

def checksum_frame(buf):
    return sum(b for i,b in enumerate(buf) if i not in (8,9)) & 0xFFFF

def build_frame(cmd, payload=b"", cur=0, total=1):
    body_len = 1 + len(payload)
    head = bytearray(HEAD_LEN)
    struct.pack_into("<HHHHH", head, 0, FLAG, body_len, cur, total, 0)
    frame = bytearray(head)
    frame.append(cmd & 0xFF)
    frame.extend(payload)
    chk = checksum_frame(frame)
    frame[8] = chk & 0xFF
    frame[9] = (chk >> 8) & 0xFF
    return bytes(frame)

def hexdump(data, limit=48):
    s = " ".join(f"{b:02X}" for b in data[:limit])
    if len(data) > limit:
        s += f" ...(+{len(data)-limit})"
    return s

def parse_frames(buf):
    out = []
    i = 0
    while i + HEAD_LEN + 1 <= len(buf):
        flag, length, cur, total, chk = struct.unpack_from("<HHHHH", buf, i)
        if flag != FLAG:
            i += 1
            continue
        need = HEAD_LEN + length
        if i + need > len(buf):
            break
        raw = buf[i:i+need]
        if checksum_frame(raw) != chk:
            i += 1
            continue
        out.append((raw[HEAD_LEN], raw[HEAD_LEN+1:need], raw))
        i += need
    return out

def open_port(port, baud):
    ser = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N",
                        stopbits=1, timeout=0.02, write_timeout=2.0)
    try:
        ser.reset_input_buffer(); ser.reset_output_buffer()
    except Exception:
        pass
    return ser

def wait_cmd(ser, want_cmds, wait_s, label=""):
    """Return as soon as a wanted cmd frame is parsed."""
    deadline = time.time() + wait_s
    rx = bytearray()
    while time.time() < deadline:
        chunk = ser.read(1024)
        if chunk:
            rx.extend(chunk)
            frames = parse_frames(bytes(rx))
            for c, p, _ in frames:
                if (c & 0x7F) in want_cmds or c in want_cmds:
                    if label:
                        print(f"  {label} RX({len(rx)}): {hexdump(rx)}")
                        print(f"    cmd=0x{c:02X} pay={hexdump(p,16)}")
                    return c, p, bytes(rx)
        else:
            time.sleep(0.001)
    if label:
        print(f"  {label} RX({len(rx)}): {hexdump(rx)} (timeout)")
    return None, b"", bytes(rx)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="COM11")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-f", "--file", required=True)
    ap.add_argument("--chunk", type=int, default=CHUNK)
    ap.add_argument("--init-wait", type=float, default=15.0)
    ap.add_argument("--pkg-wait", type=float, default=0.5)
    ap.add_argument("--final-wait", type=float, default=5.0)
    args = ap.parse_args()

    path = Path(args.file)
    data = path.read_bytes()
    chunk = args.chunk
    if chunk <= 0 or chunk > 512:
        print("chunk must be 1..1024"); return 1
    pkgs = [data[i:i+chunk] for i in range(0, len(data), chunk)]
    total = len(pkgs)
    print(f"file={path} size={len(data)} chunk={chunk} pkgs={total}")

    ser = open_port(args.port, args.baud)
    try:
        print("\n=== probe VERSION ===")
        ser.write(build_frame(OTA_READ_VERSION)); ser.flush()
        c,p,_ = wait_cmd(ser, {OTA_READ_VERSION}, 1.0, "VERSION")
        if c is not None:
            print(f"  version={p!r}")
        else:
            print("  WARN: no version reply")

        print("\n=== OTA_INIT ===")
        init = build_frame(OTA_INIT, struct.pack("<I", len(data)))
        print(f"  TX: {hexdump(init)}")
        ser.write(init); ser.flush()
        c,p,_ = wait_cmd(ser, {OTA_INIT}, args.init_wait, "INIT")
        if c is None or not p or p[0] != 0x00:
            print(f"FAIL: OTA_INIT c={c} pay={p.hex() if p else None}"); return 3
        print("  OTA_INIT OK")

        print("\n=== OTA_RECIVE ===")
        t0 = time.time()
        for i in range(total):
            wait = args.final_wait if i == total - 1 else args.pkg_wait
            frame = build_frame(OTA_RECIVE, pkgs[i], cur=i, total=total)
            ser.write(frame); ser.flush()
            c,p,rx = wait_cmd(ser, {OTA_RECIVE, OTA_SUCCESS, OTA_FAIL}, wait)
            if c is None:
                print(f"FAIL: pkg {i}/{total} timeout elapsed={time.time()-t0:.1f}s")
                return 4
            status = p[0] if p else 0xFF
            base = c & 0x7F
            if base == OTA_RECIVE and status != 0x00:
                print(f"FAIL: pkg {i}/{total} NACK 0x{status:02X}"); return 5
            if (i % 16 == 0) or (i == total - 1) or base in (OTA_SUCCESS, OTA_FAIL):
                rate = ((i+1)*chunk)/(time.time()-t0+1e-6)/1024
                print(f"  pkg {i+1}/{total} cmd=0x{c:02X} st=0x{status:02X} {time.time()-t0:.1f}s {rate:.1f}KB/s")
            if base == OTA_SUCCESS:
                print(f"SUCCESS: OTA_SUCCESS in {time.time()-t0:.1f}s"); break
            if base == OTA_FAIL:
                print("FAIL: OTA_FAIL"); return 6
            if i == total - 1:
                # last RECIVE ack may be followed by SUCCESS in same or next RX
                frames = parse_frames(rx)
                cmds = [x[0] & 0x7F for x in frames]
                if OTA_SUCCESS in cmds:
                    print(f"SUCCESS: OTA_SUCCESS in last RX, {time.time()-t0:.1f}s")
                else:
                    c2,p2,_ = wait_cmd(ser, {OTA_SUCCESS, OTA_FAIL}, args.final_wait, "FINAL")
                    if c2 is not None and (c2 & 0x7F) == OTA_SUCCESS:
                        print(f"SUCCESS: OTA_SUCCESS after last, {time.time()-t0:.1f}s")
                    elif c2 is not None and (c2 & 0x7F) == OTA_FAIL:
                        print("FAIL: OTA_FAIL after last"); return 6
                    else:
                        print("WARN: last ACK ok, no SUCCESS seen (device may have reset)")

        time.sleep(2.0)
        print("\n=== probe VERSION after OTA ===")
        try: ser.close()
        except Exception: pass
        time.sleep(1.5)
        ser = open_port(args.port, args.baud)
        ser.write(build_frame(OTA_READ_VERSION)); ser.flush()
        c,p,_ = wait_cmd(ser, {OTA_READ_VERSION}, 2.0, "VERSION")
        print(f"  version={p!r}" if c is not None else "  WARN: no version after reset")
        return 0
    finally:
        try: ser.close()
        except Exception: pass

if __name__ == "__main__":
    raise SystemExit(main())

