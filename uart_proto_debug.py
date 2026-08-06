#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AD15 UART protocol debug tool (COM port)."""
from __future__ import annotations
import argparse, struct, sys, time
try:
    import serial
except ImportError:
    print("need pyserial: pip install pyserial"); sys.exit(1)

FLAG = 0x4658
HEAD_LEN = 16
CMD = {
    "OTA_INIT": 0x01, "OTA_RECIVE": 0x02, "OTA_FAIL": 0x03, "OTA_SUCCESS": 0x04,
    "OTA_READ_VERSION": 0x05, "GET_UUID": 0x06, "GET_SN": 0x07, "SET_SN": 0x08,
    "IR_ONOFF": 0x09, "SR_ONOFF": 0x0A, "IR_STATUS": 0x0B, "SR_SET_STATUS": 0x0C,
}
CMD_NAME = {v: k for k, v in CMD.items()}

def checksum_frame(buf):
    s = 0
    for i, b in enumerate(buf):
        if i in (8, 9):
            continue
        s += b
    return s & 0xFFFF

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
        cmd = raw[HEAD_LEN]
        payload = raw[HEAD_LEN+1:need]
        out.append((cmd, payload, raw))
        i += need
    return out

def hexdump(data):
    return " ".join(f"{b:02X}" for b in data)

def open_port(port, baud):
    ser = serial.Serial(port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                        timeout=0.05, write_timeout=1.0)
    try:
        ser.reset_input_buffer(); ser.reset_output_buffer()
    except Exception:
        pass
    return ser

def transact(ser, frame, wait_s=0.8):
    print(f"TX ({len(frame)}): {hexdump(frame)}")
    ser.write(frame); ser.flush()
    deadline = time.time() + wait_s
    rx = bytearray()
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            rx.extend(chunk)
            deadline = max(deadline, time.time() + 0.15)
        else:
            time.sleep(0.01)
    return bytes(rx)

def show_rx(rx):
    if not rx:
        print("RX: (empty / timeout)"); return
    print(f"RX ({len(rx)}): {hexdump(rx)}")
    for cmd, payload, raw in parse_frames(rx):
        name = CMD_NAME.get(cmd & 0x7F, f"UNK_{cmd:02X}")
        reply = " (reply|0x80)" if (cmd & 0x80) else ""
        print(f"  frame cmd=0x{cmd:02X} {name}{reply} payload[{len(payload)}]={hexdump(payload)}")
        if (cmd & 0x7F) == CMD["GET_UUID"]:
            print(f"    UUID: {payload!r}")
        elif (cmd & 0x7F) == CMD["GET_SN"]:
            print(f"    SN: {payload!r}")
        elif (cmd & 0x7F) == CMD["OTA_READ_VERSION"]:
            print(f"    VERSION: {payload!r}")
        elif (cmd & 0x7F) == CMD["SR_SET_STATUS"] and len(payload) >= 4:
            print(f"    angle={struct.unpack_from('<I', payload, 0)[0]}")

def cmd_payload(name, args):
    if name == "SET_SN":
        sn = (args.sn or "AD15TESTSN00000000000000000001").encode("ascii")
        return (sn + b"\x00"*32)[:32]
    if name == "SR_SET_STATUS":
        return struct.pack("<I", int(args.angle))
    if name == "OTA_INIT":
        return struct.pack("<I", int(args.ota_size))
    return b""

def main():
    ap = argparse.ArgumentParser(description="AD15 UART protocol debug")
    ap.add_argument("-p", "--port", default="COM11")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-c", "--cmd", action="append",
                    choices=list(CMD.keys()) + ["ALL_QUERY"])
    ap.add_argument("--angle", type=int, default=90)
    ap.add_argument("--sn", default=None)
    ap.add_argument("--ota-size", type=int, default=4096)
    ap.add_argument("-w", "--wait", type=float, default=0.8)
    ap.add_argument("--listen", type=float, default=0.0)
    args = ap.parse_args()
    cmds = args.cmd or ["ALL_QUERY"]
    if "ALL_QUERY" in cmds:
        cmds = ["OTA_READ_VERSION","GET_UUID","GET_SN","IR_ONOFF","SR_ONOFF","IR_STATUS"]
    print(f"open {args.port} @ {args.baud}")
    try:
        ser = open_port(args.port, args.baud)
    except serial.SerialException as e:
        print(f"open failed: {e}"); return 1
    try:
        for name in cmds:
            frame = build_frame(CMD[name], cmd_payload(name, args))
            print(f"\n=== {name} (0x{CMD[name]:02X}) ===")
            show_rx(transact(ser, frame, wait_s=args.wait))
            time.sleep(0.05)
        if args.listen > 0:
            print(f"\n=== listen {args.listen}s ===")
            deadline = time.time() + args.listen
            rx = bytearray()
            while time.time() < deadline:
                chunk = ser.read(256)
                if chunk: rx.extend(chunk)
            show_rx(bytes(rx))
    finally:
        ser.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
