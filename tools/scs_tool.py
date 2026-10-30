#!/usr/bin/env python3
"""Talk to a Feetech SCS servo (e.g. Waveshare SC15) through the Bus Servo Adapter's USB port.

Set the adapter's jumper to B (USB mode) and unplug the ESP32's UART wires from the adapter first.

  scs_tool.py PORT scan                 find the servo's baud rate and ID (read-only)
  scs_tool.py PORT set-baud 115200      change the servo's stored baud rate

Needs pyserial. Register map as in Feetech's SCSCL library: ID 5, baud 6, EEPROM lock 48.
"""

import argparse
import sys
import time

import serial

BAUDS = {1000000: 0, 500000: 1, 250000: 2, 128000: 3, 115200: 4, 76800: 5, 57600: 6, 38400: 7}
REG_BAUD = 6
REG_LOCK = 48
REG_PRESENT_POSITION = 56
INST_PING, INST_READ, INST_WRITE = 0x01, 0x02, 0x03
BROADCAST = 0xFE


def packet(servo_id, instruction, params=()):
    body = [servo_id, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, ~sum(body) & 0xFF])


def transact(port, servo_id, instruction, params=(), reply_params=0, timeout=0.05):
    """Send one instruction; return (id, error, params) from the status packet, or None."""
    tx = packet(servo_id, instruction, params)
    port.reset_input_buffer()
    port.write(tx)
    port.flush()
    rx = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rx += port.read(port.in_waiting or 1)
    if rx.startswith(tx):  # half-duplex echo of our own packet
        rx = rx[len(tx):]
    n = reply_params + 6
    for i in range(len(rx) - n + 1):
        p = rx[i:i + n]
        if p[0] == p[1] == 0xFF and p[3] == reply_params + 2 and (~sum(p[2:-1]) & 0xFF) == p[-1]:
            return p[2], p[4], p[5:-1]
    return None


def open_port(name, baud):
    return serial.Serial(name, baud, timeout=0.01)


def scan(name):
    """Return (baud, id) of the first servo that answers a broadcast ping."""
    for baud in BAUDS:
        with open_port(name, baud) as port:
            reply = transact(port, BROADCAST, INST_PING)
        print(f"{baud:>8} baud: {'servo ID %d, error flags 0x%02X' % reply[:2] if reply else 'no reply'}")
        if reply:
            return baud, reply[0]
    return None


def write(port, servo_id, address, *values):
    reply = transact(port, servo_id, INST_WRITE, (address, *values))
    if reply is None:
        sys.exit(f"No reply writing register {address}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    sb = sub.add_parser("set-baud")
    sb.add_argument("baud", type=int, choices=sorted(BAUDS))
    args = ap.parse_args()

    found = scan(args.port)
    if not found:
        sys.exit("No servo answered. Check the jumper is on B, the servo is powered and plugged in.")
    baud, servo_id = found
    with open_port(args.port, baud) as port:
        pos = transact(port, servo_id, INST_READ, (REG_PRESENT_POSITION, 2), reply_params=2)
    if pos:
        print(f"Present position: {pos[2][0] << 8 | pos[2][1]}")
    if args.cmd == "scan":
        return

    if baud == args.baud:
        print(f"Servo {servo_id} is already at {baud} baud.")
        return
    with open_port(args.port, baud) as port:
        write(port, servo_id, REG_LOCK, 0)  # unlock EEPROM
        write(port, servo_id, REG_BAUD, BAUDS[args.baud])
    time.sleep(0.05)
    with open_port(args.port, args.baud) as port:
        write(port, servo_id, REG_LOCK, 1)  # lock EEPROM again, now at the new baud
        ok = transact(port, servo_id, INST_PING)
    print(f"Servo {servo_id} now answers at {args.baud} baud." if ok else "Servo did not answer at the new baud.")
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
