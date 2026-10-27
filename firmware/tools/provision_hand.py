#!/usr/bin/env python3
"""
Yeah Hand - factory provisioning (manufacturer use only).

For each new hand:
  1. generates a random password (cryptographically secure, 14 characters),
  2. programs serial number and password into the hand over the USB cable
     (FACTORY PROVISION, accepted only once per device),
  3. appends a line to the manufacturer's register (CSV),
  4. prints the text for the label.

The hand never reveals the password afterwards: keep the register safe
(e.g. on an encrypted disk), it is the only copy besides the label.

Usage:
  pip install pyserial
  python provision_hand.py --port /dev/ttyUSB0 --serial 2026-L-000001 [--register hands_register.csv]
  (Windows: --port COM5, macOS: --port /dev/tty.usbserial-XXXX)
"""

import argparse
import csv
import datetime
import os
import secrets
import sys
import time

import serial  # pyserial

# Unambiguous characters (no 0/O, 1/I/L), same rule as the firmware.
ALPHABET = "ABCDEFGHJKMNPQRSTUVWXYZ23456789"
PASSWORD_LEN = 14  # 14 x log2(31) = about 69 bit


def generate_password() -> str:
    return "".join(secrets.choice(ALPHABET) for _ in range(PASSWORD_LEN))


def send(port: serial.Serial, line: str, wait_s: float = 2.0) -> list[str]:
    port.reset_input_buffer()
    port.write((line + "\n").encode("ascii"))
    port.flush()
    lines, deadline = [], time.time() + wait_s
    while time.time() < deadline:
        raw = port.readline()
        if raw:
            text = raw.decode("ascii", errors="replace").strip()
            if text:
                lines.append(text)
                if text.startswith(("OK", "ERR", "FACTORY STATUS")):
                    break
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(description="Yeah Hand factory provisioning")
    ap.add_argument("--port", required=True, help="USB serial port of the hand")
    ap.add_argument("--serial", required=True, help="serial number YYYY-S-NNNNNN, S = L or R, e.g. 2026-L-000001 (label shows \"2026 L 000001\")")
    ap.add_argument("--register", default="hands_register.csv", help="CSV register of provisioned hands")
    args = ap.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.2) as port:
        # Opening the port resets the ESP32, which then calibrates the hand:
        # ask for the status until it answers (up to about 2 min).
        status = []
        for _ in range(60):
            status = send(port, "FACTORY STATUS")
            if any(s.startswith("FACTORY STATUS") for s in status):
                break
        print("\n".join(status))
        if not any("provisioned=no" in s for s in status):
            print("STOP: the hand is already provisioned (or not responding).", file=sys.stderr)
            return 1

        password = generate_password()
        result = send(port, f"FACTORY PROVISION {args.serial} {password}")
        print("\n".join(result))
        if not any(s.startswith("OK PROVISIONED") for s in result):
            print("STOP: provisioning failed, nothing written to the register.", file=sys.stderr)
            return 1

        # The hand restarts and calibrates again: wait for it.
        status = []
        for _ in range(60):
            status = send(port, "FACTORY STATUS")
            if any(s.startswith("FACTORY STATUS") for s in status):
                break
        print("\n".join(status))
        name = next((p.split("=", 1)[1] for s in status for p in s.split() if p.startswith("name=")), "")

    new_file = not os.path.exists(args.register)
    with open(args.register, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["serial", "bluetooth_name", "password", "provisioned_at", "customer", "invoice"])
        w.writerow([args.serial, name, password, datetime.datetime.now().isoformat(timespec="seconds"), "", ""])

    print("\n--- LABEL ---")
    print(f"Yeah Hand  S/N: {args.serial}")
    print(f"Bluetooth: {name}")
    print(f"Password:  {password}")
    print("-------------")
    print(f"Saved in {args.register}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
