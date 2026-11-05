#!/usr/bin/env python3
"""
Yeah Hand - functional EMC / Bluetooth endurance test (manufacturer use).

Opens and closes the hand every --period seconds over the Bluetooth serial
port, checks that every command is acknowledged ("OK CLOSURE n") and logs
every event to a CSV file. At the end it prints a summary to copy into the
conformity assessment (table "Prove funzionali del fabbricante").

A lost link is logged, then the script reconnects and authenticates again,
so a long test does not stop at the first problem.

Usage:
  pip install pyserial
  python emc_endurance_test.py --port /dev/rfcomm0 --password XXXXXXXXXXXXXX --hours 8 --label "continuo 8h"
  (Windows: --port COM7, the "outgoing" Bluetooth COM port of the paired hand)
Every --temp-every minutes it also reads the internal temperature of the five
servos (command INFO) and logs it: useful for the overheating test.
Stop early with Ctrl+C: the summary is printed anyway.
"""

import argparse
import csv
import datetime
import sys
import time

import serial  # pyserial

ACK_TIMEOUT_S = 2.0


def now() -> str:
    return datetime.datetime.now().isoformat(timespec="seconds")


class Hand:
    def __init__(self, port: str, password: str):
        self.port_name = port
        self.password = password
        self.ser = None
        self.calibrated_now = False  # True if the last connect() had to calibrate

    def connect(self) -> bool:
        """Open, authenticate and, if the hand reports NOT CALIBRATED (fresh power-on
        or reset), send CALIBRATE and wait for it to finish."""
        self.calibrated_now = False
        try:
            self.close()
            self.ser = serial.Serial(self.port_name, 115200, timeout=0.1)
            time.sleep(1.0)
            self.ser.reset_input_buffer()
            self.ser.write(f"AUTH {self.password}\n".encode("ascii"))
            self.ser.flush()
            authed, not_calibrated = False, False
            deadline = time.time() + ACK_TIMEOUT_S
            while time.time() < deadline:
                raw = self.ser.readline()
                text = raw.decode("ascii", errors="replace").strip() if raw else ""
                if text.startswith("OK AUTH"):
                    authed = True
                    deadline = time.time() + 1.0  # the NOT CALIBRATED notice follows
                elif text.startswith("INFO NOT CALIBRATED"):
                    not_calibrated = True
                    break
            if not authed:
                return False
            if not_calibrated:
                self.ser.write(b"CALIBRATE\n")
                self.ser.flush()
                deadline = time.time() + 150  # calibration takes up to about 90 s
                while time.time() < deadline:
                    raw = self.ser.readline()
                    if raw and raw.decode("ascii", errors="replace").strip().startswith("OK CALIBRATED"):
                        self.calibrated_now = True
                        return True
                return False
            return True
        except (serial.SerialException, OSError):
            return False

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None

    def servo_temperature(self, servo_id: int):
        """Internal temperature (deg C) reported by one servo via INFO, or None."""
        self.ser.reset_input_buffer()
        self.ser.write(f"INFO {servo_id}\n".encode("ascii"))
        self.ser.flush()
        deadline = time.time() + ACK_TIMEOUT_S
        while time.time() < deadline:
            raw = self.ser.readline()
            text = raw.decode("ascii", errors="replace").strip() if raw else ""
            if text.startswith(f"ID:{servo_id},"):
                for part in text.split(","):
                    part = part.strip()
                    if part.startswith("T:"):
                        return int(part[2:])
        return None

    def command(self, line: str, expect: str) -> bool:
        """Send a line, wait for a reply starting with `expect`."""
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()
        deadline = time.time() + ACK_TIMEOUT_S
        while time.time() < deadline:
            raw = self.ser.readline()
            if raw and raw.decode("ascii", errors="replace").strip().startswith(expect):
                return True
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Yeah Hand functional EMC endurance test")
    ap.add_argument("--port", required=True, help="Bluetooth serial port of the paired hand")
    ap.add_argument("--password", required=True, help="hand password (label)")
    ap.add_argument("--hours", type=float, default=8.0, help="test duration in hours")
    ap.add_argument("--period", type=float, default=2.0, help="seconds between open and close")
    ap.add_argument("--closure", type=int, default=100, help="closure percent when closed (0-100)")
    ap.add_argument("--label", default="", help="test name written in the log, e.g. 'distanza 10 m'")
    ap.add_argument("--temp-every", type=float, default=5.0, help="minutes between servo temperature readings (0 = off)")
    ap.add_argument("--log", default=None, help="CSV log file (default: emc_test_<timestamp>.csv)")
    args = ap.parse_args()

    log_name = args.log or f"emc_test_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
    hand = Hand(args.port, args.password)
    stats = {"commands": 0, "missed_acks": 0, "link_losses": 0, "resets": 0, "longest_outage_s": 0.0}
    max_temp = {}  # servo id -> highest temperature seen
    next_temp = time.time()
    start = time.time()
    end = start + args.hours * 3600

    with open(log_name, "w", newline="", encoding="utf-8") as f:
        log = csv.writer(f)
        log.writerow(["time", "event", "detail"])
        log.writerow([now(), "START", f"{args.label} port={args.port} hours={args.hours} period={args.period}"])

        if not hand.connect():
            print("STOP: cannot connect or authenticate (paired? right port? right password?)", file=sys.stderr)
            log.writerow([now(), "ABORT", "first connection failed"])
            return 1
        if hand.calibrated_now:
            log.writerow([now(), "CALIBRATED", "hand calibrated after power-on"])

        closed = False
        try:
            while time.time() < end:
                target = 0 if closed else args.closure
                try:
                    ok = hand.command(str(target), "OK CLOSURE")
                except (serial.SerialException, OSError) as e:
                    ok = None
                    log.writerow([now(), "LINK_LOST", str(e)])
                stats["commands"] += 1

                if ok is False:
                    stats["missed_acks"] += 1
                    log.writerow([now(), "NO_ACK", f"closure {target}"])
                if ok is None or (ok is False and stats["missed_acks"] % 3 == 0):
                    # Link gone (or three missed replies): reconnect until it works.
                    stats["link_losses"] += 1
                    lost_at = time.time()
                    print(f"{now()} link lost, reconnecting...")
                    while time.time() < end and not hand.connect():
                        time.sleep(5)
                    outage = time.time() - lost_at
                    stats["longest_outage_s"] = max(stats["longest_outage_s"], outage)
                    log.writerow([now(), "RECONNECTED", f"outage {outage:.0f} s"])
                    if hand.calibrated_now:
                        # The hand had lost its calibration: it restarted (reset or power loss).
                        stats["resets"] += 1
                        log.writerow([now(), "RESET_DETECTED", "hand reported NOT CALIBRATED, recalibrated"])
                        print(f"{now()} RESET detected: the hand had restarted, recalibrated")
                else:
                    closed = not closed

                if args.temp_every > 0 and time.time() >= next_temp:
                    next_temp = time.time() + args.temp_every * 60
                    temps = []
                    for sid in range(1, 6):
                        try:
                            t = hand.servo_temperature(sid)
                        except (serial.SerialException, OSError):
                            t = None
                        temps.append(t)
                        if t is not None:
                            max_temp[sid] = max(max_temp.get(sid, t), t)
                    log.writerow([now(), "TEMP", " ".join(f"servo{i + 1}={t}" for i, t in enumerate(temps))])
                    print(f"{now()} servo temperatures (C): {temps}")

                elapsed_h = (time.time() - start) / 3600
                if stats["commands"] % 300 == 0:
                    print(f"{now()} {elapsed_h:.2f} h, {stats['commands']} commands, "
                          f"{stats['missed_acks']} missed, {stats['link_losses']} link losses")
                    f.flush()
                time.sleep(args.period)
        except KeyboardInterrupt:
            log.writerow([now(), "STOPPED", "Ctrl+C"])

        duration_h = (time.time() - start) / 3600
        log.writerow([now(), "END", str(stats)])
    hand.close()

    print("\n--- SUMMARY ---")
    print(f"Test:            {args.label}")
    print(f"Duration:        {duration_h:.2f} h")
    print(f"Commands sent:   {stats['commands']}")
    print(f"Missed replies:  {stats['missed_acks']}")
    print(f"Link losses:     {stats['link_losses']} (longest outage {stats['longest_outage_s']:.0f} s)")
    print(f"Hand resets:     {stats['resets']}")
    if max_temp:
        print("Max servo temp:  " + ", ".join(f"servo {k}: {v} C" for k, v in sorted(max_temp.items())))
    print(f"Log:             {log_name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
