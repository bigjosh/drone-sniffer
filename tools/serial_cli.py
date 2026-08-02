#!/usr/bin/env python3
"""Bench-test helper: send lines to a serial port and echo everything received.

Opens the port without asserting DTR/RTS so the ESP32 is NOT reset on connect,
letting device state persist across invocations.

Examples:
  python tools/serial_cli.py COM3 --send '{"id":"X","mac":"aa:bb:cc:00:00:01","rssi":-60,"tp":"ble"}' --send dump
  python tools/serial_cli.py COM3 --send "backdate 290" --send wait:15 --send dump
  python tools/serial_cli.py COM6 --listen 10
"""
import argparse
import time

import serial


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument(
        "--send",
        action="append",
        default=[],
        metavar="LINE",
        help="line to send (repeatable, in order); 'wait:N' sleeps N seconds while echoing output",
    )
    p.add_argument(
        "--listen",
        type=float,
        default=3.0,
        help="seconds to keep reading after the last send (default 3)",
    )
    a = p.parse_args()

    s = serial.Serial()
    s.port = a.port
    s.baudrate = a.baud
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()

    def drain() -> None:
        while True:
            raw = s.readline()
            if not raw:
                break
            print(raw.decode(errors="replace").rstrip())

    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        for item in a.send:
            if item.startswith("wait:"):
                end = time.time() + float(item[5:])
                while time.time() < end:
                    drain()
                continue
            s.write(item.encode() + b"\n")
            s.flush()
            time.sleep(0.15)
            drain()
        end = time.time() + a.listen
        while time.time() < end:
            drain()
    finally:
        s.close()


if __name__ == "__main__":
    main()
