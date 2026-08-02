# Standalone Drone ID Sniffer

A self-contained drone Remote ID sniffer built on the [Colonel Panic Mesh-Detect](https://colonelpanic.tech/) carrier board — no phone, no laptop, no mesh required. Detections appear on the built-in OLED.

## Hardware

| Board | Role | USB port (varies) |
|---|---|---|
| Seeed XIAO ESP32-S3 | Sniffs WiFi + BLE for OpenDroneID broadcasts | COM6 |
| Heltec WiFi LoRa 32 V3 | Reads detections over UART, drives the OLED UI | COM3 |
| ESP32-C3 (bench only) | Optional test transmitter, see `test-transmitter/` | COM9 |

The Mesh-Detect carrier wires XIAO GPIO5 (TX) → Heltec GPIO19 (RX) at 115200 8N1. LoRa/Meshtastic is not used.

## Firmware

### `xiao-firmware/` — detector (modified upstream)

Fork of [`colonelpanichacks/drone-mesh-mapper`](https://github.com/colonelpanichacks/drone-mesh-mapper) `remoteid-mesh-dualcore`. Detection engine is untouched (BLE advertisement scanning + WiFi promiscuous mode on channel 6). Changes:

- UART output is now one JSON line per detection, including the decoded UAS Basic ID (the stock firmware only sent MAC + a maps link):

  ```json
  {"id":"<UAS Basic ID>","mac":"aa:bb:cc:dd:ee:ff","rssi":-67,"tp":"ble","dlat":38.8977,"dlon":-77.0365,"plat":38.8895,"plon":-77.0352}
  ```

  `id` may be empty (drone hasn't broadcast Basic ID yet); `tp` is `ble` or `wifi`; coordinate pairs are omitted until known.
- Rate limiting is per drone (1 s) instead of one global message per 5 s.
- Optional build flags: `-DTEST_EMIT` emits three synthetic drones on the UART every 3 s for integration testing without RF; `-DMIRROR_LINK_TO_USB` copies each link JSON line to USB serial so the exact bytes sent to the display can be inspected with the Heltec detached.

USB serial (115200) still prints the full detection JSON per the stock firmware.

### `heltec-firmware/` — display (new, replaces Meshtastic)

- Tracks up to 32 drones, keyed by UAS ID with MAC fallback (records merge when a MAC-only drone later reveals its ID).
- Records expire after **5 minutes** unseen; table-full evicts the least recently seen.
- OLED UI: header `n/m` (current record / total). With no drones: `0/0 — No drones detected`. Per record: last-seen age, ID (or MAC), transport + RSSI, pilot location.
- Screen is rotated 180° (`DISPLAY_ROTATION` at the top of `src/main.cpp` — set `U8G2_R0` for the default orientation).
- Auto-advances every 4 s; PRG button advances immediately.
- White LED blinks on every received detection.
- USB serial (115200) accepts the same JSON lines as the UART — plus `dump`, `reset`, and `backdate N` commands — so the whole UI is testable from a PC.

### `test-transmitter/` — bench test drone (optional)

Broadcasts two synthetic drones so the receiver can be validated without a real aircraft, one per transport so the two detection paths are distinguishable:

| Transport | UAS ID | Pilot location |
|---|---|---|
| WiFi Beacon, channel 6 | `BENCHTEST0000001` | 45.0010, −93.0010 |
| BLE advertisement | `BENCHTEST0000002` | 45.0110, −93.0110 |

Both are built with the same OpenDroneID reference library the receiver decodes with, so the frames are standard-compliant by construction. WiFi sends a full message pack once per second; BLE legacy advertisements hold only one 25-byte message, so Basic ID / Location / System / Operator ID are cycled every 250 ms. Each drone flies a slow 55 m circle so successive packets differ.

Targets an ESP32-C3 (`huge_app.csv` partition — WiFi + BLE together exceed the default app partition). Verified against a real Remote ID scanner app as well as this receiver.

> Bench use only. Keep it indoors at low power and don't leave it running — any Remote ID receiver in range will report these as aircraft.

## Building & flashing

```powershell
python -m platformio run -d xiao-firmware     -t upload --upload-port COM6
python -m platformio run -d heltec-firmware   -t upload --upload-port COM3
python -m platformio run -d test-transmitter  -t upload --upload-port COM9   # optional
python -m platformio device monitor -p COM3 -b 115200   # watch the display node
```

Bench testing without RF — `tools/serial_cli.py` opens a port without asserting DTR/RTS, so the board is not reset on connect:

```powershell
python tools/serial_cli.py COM3 --send '{"id":"X","mac":"aa:bb:cc:00:00:01","rssi":-60,"tp":"ble"}' --send dump
python tools/serial_cli.py COM3 --send 'backdate 290' --send wait:15 --send dump   # expiry test
```

## Restoring Meshtastic on the Heltec

A byte-exact dump of the original flash (Meshtastic + settings) is in `backups/heltec-meshtastic-full.bin`:

```powershell
python -m esptool --port COM3 --baud 921600 write-flash 0 backups/heltec-meshtastic-full.bin
```

(Or flash fresh via the [Meshtastic web flasher](https://flasher.meshtastic.org/).) Stock XIAO binaries live in `reference/drone-mesh-mapper/firmware/`.

## Notes

- Receive-only: nothing transmits. BLE scanning + WiFi promiscuous listening.
- WiFi Remote ID detection is fixed on channel 6 (upstream behavior); BLE catches the common broadcast path.
- Timestamps are relative ("seen 37s ago") — no RTC on either board.
