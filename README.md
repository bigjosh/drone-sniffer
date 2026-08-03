# Standalone Drone ID Sniffer

A self-contained drone Remote ID sniffer built on the [Colonel Panic Mesh-Detect](https://colonelpanic.tech/) carrier board — no phone, no laptop, no mesh required. Detections appear on the built-in OLED.

## Hardware

| Board | Role | USB port (varies) |
|---|---|---|
| Seeed XIAO ESP32-S3 | Sniffs WiFi + BLE for OpenDroneID broadcasts | COM6 |
| Heltec WiFi LoRa 32 V3 | Reads detections over UART, drives the OLED UI | COM3 |
| ESP32-C3 (bench only) | Optional test transmitter, see `test-transmitter/` | COM9 |

The Mesh-Detect carrier wires XIAO GPIO5 (TX) → Heltec GPIO19 (RX) at 115200 8N1. LoRa/Meshtastic is not used.

## Coverage

Both radios are used: the XIAO sniffs BLE and WiFi channel 6, and the Heltec — whose WiFi would otherwise sit idle — sweeps the remaining 2.4 GHz channels, so Beacon-method broadcasts away from channel 6 are caught too.

| Remote ID transport | Caught by | Notes |
|---|---|---|
| Bluetooth LE (legacy advertising) | XIAO | |
| WiFi NAN | XIAO | NAN is spec-locked to channel 6, so this is fully covered |
| WiFi Beacon, channel 6 | XIAO | |
| WiFi Beacon, channels 1–5, 7–11 | Heltec hopping scan | up to ~15 s to land on the right channel |
| Anything on 5 GHz | ✗ | both boards are 2.4 GHz only |
| Bluetooth 5 Long Range (Coded PHY) | ✗ (unverified) | the scanner uses the default legacy advertising scan |

## Firmware

### `xiao-firmware/` — detector (modified upstream)

Fork of [`colonelpanichacks/drone-mesh-mapper`](https://github.com/colonelpanichacks/drone-mesh-mapper) `remoteid-mesh-dualcore`. Detection engine is untouched (BLE advertisement scanning + WiFi promiscuous mode on channel 6). Changes:

- UART output is now one JSON line per detection, including the decoded UAS Basic ID (the stock firmware only sent MAC + a maps link):

  ```json
  {"id":"<UAS Basic ID>","mac":"aa:bb:cc:dd:ee:ff","rssi":-67,"tp":"wifi","ch":6,"dlat":38.8977,"dlon":-77.0365,"plat":38.8895,"plon":-77.0352}
  ```

  `id` may be empty (drone hasn't broadcast Basic ID yet); `tp` is `ble` or `wifi`; `ch` accompanies WiFi detections only; coordinate pairs are omitted until known.
- Rate limiting is per drone (1 s) instead of one global message per 5 s.
- Optional build flags: `-DTEST_EMIT` emits three synthetic drones on the UART every 3 s for integration testing without RF; `-DMIRROR_LINK_TO_USB` copies each link JSON line to USB serial so the exact bytes sent to the display can be inspected with the Heltec detached.

USB serial (115200) still prints the full detection JSON per the stock firmware.

### `heltec-firmware/` — display (new, replaces Meshtastic)

- Tracks up to 32 drones, keyed by UAS ID with MAC fallback (records merge when a MAC-only drone later reveals its ID).
- Records expire after **5 minutes** unseen; table-full evicts the least recently seen.
- OLED UI: header `n/m` (current record / total). With no drones: `0/0 — No drones detected`. Per record: last-seen age, ID (or MAC), transport + RSSI, pilot location.
- Screen orientation is set by `DISPLAY_ROTATION` at the top of `src/main.cpp`: `U8G2_R0` for the board's default, `U8G2_R2` to flip it 180°.
- Auto-advances every 4 s; PRG button advances immediately.
- White LED blinks on every received detection.
- USB serial (115200) accepts the same JSON lines as the UART — plus `dump`, `status`, `reset`, and `backdate N` commands — so the whole UI is testable from a PC.

**Second WiFi receiver.** The Heltec also sniffs WiFi itself, covering the channels the XIAO cannot:

- Sweeps channels 1–5 and 7–11 in a reshuffled order, 1.5 s dwell. Channel 6 is skipped because the XIAO sits there permanently.
- Receive-only (`WIFI_MODE_NULL`) and filtered to management frames in the driver, so it never transmits and doesn't take an interrupt per data frame.
- Decoding happens in the WiFi task and results are queued; `loop()` owns the drone table, so there is no lock on shared state.
- Records merge with XIAO detections by UAS ID, and the channel shown on the transport line identifies the source: **ch 6 = XIAO**, anything else = Heltec's own radio.
- Build with `-DENABLE_WIFI_SCAN=0` for the display-only firmware.

The channel shown is **the channel the receiver was tuned to**, not necessarily the drone's own channel. 2.4 GHz channels are ~20 MHz wide but spaced 5 MHz apart, so a strong transmitter is received on neighbouring channels as well — bench-verified: a channel 3 transmitter is picked up on channels 2, 3 and 4. Treat it as approximate, and as a reliable source indicator only when it reads exactly 6 versus clearly away from 6.

Note this adds Wi-Fi **Beacon** coverage only. Wi-Fi NAN Remote ID is spec-locked to channel 6, so the XIAO already captures all of it. Both boards are 2.4 GHz only — 5 GHz Remote ID is out of reach either way.

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

Flashing the Heltec overwrites Meshtastic. Dump the original flash **first** — it is the only way back to the existing install *and its settings*:

```powershell
python -m esptool --port COM3 --baud 921600 read-flash 0 ALL backups/heltec-meshtastic-full.bin   # before flashing
python -m esptool --port COM3 --baud 921600 write-flash 0 backups/heltec-meshtastic-full.bin      # to restore
```

`backups/` is git-ignored on purpose: a full flash image contains your node's private keys and channel configuration, so it should never be committed. Failing that, reflash from scratch with the [Meshtastic web flasher](https://flasher.meshtastic.org/) — you keep the firmware but lose the settings. Stock XIAO binaries are in the upstream repo's `firmware/` directory.

## Notes

- Both sniffer boards are receive-only — neither transmits. (`test-transmitter/` is a separate bench tool that does transmit, deliberately.)
- Timestamps are relative ("seen 37s ago") — no RTC on either board.

## Credits and licence

- Detection firmware is derived from [`colonelpanichacks/drone-mesh-mapper`](https://github.com/colonelpanichacks/drone-mesh-mapper) (MIT), on [Colonel Panic's Mesh-Detect](https://colonelpanic.tech/) hardware.
- `opendroneid.c/h`, `wifi.c`, `odid_wifi.h` are the [OpenDroneID reference library](https://github.com/opendroneid/opendroneid-core-c), © 2019 Intel Corporation, Apache-2.0. Original copyright headers are preserved in those files.
- Everything else here is MIT.
