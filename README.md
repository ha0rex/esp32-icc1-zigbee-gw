# ESP32-C3 + IKEA ICC-1 Zigbee NCP Gateway

Production-oriented **ESP-IDF** firmware for an **ESP32-C3 Super Mini** acting as an EZSP host for an **IKEA ICC-1 / ICC-A-1** Zigbee NCP (Silicon Labs EFR32MG1).

This is **not** ESP-Zigbee radio mode. The ESP32-C3 is the host MCU; the ICC-1 is the Zigbee radio.

```
ESP32 application
        │
       EZSP
        │
       ASH
        │
 UART 115200 8N1
        │
ICC-1 / EFR32MG1 NCP
        │
     Zigbee
```

## Current milestone

Reliable ICC bring-up:

1. UART open (GPIO6 TX / GPIO7 RX @ 115200)
2. ASH reset → RSTACK handshake
3. EZSP version negotiation (protocol v8)
4. Read EUI64 + network state
5. Wi-Fi station (optional) + embedded HTTP status page

HomeKit bridging and Zigbee network formation are intentionally **not** implemented yet.

## Hardware

| ESP32-C3 Super Mini | ICC-1 |
| --- | --- |
| **GPIO6 TX** | pad **2** / **PB15** / RX |
| **GPIO7 RX** | pad **3** / **PB14** / TX |
| **3.3V** | pad **11** / VDD |
| **GND** | pad **12** / GND |

**NEVER connect the ICC module to 5V.**

No RTS/CTS, reset, bootloader, or SWD lines are assumed.

### ICC firmware requirement

The ICC must already run EmberZNet NCP firmware, for example:

`NCP_USW_115k2_F256_678_PB14-PB15-PA0.s37`

- EmberZNet **6.7.8.x**
- EZSP protocol **v8**
- ASH over UART @ **115200 8N1**, no hardware flow control

## Architecture

| Component | Role |
| --- | --- |
| `icc_uart` | Async UART transport + byte counters |
| `ash` | Silicon Labs ASH framing (CRC, stuffing, LFSR randomize, RST/RSTACK/DATA/ACK/NAK) |
| `ezsp` / `ezsp_v8` | EZSP command layer (version, getEui64, networkState, nop) |
| `zigbee_host` | Bring-up orchestration + status snapshot |
| `wifi_manager` | Optional STA (menuconfig credentials) |
| `web` | Lightweight HTTP status UI |

## Build

```bash
. $HOME/esp/esp-idf/export.sh   # or your IDF install path
cd /path/to/esp32-icc1-zigbee-gw
idf.py set-target esp32c3
idf.py build
```

### Configure Wi-Fi (SoftAP portal)

On first boot the device creates an open SoftAP named like `ICC1-Gateway-XXXX`.

1. Join that network from your phone/laptop
2. Open **http://192.168.4.1/**
3. Pick/enter your home Wi-Fi SSID + password → **Save & connect**
4. Credentials are stored in NVS and reused after reboot
5. SoftAP stays available so you can reconfigure; use **Forget home Wi-Fi** to return to setup-only mode

Optional menuconfig SSID/password only seeds NVS when empty (dev convenience). Prefer the portal.

## Flash / monitor

```bash
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

On macOS the port is often `/dev/cu.usbmodem*` or `/dev/cu.wchusbserial*`.

Exit monitor with `Ctrl+]`.

## Expected serial output (success)

```
------------------------------------------------
ICC-1 Zigbee NCP
------------------------------------------------
UART: GPIO6 TX / GPIO7 RX @ 115200
ASH: connected
EZSP: protocol version 8
EmberZNet: 6.7.8.x
EUI64: xx:xx:xx:xx:xx:xx:xx:xx
ICC status: CONNECTED
------------------------------------------------
```

With `CONFIG_ASH_LOG_HEX_FRAMES=y` and log level Debug you will also see lines like:

```
TX ASH: 1A 1A 1A 1A C0 38 BC 7E
RX ASH: C1 02 02 9B 7B 7E
```

## Expected output (no ICC / wiring fault)

```
ASH reset sent
waiting for RSTACK...
timeout
RX bytes received: 0
ICC status: NOT CONNECTED
```

The firmware retries every few seconds and does not crash.

## Web UI

When Wi-Fi is connected, open `http://<device-ip>/`.

Also available: `http://<device-ip>/api/status` (JSON).

## Troubleshooting

| Symptom | Checks |
| --- | --- |
| RX bytes = 0 | Wiring TX↔RX crossed correctly? Common GND? ICC powered from **3.3V**? |
| Garbage CRC errors | Baud must be 115200; confirm NCP image is the 115k2 build |
| RSTACK never arrives | Confirm NCP firmware; try power-cycle ICC while monitor is open |
| EZSP timeout after ASH OK | Capture Debug hex dumps of TX/RX ASH frames |
| Wi-Fi fails | Ignored for ICC testing — set SSID in menuconfig |

### Debug dump to share if ICC does not answer

1. Set Component config → Log output → Default log verbosity → **Debug**
2. Ensure `ASH: Log ASH TX/RX frames as hex` is enabled
3. Flash + monitor for ~30 seconds
4. Send the log containing: `TX ASH`, `RX ASH`, `RX bytes received`, any CRC warnings

## Roadmap

1. ~~UART~~ / ~~ASH~~ / ~~EZSP version + EUI64~~ / ~~Wi-Fi status UI~~
2. Zigbee network formation + NVS backup
3. Permit join + device discovery + persistent device DB
4. ZCL attribute / reporting path
5. Apple HomeKit bridge mapping Zigbee devices → HAP services

## License note

ASH/EZSP behaviour is implemented from Silicon Labs public protocol documentation. Do not paste GPL-licensed host stacks into this tree.
