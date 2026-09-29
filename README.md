# ESP32-C3 + IKEA ICC-1 Zigbee → HomeKit Gateway

ESP-IDF firmware for an **ESP32-C3 Super Mini** that hosts an **IKEA ICC-1 / ICC-A-1** Zigbee NCP (Silicon Labs EFR32MG1) and bridges Zigbee devices into **Apple HomeKit**.

The ESP32-C3 is **not** the Zigbee radio. It speaks **EZSP over ASH over UART** to the ICC-1.

```
Phone / Home app / browser
        │
   Wi‑Fi (portal :80, HomeKit :8118)
        │
   ESP32-C3 application
        │
      EZSP → ASH → UART 115200 8N1
        │
   ICC-1 / EFR32MG1 NCP
        │
     Zigbee mesh
```

**Current firmware version:** `0.3.0` (see `PROJECT_VER` in `CMakeLists.txt`)

**Repo:** [ha0rex/esp32-icc1-zigbee-gw](https://github.com/ha0rex/esp32-icc1-zigbee-gw)

---

## Features

| Area | What it does |
| --- | --- |
| **Zigbee coordinator** | Form / leave network, permit join, persistent device inventory (NVS), ZCL interview |
| **Device types** | Temperature/humidity sensors, lights (On/Off + brightness), plugs/switches, IKEA remotes |
| **HomeKit bridge** | Opt-in exposure of devices and groups; setup code `111-22-333` |
| **Remotes** | HomeKit buttons (stateless/stateful) **or** direct control of lights/switches/groups |
| **Grouped devices** | Combine members into one HomeKit accessory; thermostat mode (sensor + heater, ±0.5 °C) |
| **Portal** | SoftAP Wi‑Fi setup + full management UI on home Wi‑Fi |
| **Sniffer** | Live Zigbee RX/TX/EVT log with friendly device names |
| **OTA** | Dual-slot HTTPS update from GitHub `main` (rolling `ota` release) |

---

## Hardware

| ESP32-C3 Super Mini | ICC-1 |
| --- | --- |
| **GPIO6 TX** | pad **2** / **PB15** / RX |
| **GPIO7 RX** | pad **3** / **PB14** / TX |
| **3.3V** | pad **11** / VDD |
| **GND** | pad **12** / GND |

- Flash: **4MB**
- UART: **115200 8N1**, no RTS/CTS / reset / SWD assumed
- **Never power the ICC from 5V**

### ICC NCP firmware

The ICC must already run EmberZNet NCP firmware, for example:

`NCP_USW_115k2_F256_678_PB14-PB15-PA0.s37`

- EmberZNet **6.7.8.x**
- EZSP protocol **v8**
- ASH over UART @ **115200 8N1**

---

## Quick start

### Build

```bash
. $HOME/esp/esp-idf/export.sh   # ESP-IDF v5.x / v6.x
cd /path/to/esp32-icc1-zigbee-gw
idf.py set-target esp32c3
idf.py build
```

### First flash (dual OTA partitions)

The partition table uses two OTA app slots. After changing to this layout (or on a blank chip):

```bash
idf.py -p /dev/cu.usbmodem* erase-flash
idf.py -p /dev/cu.usbmodem* flash monitor
```

Later app-only flashes: `idf.py -p PORT flash`. Exit monitor with `Ctrl+]`.

### Wi‑Fi setup

1. On first boot (no STA credentials), join SoftAP **`ICC1-Gateway-XXXX`**
2. Open **http://192.168.4.1/**
3. Scan / enter home SSID + password → **Save & connect**
4. SoftAP turns off after a successful join (APSTA starves ESP32-C3 TX)
5. Portal then lives at **http://\<home-ip\>/** — use **System → Forget home Wi‑Fi** to return to setup mode

### HomeKit

1. Wait ~12 s after boot (HomeKit start is deferred so Wi‑Fi can settle)
2. Home app → **Add Accessory** → **More options** → **ICC Gateway**
3. Setup code: **`111-22-333`** (setup ID `ICC1`)
4. HAP runs on port **8118**; the portal stays on **80**

Expose devices from the **Devices** tab (`Expose to HomeKit`). Remotes in “Control a device” mode are **not** HomeKit accessories.

---

## Portal

| Tab | Purpose |
| --- | --- |
| **Overview** | ICC/ASH/EZSP health, network, Wi‑Fi, HomeKit code |
| **Zigbee** | Form / leave network, channel & TX power, permit join |
| **Devices** | Inventory, rename, interview, remove, HomeKit, remote config, button test |
| **Grouped devices** | General groups (one light/switch) or thermostats (sensor + heater) |
| **Sniffer** | Live frames with `[device name]` labels; filter RX/TX/EVT |
| **System** | Firmware OTA, diagnostics, forget Wi‑Fi |

Useful APIs: `/api/status`, `/api/ping`, `/api/ota`, `/api/zigbee/sniff`.

---

## Devices & remotes

**Sensors** (e.g. Sonoff SNZB-02D): temperature, humidity, battery → HomeKit. After join, use **Read values** and press the sensor button a few times while awake so reporting can be configured.

**Lights / switches:** On/Off; lights also map brightness (Level Control).

**IKEA remotes** (Tradfri 5-button, STYRBAR, shortcut, etc.):

| Mode | Behaviour |
| --- | --- |
| **HomeKit buttons** | Each button → programmable switch and/or stateful On/Off in Home; names editable |
| **Control a device** | Power (and related) presses toggle selected lights / switches / groups on the Zigbee side; **not** exposed to HomeKit |

**Touchlink (control mode):** after join, hold the remote **≤5 cm from a real bulb** until the bulb flashes. The gateway learns that group and removes the lights from it so presses go to the gateway only. Holding the remote toward the gateway alone does **not** bind.

Limits: up to **32** devices, **8** groups × **8** members, **5** buttons per remote, **8** control targets.

---

## OTA updates (`main` channel)

Pushes to **`main`** (firmware changes) build with the `espressif/idf:latest` image and refresh a rolling GitHub Release tagged **`ota`**:

- Manifest: https://github.com/ha0rex/esp32-icc1-zigbee-gw/releases/download/ota/manifest.json
- Binary: https://github.com/ha0rex/esp32-icc1-zigbee-gw/releases/download/ota/esp32_icc1_zigbee_gw.bin

On the device (home Wi‑Fi): **System → Check for update → Install update**. The gateway reboots into the new slot. Keep power applied during the download.

**Branches**

| Branch | Role |
| --- | --- |
| **`main`** | Production + OTA CI |
| **`dev`** | Day-to-day development (merge to `main` when ready to ship OTA) |

---

## Architecture

| Component | Role |
| --- | --- |
| `icc_uart` | UART transport |
| `ash` | Silicon Labs ASH framing |
| `ezsp` | EZSP v8 + sniff ring + Touchlink/multicast helpers |
| `zigbee_host` | Network lifecycle, device DB, remotes, sensor path |
| `thermostat` | Grouped devices + local thermostat loop |
| `homekit_bridge` | HAP bridge + bridged accessories / EVENTs |
| `wifi_manager` | SoftAP provisioning + STA (C3-safe AMPDU/HT off) |
| `web` | Embedded portal + JSON APIs |
| `fw_ota` | HTTPS OTA from GitHub `ota` release |

---

## Configuration highlights

See `sdkconfig.defaults`:

- Portal `:80`, HomeKit `:8118`, setup code / setup ID
- Dual OTA `partitions.csv` (`ota_0` / `ota_1`, ~1.6 MiB each)
- Wi‑Fi AMPDU disabled (avoids silent STA on some APs)
- ASH/EZSP hex dumps off by default (enable in menuconfig for ICC bring-up)

---

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| RX bytes = 0 / no RSTACK | TX↔RX crossed? Common GND? ICC on **3.3V**? NCP image flashed? |
| CRC / garbage | Baud **115200**; correct 115k2 NCP build |
| Portal dies, ping fails, USB still up | Classic C3 silent STA — power-cycle or USB reset; SoftAP should not stay up beside STA |
| HomeKit “No Response” | Wait for deferred start; confirm `:8118`; try lock→unlock once after large inventory changes |
| Remote does nothing in Home | Mode = HomeKit buttons? Exposed? For control mode, complete Touchlink to a **bulb** |
| Sensor stuck / empty readings | **Read values** + wake the sleepy end device with its button |
| OTA check fails | Home STA online? GitHub reachable? First CI `ota` release published from `main`? |

### Serial success snapshot

```
ICC-1 Zigbee NCP
UART: GPIO6 TX / GPIO7 RX @ 115200
ASH: connected
EZSP: protocol version 8
EmberZNet: 6.7.8.x
Network: JOINED
ICC status: CONNECTED
```

---

## Development notes

- Prefer feature work on **`dev`**; promote to **`main`** for OTA.
- Keep **`README.md` in sync** with firmware behaviour when you change features, APIs, partitions, ports, or workflows (see `.cursor/rules/readme-sync.mdc`).
- ASH/EZSP behaviour follows Silicon Labs public protocol docs — do not paste GPL host stacks into this tree.

## License note

Protocol implementations are original host-side code based on public documentation. Third-party component licenses (ESP-IDF, HomeKit ADK derivatives in-tree, etc.) apply as marked in those trees.
