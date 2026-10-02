# ESP32 + IKEA ICC-1 Zigbee → HomeKit Gateway

ESP-IDF firmware for an **ESP32** host that drives an **IKEA ICC-1 / ICC-A-1** Zigbee NCP (Silicon Labs EFR32MG1) and bridges Zigbee devices into **Apple HomeKit**.

The ESP32 is **not** the Zigbee radio. It speaks **EZSP over ASH over UART** to the ICC-1.

```
Phone / Home app / browser
        │
   Wi‑Fi (portal :80, HomeKit :8118)
        │
   ESP32 application (Wi‑Fi + HomeKit + portal)
        │
      EZSP → ASH → UART 115200 8N1
        │
   ICC-1 / EFR32MG1 NCP
        │
     Zigbee mesh
```

**Current firmware version:** `0.3.51` (see `PROJECT_VER` in `CMakeLists.txt`)

**Repo:** [ha0rex/esp32-icc1-zigbee-gw](https://github.com/ha0rex/esp32-icc1-zigbee-gw)

### Supported ESP32 boards

| Status | Target | Notes |
| --- | --- | --- |
| **Tested** | **Seeed XIAO ESP32-S3 (Plus)** | Recommended. Enough DRAM/PSRAM for HomeKit + portal; UART defaults **GPIO2 TX / GPIO4 RX** |
| Supported in-tree | ESP32-C3 Super Mini | Builds and OTA published, but **unstable in practice** (DRAM exhaustion / panic resets under HomeKit load). Prefer S3. |
| Other ESP32 | Any IDF-supported chip | Should work if you set the **UART GPIOs**, flash size, and `idf.py set-target` for your board. No extra board support is required beyond Kconfig / `sdkconfig.defaults.<target>`. |

One tree for all targets. Select the chip with `idf.py set-target …`. The portal **Overview** and **System → Diagnostics** show board name, chip id, and UART GPIOs. **OTA publishes per-target binaries**; the portal installs the image that matches the running chip (`esp32s3`, `esp32c3`, …).

---

## Features

| Area | What it does |
| --- | --- |
| **Zigbee coordinator** | Form / leave network, permit join, persistent device inventory (NVS), ZCL interview |
| **Device types** | Climate sensors, lights, outlets, switches, irrigation valves, contact/motion/leak/smoke (IAS), IKEA remotes |
| **HomeKit bridge** | Opt-in exposure of devices and groups; setup code `111-22-333` |
| **Remotes** | HomeKit buttons (stateless/stateful) **or** direct control of lights/switches/groups |
| **Grouped devices** | General groups + **temperature** thermostats (optional heater/cooler, gap & hysteresis, humidity-forced heat) or **humidity** regulators; **After power failure** on general groups (mains power-on/brownout only — not flash/USB reboot) |
| **Portal** | SoftAP + home Wi‑Fi management (scan APs by BSSID for mesh, primary + secondary fallback) |
| **Sniffer** | Live Zigbee RX/TX/EVT log with friendly device names |
| **OTA** | Dual-slot update with **Stable** / **Nightly** channels; CI builds each target and the portal picks the matching binary |
| **Backup** | **System → Backup & migrate** exports names / HomeKit / remotes / groups (Zigbee pairings stay on the ICC) |

---

## Hardware

### ICC-1 wiring (common)

| ESP32 | ICC-1 |
| --- | --- |
| **UART TX** | pad **2** / **PB15** / RX |
| **UART RX** | pad **3** / **PB14** / TX |
| **3.3V** | pad **11** / VDD |
| **GND** | pad **12** / GND |

- UART: **115200 8N1**, no RTS/CTS / reset / SWD assumed for day-to-day use
- **Never power the ICC from 5V**
- Pick any free GPIOs on your board; set them in `sdkconfig` / menuconfig (`ICC_UART_TX_GPIO` / `ICC_UART_RX_GPIO`)

### Tested defaults (in-tree)

| Board | TX | RX | Flash / notes |
| --- | --- | --- | --- |
| **XIAO ESP32-S3 (Plus)** | GPIO**2** (D1) | GPIO**4** (D3) | 8–16MB; octal PSRAM when present. Do **not** use GPIO3 for RX (strapping). |
| ESP32-C3 Super Mini | GPIO**6** | GPIO**7** | 4MB. Worked for bring-up but **ran out of DRAM** with HomeKit — not recommended. |

Defaults live in `sdkconfig.defaults.esp32s3` / `sdkconfig.defaults.esp32c3`.

### ICC NCP firmware

The ICC must already run EmberZNet NCP firmware — see **[Flashing the ICC NCP](docs/icc-ncp-flash.md)** for SWD wiring, Simplicity Commander steps, and pinout. Example image:

`NCP_USW_115k2_F256_678_PB14-PB15-PA0.s37`

- EmberZNet **6.7.8.x**
- EZSP protocol **v8**
- ASH over UART @ **115200 8N1** (NCP UART on **PB14/PB15**)

---

## Quick start

### Build

```bash
. $HOME/esp/esp-idf/export.sh   # ESP-IDF v5.x / v6.x
cd /path/to/esp32-icc1-zigbee-gw
idf.py set-target esp32s3       # recommended; or esp32c3 / your chip
idf.py build
```

Switching target: `idf.py fullclean` then `idf.py set-target …` again.

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
3. **Scan** access points (mesh nodes appear as the same SSID with different BSSIDs) → pick the **strongest** (highest / least-negative dBm) → password → **Save & connect**. Prefer ≥ about **-70 dBm**; **-85 dBm** is already marginal.
4. SoftAP turns off after a successful join. SoftAP recovery uses a **compact setup page** (full portal is too large for SoftAP TX) and keeps APSTA so Scan does not flip radio modes. On a healthy RSSI link the firmware does not soft-reconnect for idle/TCP probe timeouts; it recovers associated-but-silent STA via **gateway ARP** misses after portal/HAP have been quiet, and roams when RSSI stays below about **−80 dBm**.
5. On home Wi‑Fi: **System → Wi‑Fi** — **Scan**, then **Set primary** / **Set secondary** on an AP row, enter passwords, **Save & connect** once. Leave secondary SSID blank to clear it on save.

Secondary is tried only after primary exhausts reconnect retries.

### HomeKit

1. Wait ~12 s after boot (HomeKit start is deferred so Wi‑Fi can settle)
2. Home app → **Add Accessory** → **More options** → **ICC Gateway**
3. Setup code: **`111-22-333`** (setup ID `ICC1`)
4. HAP runs on port **8118**; the portal stays on **80**

Expose devices from the **Devices** tab (`Expose to HomeKit`). Remotes in “Control a device” mode are **not** HomeKit accessories.

---

## Migrating between ESP boards (keep Zigbee devices)

Zigbee pairings and the mesh live on the **ICC-1**, not the ESP. ESP NVS holds friendly names, HomeKit expose flags, remote modes, and groups.

1. On the **old ESP**, export a backup:
   - Portal **System → Backup & migrate → Download backup**, or `./tools/backup-from-gateway.sh <ip> icc-backup.json`
   - **Without ICC attached (USB):** `./tools/backup-from-gateway.sh --usb icc-backup.json` — reads the NVS partition over serial (devices are not shown in `/api/status` until ASH connects)
2. Flash and set up the **new ESP** (Wi‑Fi SoftAP / home network).
3. Power down, move the **same ICC-1** (UART + 3.3V + GND) onto the new board’s wiring.
4. On the new portal: wait until devices reappear from the NCP (or restore registers them), then **System → Restore backup…** and pick the JSON.
5. Re-add **HomeKit** on the new board (pairing is per-ESP); room/names for bridged accessories may need a one-time touch-up in Home.

Do **not** form a new Zigbee network on the new ESP if the ICC still holds the old mesh.

---

## Portal

| Tab | Purpose |
| --- | --- |
| **Overview** | Board type, ICC/ASH/EZSP health, network, Wi‑Fi, HomeKit code |
| **Zigbee** | Form / leave network, channel & TX power, permit join |
| **Devices** | Inventory (live value chips without redrawing rows), rename, interview, remove, HomeKit, remote config, button test — tap a row for details |
| **Grouped devices** | General groups; temperature thermostats (heater and/or cooler, gap/hysteresis, optional humidity-forced heat); humidity regulators (±3 % RH) — tap a row to edit |
| **Sniffer** | Live frames with `[device name]` labels; filter RX/TX/EVT |
| **System** | Wi‑Fi, firmware OTA (Stable / Nightly, **board-matched** binary), diagnostics (board + heap), backup/restore, forget Wi‑Fi |

---

## Devices & remotes

Supported kinds (portal chip + HomeKit when exposed):

| Kind | Zigbee signal | HomeKit |
| --- | --- | --- |
| **Light** | Level Control / light models | Lightbulb (On/Off + brightness) |
| **Outlet** | Plug models (`S31`, `BASICZBR3`, …) | Outlet |
| **Switch** | Generic On/Off | Switch |
| **Irrigation** | Valve / Sonoff **SWV** / irrigation models | Irrigation System + Valve (open/close = On/Off; no native schedules) |
| **Sensor** | Temp / humidity | Temperature + Humidity (+ battery); optional wake button (None / SPS / Switch) |
| **Contact / Motion / Leak / Smoke** | IAS Zone (or Occupancy for PIR) | Matching HomeKit sensor |
| **Remote** | IKEA buttons | Programmable switches or control mode |

**Sensors** (Sonoff SNZB-02 / SNZB-02D / TH01, etc.): temperature, humidity, battery → HomeKit. Classic SNZB-02 (TI `00:12:4b`) is a sleepy end device — **Read values** queues one ZCL frame until the next poll; press the sensor button shortly after so it can check in. Kind detection prefers temp/humidity (and climate model IDs) over contact name fingerprints, so names like **Outdoors** are not mistaken for door/contact sensors. Bridged HomeKit AIDs are stable per Zigbee EUI (not per kind). The bridged accessory **kind is sticky in NVS** so reboots do not rebuild as a different service type (which made Home reject room/name edits). A one-time heal still rewrites former Contact tiles that are actually climate sensors (same AID — set room/name once after that). After create, the bridge does **not** push Name updates (Home owns room/custom name). Plugs/switches ignore On/Off attribute echoes for a few seconds after a HomeKit write so the UI does not flip back (e.g. CK-BL702).

**SNZB-02D wake button:** the physical button sends Zigbee OnOff Toggle. In the device edit modal, **Switch type** is **None** (default — no HomeKit button accessory), **Stateless** (programmable switch), or **Stateful** (On/Off that toggles on each press). If that sensor is the temperature sensor of a **Grouped thermostat**, each press still raises the target by **0.5 °C** (wraps 38→10 °C) and best-effort writes Sonoff **0xFC11** (`0x600E=external`, then `0x600D` = °C×100) for the small **EXT1** LCD area (large digits stay local). Delivery is confirmed via `messageSent`; failures retry on the next poll. Official Z2M docs expose these attrs mainly on **SNZB-02DR2**; community reports SNZB-02D ~1.0.4+.

Device **kinds are compile-time** in firmware. Only accessories you expose allocate HomeKit objects on the heap. Heap pressure scales with live inventory, bridged accessories, and portal/HAP buffers — a common failure mode on the **ESP32-C3**. Prefer **ESP32-S3** (or another chip with more internal RAM / PSRAM).

**Sonoff SWV:** pairs as Irrigation; Active in Home opens/closes the valve. Flow metering and eWeLink schedules are not bridged.

**Lights / outlets / switches:** On/Off; lights also map brightness (Level Control).

**IKEA remotes** (Tradfri 5-button, STYRBAR, shortcut, etc.):

| Mode | Behaviour |
| --- | --- |
| **HomeKit buttons** | Each button → programmable switch and/or stateful On/Off in Home; names editable. Presses notify HomeKit immediately (duplicate Zigbee echoes from one physical press are filtered; intentional re-presses are not muted for seconds). |
| **Control a device** | Power (and related) presses toggle selected lights / switches / groups on the Zigbee side; **not** exposed to HomeKit |

**Touchlink (control mode):** after join, hold the remote **≤5 cm from a real bulb** until the bulb flashes. The gateway learns that group and removes the lights from it so presses go to the gateway only. Holding the remote toward the gateway alone does **not** bind.

Limits: up to **16** devices, **8** groups × **8** members, **5** buttons per remote, **8** control targets.

---

## Firmware updates

On **System**, change the update channel dropdown (saved immediately), then **Check for update → Install update**.

CI builds **each** published target (`esp32s3`, `esp32c3`, …). The browser reads the manifest and installs **`targets.<chip>.url`** for the running board (legacy top-level `url` remains a C3 fallback for older portals).

| Channel | What you get |
| --- | --- |
| **Stable** | Production builds from `main` |
| **Nightly** | Newer experimental builds from `dev` |

The browser fetches the manifest/firmware from GitHub, then uploads the image to the gateway over local HTTP. Keep power applied during install; the gateway reboots when done.

Check compares **semantic** version numbers: downgrades are never offered. If you are already newer than the selected channel, the portal shows up to date.

---

## Configuration

See `sdkconfig.defaults` plus target files:

- `sdkconfig.defaults.esp32s3` — GPIO2/4, flash/PSRAM for XIAO S3 Plus
- `sdkconfig.defaults.esp32c3` — GPIO6/7, 4MB flash (legacy)
- Portal `:80`, HomeKit `:8118`, setup code / setup ID
- Dual OTA partitions (`ota_0` / `ota_1`, ~1.6 MiB each)
- Wi‑Fi AMPDU disabled (avoids silent STA on some APs)

---

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| RX bytes = 0 / no RSTACK | TX↔RX crossed? Common GND? ICC on **3.3V**? NCP image flashed? On S3, RX on **GPIO4**, not GPIO3? |
| CRC / garbage | Baud **115200**; correct 115k2 NCP build (PB14/PB15) |
| Portal dies, ping fails, USB still up | “Silent STA” or mesh **client isolation**. Firmware recovers via gateway ARP silence (after quiet) and weak-RSSI roam (&lt; about −80 dBm). Overview shows **Last reset**. Prefer pinning ≥ about −70 dBm. |
| Panic reboot / low min heap | Typical on **ESP32-C3** under HomeKit. Move to **ESP32-S3** (or another roomier chip). Overview shows **Free / Min / Largest** heap and **Board**. |
| HomeKit “No Response” | Wait for deferred start; confirm `:8118`; try lock→unlock once after large inventory changes |
| Remote does nothing in Home | Mode = HomeKit buttons? Exposed? For control mode, complete Touchlink to a **bulb** |
| Sensor stuck / empty readings | **Read values**, then press the sensor button within a few seconds (sleepy devices only receive while polling) |
| OTA check fails | Browser can reach GitHub? Correct channel? Manifest has a URL for **this board’s chip**? |

### Serial success snapshot (XIAO ESP32-S3)

```
esp32s3 + ICC-1 Zigbee NCP gateway
UART TX=GPIO2 RX=GPIO4 @ 115200
ASH: connected
EZSP: protocol version 8
EmberZNet: 6.7.8.x
Network: JOINED
ICC status: CONNECTED
```

---

## License note

Protocol implementations are original host-side code based on public documentation. Third-party component licenses (ESP-IDF, HomeKit ADK derivatives in-tree, etc.) apply as marked in those trees.
