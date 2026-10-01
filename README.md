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

**Current firmware version:** `0.3.37` (see `PROJECT_VER` in `CMakeLists.txt`)

**Repo:** [ha0rex/esp32-icc1-zigbee-gw](https://github.com/ha0rex/esp32-icc1-zigbee-gw)

---

## Features

| Area | What it does |
| --- | --- |
| **Zigbee coordinator** | Form / leave network, permit join, persistent device inventory (NVS), ZCL interview |
| **Device types** | Climate sensors, lights, outlets, switches, irrigation valves, contact/motion/leak/smoke (IAS), IKEA remotes |
| **HomeKit bridge** | Opt-in exposure of devices and groups; setup code `111-22-333` |
| **Remotes** | HomeKit buttons (stateless/stateful) **or** direct control of lights/switches/groups |
| **Grouped devices** | General groups + **temperature** thermostats (optional heater/cooler, gap & hysteresis, humidity-forced heat) or **humidity** regulators (humidifier/dehumidifier); **After power failure** on general groups (mains power-on/brownout only — not flash/USB reboot) |
| **Portal** | SoftAP + home Wi‑Fi management (scan APs by BSSID for mesh, primary + secondary fallback) |
| **Sniffer** | Live Zigbee RX/TX/EVT log with friendly device names |
| **OTA** | Dual-slot update with **Stable** / **Nightly** channels (choice remembered on the device) |

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

## Portal

| Tab | Purpose |
| --- | --- |
| **Overview** | ICC/ASH/EZSP health, network, Wi‑Fi, HomeKit code |
| **Zigbee** | Form / leave network, channel & TX power, permit join |
| **Devices** | Inventory (live value chips without redrawing rows), rename, interview, remove, HomeKit, remote config, button test — tap a row for details |
| **Grouped devices** | General groups; temperature thermostats (heater and/or cooler, gap/hysteresis, optional humidity-forced heat); humidity regulators (±3 % RH) — tap a row to edit |
| **Sniffer** | Live frames with `[device name]` labels; filter RX/TX/EVT |
| **System** | Wi‑Fi (AP list → set primary/secondary, one Save), firmware OTA (Stable / Nightly), diagnostics, forget Wi‑Fi |

---

## Devices & remotes

Supported kinds (portal chip + HomeKit when exposed):

| Kind | Zigbee signal | HomeKit |
| --- | --- | --- |
| **Light** | Level Control / light models | Lightbulb (On/Off + brightness) |
| **Outlet** | Plug models (`S31`, `BASICZBR3`, …) | Outlet |
| **Switch** | Generic On/Off | Switch |
| **Irrigation** | Valve / Sonoff **SWV** / irrigation models | Irrigation System + Valve (open/close = On/Off; no native schedules) |
| **Sensor** | Temp / humidity | Temperature + Humidity (+ battery) |
| **Contact / Motion / Leak / Smoke** | IAS Zone (or Occupancy for PIR) | Matching HomeKit sensor |
| **Remote** | IKEA buttons | Programmable switches or control mode |

**Sensors** (Sonoff SNZB-02 / SNZB-02D / TH01, etc.): temperature, humidity, battery → HomeKit. Classic SNZB-02 (TI `00:12:4b`) is a sleepy end device — **Read values** queues one ZCL frame until the next poll; press the sensor button shortly after so it can check in. Kind detection prefers temp/humidity (and climate model IDs) over contact name fingerprints, so names like **Outdoors** are not mistaken for door/contact sensors. Bridged HomeKit AIDs are stable per Zigbee EUI (not per kind). The bridged accessory **kind is sticky in NVS** so reboots do not rebuild as a different service type (which made Home reject room/name edits). A one-time heal still rewrites former Contact tiles that are actually climate sensors (same AID — set room/name once after that). After create, the bridge does **not** push Name updates (Home owns room/custom name). Plugs/switches ignore On/Off attribute echoes for a few seconds after a HomeKit write so the UI does not flip back (e.g. CK-BL702).

Device **kinds are compile-time** in firmware (flash code). Only accessories you expose allocate HomeKit objects on the heap — unused kinds (e.g. irrigation when you have no valve) do **not** reserve RAM. Downloading per-model “drop-ins” from GitHub would **not** free heap on the ESP32-C3: TLS download buffers and a runtime loader cost more DRAM than the small classifier/HAP create paths they would replace. Heap pressure comes from the live device table, bridged accessory slots, task stacks, and portal/HAP buffers (trimmed in **0.3.37**).

**Sonoff SWV:** pairs as Irrigation; Active in Home opens/closes the valve. Flow metering and eWeLink schedules are not bridged.

**Lights / outlets / switches:** On/Off; lights also map brightness (Level Control).

**IKEA remotes** (Tradfri 5-button, STYRBAR, shortcut, etc.):

| Mode | Behaviour |
| --- | --- |
| **HomeKit buttons** | Each button → programmable switch and/or stateful On/Off in Home; names editable |
| **Control a device** | Power (and related) presses toggle selected lights / switches / groups on the Zigbee side; **not** exposed to HomeKit |

**Touchlink (control mode):** after join, hold the remote **≤5 cm from a real bulb** until the bulb flashes. The gateway learns that group and removes the lights from it so presses go to the gateway only. Holding the remote toward the gateway alone does **not** bind.

Limits: up to **16** devices, **8** groups × **8** members, **5** buttons per remote, **8** control targets.

---

## Firmware updates

On **System**, change the update channel dropdown (saved immediately), then **Check for update → Install update**.

When an update is available the portal shows a **Changelog** from git commits since the previous publish on that channel.

| Channel | What you get |
| --- | --- |
| **Stable** | Production builds from `main` |
| **Nightly** | Newer experimental builds from `dev` |

The browser fetches the manifest/firmware from GitHub, then uploads the image to the gateway over local HTTP (the ESP32-C3 does not run HTTPS itself). Keep power applied during install; the gateway reboots when done.

Check compares **semantic** version numbers: downgrades are never offered. If you are already newer than the selected channel, the portal shows up to date.

---

## Configuration

See `sdkconfig.defaults` for defaults such as:

- Portal `:80`, HomeKit `:8118`, setup code / setup ID
- Dual OTA partitions (`ota_0` / `ota_1`, ~1.6 MiB each)
- Wi‑Fi AMPDU disabled (avoids silent STA on some APs)

---

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| RX bytes = 0 / no RSTACK | TX↔RX crossed? Common GND? ICC on **3.3V**? NCP image flashed? |
| CRC / garbage | Baud **115200**; correct 115k2 NCP build |
| Portal dies, ping fails, USB still up | ESP32-C3 “silent STA” or mesh **client isolation**. Firmware recovers via gateway ARP silence (after quiet) and weak-RSSI roam (&lt; about −80 dBm). Overview shows **Last reset** (`panic` / `brownout` / `task_wdt` / …) when uptime restarts. Prefer pinning ≥ about −70 dBm. Disable AP client isolation, or join SoftAP `ICC1-Gateway-…` / `192.168.4.1` to reconfigure. |
| Panic reboot / Min heap &lt; ~15 KiB | DRAM exhaustion on the C3 (HomeKit + tables + stacks). **0.3.37** caps inventory at 16 devices, drops the duplicate status device snapshot, and trims portal/sniffer buffers + key task stacks. Overview shows **Free / Min / Largest** heap. |
| HomeKit “No Response” | Wait for deferred start; confirm `:8118`; try lock→unlock once after large inventory changes |
| Remote does nothing in Home | Mode = HomeKit buttons? Exposed? For control mode, complete Touchlink to a **bulb** |
| Sensor stuck / empty readings | **Read values**, then press the sensor button within a few seconds (sleepy devices only receive while polling). If identity keeps vanishing after reboot, NVS may be full — flash this build (reclaims legacy blobs) |
| OTA check fails | Browser can reach GitHub? Portal on home Wi‑Fi? Correct Stable/Nightly channel published? |

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

## License note

Protocol implementations are original host-side code based on public documentation. Third-party component licenses (ESP-IDF, HomeKit ADK derivatives in-tree, etc.) apply as marked in those trees.
