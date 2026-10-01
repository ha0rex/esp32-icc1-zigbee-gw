# Flashing EmberZNet NCP firmware on the IKEA ICC-1 / ICC-A-1

This gateway’s Zigbee radio is the **IKEA ICC-1 / ICC-A-1** module (Silicon Labs **EFR32MG1P132F256**), not the ESP32. The ESP32 speaks **EZSP over ASH over UART** to that NCP.

Stock IKEA firmware is a Zigbee **end device / router**, not a coordinator NCP. You must flash a compatible **EmberZNet NCP** image before this project can talk to the module.

For wiring the flashed module to the ESP32, see the main [README](../README.md#hardware).

---

## What you need

| Item | Notes |
| --- | --- |
| **ICC-1 or ICC-A-1** | Salvaged from TRÅDFRI gear, or a bare module |
| **SWD probe** | SEGGER J-Link, CMSIS-DAP, or similar (SWD, not USB-UART alone) |
| **3.3 V supply** | Common GND with the probe. **Never 5 V** |
| **Simplicity Commander** | [Silicon Labs download](https://www.silabs.com/developers/simplicity-studio) (Commander is included / available separately) |
| **NCP firmware** | `.s37` / `.gbl` built for **PB14/PB15** UART @ **115200** (see below) |

Community module docs and pin photos:

- [MattWestb/IKEA-TRADFRI-ICC-A-1-Module](https://github.com/MattWestb/IKEA-TRADFRI-ICC-A-1-Module) (“Billy EZSP”)
- [basilfx/TRADFRI-Hacking — MODULES.md](https://github.com/basilfx/TRADFRI-Hacking/blob/master/MODULES.md)
- Example NCP builds: [MattWestb/EFR32-FW — Billy_EZSP](https://github.com/MattWestb/EFR32-FW/tree/main/Billy_EZSP)

---

## Target firmware for this gateway

Use an NCP image that matches **all** of:

| Requirement | Value |
| --- | --- |
| Stack | EmberZNet **6.7.8.x** (EZSP protocol **v8**) |
| UART pins | **PB15 = NCP RX**, **PB14 = NCP TX** (no RTS/CTS) |
| Baud | **115200 8N1** |
| Typical filename | `NCP_USW_115k2_F256_678_PB14-PB15-PA0.s37` (or `.gbl` for serial bootloader update) |

`USW` = UART software flow / no HW flow control. Images that put UART on **PC10/PC11** will **not** match this project’s ESP wiring (pads 2/3).

`PA0` in the name is often the force-bootloader pin; you do not need it wired for normal gateway use.

---

## Module pads (ICC-1 / ICC-A-1)

Pad numbering matches the silk/layout used with this gateway:

| Pad | EFR32 | Role |
| --- | --- | --- |
| **2** | **PB15** | NCP **RX** (ESP **TX**) |
| **3** | **PB14** | NCP **TX** (ESP **RX**) |
| **6** | **PF0** | **SWCLK** |
| **7** | **PF1** | **SWDIO** |
| **8** | **PF2** | SWO (optional) |
| **10** | **RESETn** | Reset (optional but useful) |
| **11** | **VDD** | **3.3 V** |
| **12** | **GND** | GND |
| **16** | **PA0** | Force bootloader (optional) |

For flashing you only need **VDD, GND, SWCLK, SWDIO** (and ideally **RESETn**).

---

## Flash with Simplicity Commander (SWD)

1. Wire the probe:

   | Probe | ICC pad |
   | --- | --- |
   | VTref / 3V3 | **11** VDD |
   | GND | **12** GND |
   | SWDCLK | **6** PF0 |
   | SWDIO | **7** PF1 |
   | nRESET (optional) | **10** RESETn |

2. Power the module from a stable **3.3 V** source (probe VTref or a bench supply). Module must **not** be on mains / bulb driver while flashing.

3. Detect the chip (device id may vary slightly by package marking):

   ```bash
   commander device info
   ```

   Expect an **EFR32MG1** / **EFR32MG1PxxxF256**.

4. (Optional but recommended) Back up flash before overwriting IKEA firmware:

   ```bash
   commander readmem --region @mainflash -o icc-backup-mainflash.s37
   commander readmem --region @userdata -o icc-backup-userdata.s37
   ```

5. Flash the NCP image (path to your `.s37`):

   ```bash
   commander flash NCP_USW_115k2_F256_678_PB14-PB15-PA0.s37
   ```

   Some community packages ship a **combined bootloader + app**. Flash in the order their README specifies (often bootloader first, then NCP, or one combined `.s37`).

6. Power-cycle the module. Leave SWD connected only if you still need it; for gateway use, wire **pads 2/3/11/12** to the ESP as in the main README.

### Locked modules

Newer IKEA modules may refuse SWD until unlocked. See community notes (e.g. [t365.dk J-Link guide](https://t365.dk/flash-ikea-zigbee-j-link/) and MattWestb’s repo). Unlock/lock commands are device-specific — use the **EFR32MG1** part number Commander reports, not an MG21/MGM210 example blindly.

If SWD only works after a reset into the bootloader, hold **PA0 (pad 16)** low at reset, or pulse **RESETn**, then run Commander again.

---

## Alternate: GBL via serial bootloader

If a **standalone bootloader** is already on the module, you can upload a `.gbl` over UART (often XMODEM) instead of SWD. That path is image- and bootloader-specific; follow the instructions bundled with the community firmware set. This gateway does **not** implement a GBL uploader — use a PC + serial adapter on pads 2/3 (crossed TX/RX) at the bootloader baud rate.

---

## Verify with this project

1. Wire ESP ↔ ICC (any free UART GPIOs; tested defaults: S3 GPIO2/4, C3 GPIO6/7) + 3.3 V + GND.
2. Flash and run the ESP firmware; open the serial log or portal **Overview**.
3. Success looks like:

   ```
   ASH: connected
   EZSP: protocol version 8
   EmberZNet: 6.7.8.x
   ICC status: CONNECTED
   ```

| Symptom | Likely cause |
| --- | --- |
| RX bytes = 0 / no RSTACK | TX↔RX not crossed; wrong pads; NCP not running; 5 V damage |
| CRC / garbage | Wrong baud or wrong UART pins in the NCP build (not PB14/PB15) |
| EZSP version mismatch | Image not 6.7.8 / not EZSP v8 |

---

## License / firmware sources

NCP binaries are **not** redistributed in this repository. Obtain builds from Silicon Labs SDK builds you compile yourself, or from community archives linked above, and respect their licenses and local radio regulations.
