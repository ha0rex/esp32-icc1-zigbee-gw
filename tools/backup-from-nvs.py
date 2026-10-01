#!/usr/bin/env python3
"""Extract ICC Gateway device/group backup from an ESP NVS dump.

The portal /api/status only lists devices after the ICC connects (NVS load is
deferred). This tool reads the zb_net/devs5 blob directly from flash so you can
migrate C3 → S3 without reattaching the ICC to the C3.

Usage:
  ./tools/backup-from-nvs.py --port /dev/cu.usbmodem* -o icc-backup.json
  ./tools/backup-from-nvs.py --nvs /tmp/c3-nvs.bin -o icc-backup.json
"""

from __future__ import annotations

import argparse
import ctypes
import datetime as dt
import json
import os
import subprocess
import sys
from pathlib import Path

ZB_REMOTE_MAX_BUTTONS = 5
ZB_REMOTE_MAX_TARGETS = 8
GROUP_MAX = 8
GROUP_MAX_MEMBERS = 8
KIND = {
    0: "unknown",
    1: "sensor",
    2: "switch",
    3: "light",
    4: "remote",
    5: "outlet",
    6: "irrigation",
    7: "contact",
    8: "motion",
    9: "leak",
    10: "smoke",
}


class ZbDevice(ctypes.LittleEndianStructure):
    _fields_ = [
        ("used", ctypes.c_bool),
        ("eui64", ctypes.c_uint8 * 8),
        ("node_id", ctypes.c_uint16),
        ("node_type", ctypes.c_uint8),
        ("name", ctypes.c_char * 32),
        ("manufacturer", ctypes.c_char * 32),
        ("model", ctypes.c_char * 32),
        ("firmware", ctypes.c_char * 32),
        ("label", ctypes.c_char * 40),
        ("has_temp", ctypes.c_bool),
        ("temperature_c", ctypes.c_float),
        ("has_humidity", ctypes.c_bool),
        ("humidity_pct", ctypes.c_float),
        ("has_battery", ctypes.c_bool),
        ("battery_pct", ctypes.c_uint8),
        ("last_rssi", ctypes.c_int8),
        ("last_lqi", ctypes.c_uint8),
        ("homekit_expose", ctypes.c_bool),
        ("last_seen_ms", ctypes.c_int64),
        ("last_interview_ms", ctypes.c_int64),
        ("has_onoff", ctypes.c_bool),
        ("onoff_on", ctypes.c_bool),
        ("onoff_ep", ctypes.c_uint8),
        ("has_level", ctypes.c_bool),
        ("level", ctypes.c_uint8),
        ("level_ep", ctypes.c_uint8),
        ("remote_bound", ctypes.c_bool),
        ("remote_ep", ctypes.c_uint8),
        ("remote_no_zdo_bind", ctypes.c_bool),
        ("sensor_reporting", ctypes.c_bool),
        ("sensor_ep", ctypes.c_uint8),
        ("btn_mode", ctypes.c_uint8 * ZB_REMOTE_MAX_BUTTONS),
        ("btn_on", ctypes.c_bool * ZB_REMOTE_MAX_BUTTONS),
        ("remote_type", ctypes.c_uint8),
        ("target_eui64", ctypes.c_uint8 * 8),
        ("remote_group_id", ctypes.c_uint16),
        ("target_count", ctypes.c_uint8),
        ("targets", (ctypes.c_uint8 * 8) * ZB_REMOTE_MAX_TARGETS),
        ("target_group_count", ctypes.c_uint8),
        ("target_groups", ctypes.c_uint8 * ZB_REMOTE_MAX_TARGETS),
        ("sensor_cfg_step", ctypes.c_uint8),
        ("btn_name", (ctypes.c_char * 24) * ZB_REMOTE_MAX_BUTTONS),
        ("has_ias_zone", ctypes.c_bool),
        ("ias_zone_type", ctypes.c_uint16),
        ("ias_zone_status", ctypes.c_uint16),
        ("ias_zone_ep", ctypes.c_uint8),
        ("has_occupancy", ctypes.c_bool),
        ("occupancy", ctypes.c_bool),
        ("occupancy_ep", ctypes.c_uint8),
        ("binary_on", ctypes.c_bool),
        ("hk_sticky_kind", ctypes.c_uint8),
    ]


assert ctypes.sizeof(ZbDevice) == 464


class GroupPersist(ctypes.LittleEndianStructure):
    _fields_ = [
        ("used", ctypes.c_bool),
        ("id", ctypes.c_uint8),
        ("name", ctypes.c_char * 32),
        ("type", ctypes.c_uint8),
        ("homekit_expose", ctypes.c_bool),
        ("sensor_eui", ctypes.c_uint8 * 8),
        ("humidity_sensor_eui", ctypes.c_uint8 * 8),
        ("switch_eui", ctypes.c_uint8 * 8),
        ("cooler_eui", ctypes.c_uint8 * 8),
        ("target_c", ctypes.c_float),
        ("mode", ctypes.c_uint8),
        ("heating", ctypes.c_bool),
        ("cooling", ctypes.c_bool),
        ("member_count", ctypes.c_uint8),
        ("members", (ctypes.c_uint8 * 8) * GROUP_MAX_MEMBERS),
        ("power_fail_mode", ctypes.c_uint8),
        ("last_on", ctypes.c_bool),
        ("last_brightness_pct", ctypes.c_uint8),
        ("thermo_kind", ctypes.c_uint8),
        ("target_humidity_pct", ctypes.c_float),
        ("gap_c", ctypes.c_float),
        ("hysteresis_c", ctypes.c_float),
        ("humidity_force_heat", ctypes.c_bool),
    ]


def _idf_nvs_tool_dir() -> Path:
    idf = os.environ.get("IDF_PATH")
    if not idf:
        raise SystemExit("IDF_PATH is not set — run `. $HOME/esp/esp-idf/export.sh` first")
    d = Path(idf) / "components/nvs_flash/nvs_partition_tool"
    if not d.is_dir():
        raise SystemExit(f"NVS tools not found at {d}")
    return d


def dump_nvs_from_port(port: str, out_bin: Path) -> None:
    cmd = [
        "esptool",
        "-p",
        port,
        "-b",
        "460800",
        "read-flash",
        "0x9000",
        "0x6000",
        str(out_bin),
    ]
    print(f"Reading NVS from {port} …", file=sys.stderr)
    subprocess.check_call(cmd)


def assemble_blobs(nvs_bin: Path) -> tuple[dict[tuple[str, str], bytes], dict[tuple[str, str], object]]:
    """Return written blobs and scalar values from an NVS partition dump."""
    sys.path.insert(0, str(_idf_nvs_tool_dir()))
    from nvs_parser import NVS_Entry, NVS_Partition  # type: ignore

    raw = bytearray(nvs_bin.read_bytes())
    part = NVS_Partition(nvs_bin.name, raw)
    ns_map: dict[int, str] = {}
    blob_index: dict[str, object] = {}
    empty = NVS_Entry(-1, bytearray(32), "Erased")
    scalars: dict[tuple[str, str], object] = {}

    for page in part.pages:
        for entry in page.entries:
            if entry.state != "Written":
                continue
            if entry.metadata["namespace"] == 0:
                ns_map[entry.data["value"]] = entry.key
                continue
            ns_name = ns_map.get(entry.metadata["namespace"], str(entry.metadata["namespace"]))
            if entry.metadata["type"] == "blob_index":
                blob_index[f'{entry.metadata["namespace"]:03d}{entry.key}'] = {
                    "index": entry,
                    "ns": ns_name,
                    "chunks": [empty] * entry.data["chunk_count"],
                }
            elif entry.metadata["type"] in ("uint16_t", "uint8_t", "uint32_t", "int32_t", "int16_t", "int8_t"):
                scalars[(ns_name, entry.key)] = entry.data["value"]

    out: dict[tuple[str, str], bytes] = {}
    for meta in blob_index.values():
        index_entry = meta["index"]  # type: ignore[index]
        chunks = meta["chunks"]  # type: ignore[index]
        for page in part.pages:
            for entry in page.entries:
                if (
                    entry.state == "Written"
                    and entry.metadata["type"] != "blob_index"
                    and entry.metadata["namespace"] == index_entry.metadata["namespace"]
                    and entry.key == index_entry.key
                ):
                    slot_i = entry.metadata["chunk_index"] - index_entry.data["chunk_start"]
                    if 0 <= slot_i < len(chunks):
                        chunks[slot_i] = entry
        buf = bytearray()
        for kid in chunks:
            if kid is empty:
                continue
            for child in kid.children:
                buf += bytes(child.raw)
        buf = buf[: index_entry.data["size"]]
        ns_id = index_entry.metadata["namespace"]
        ns_name = ns_map.get(ns_id, str(ns_id))
        out[(ns_name, index_entry.key)] = bytes(buf)

    # Re-key scalars with resolved namespace names (collected after full scan).
    resolved: dict[tuple[str, str], object] = {}
    for (ns_name, key), val in scalars.items():
        # ns_name may already be resolved if namespace entry was seen first
        resolved[(ns_name if not str(ns_name).isdigit() else ns_map.get(int(ns_name), ns_name), key)] = val
    # Actually re-scan scalars properly:
    resolved = {}
    for page in part.pages:
        for entry in page.entries:
            if entry.state != "Written" or entry.metadata["namespace"] == 0:
                continue
            if entry.metadata["type"] in ("uint16_t", "uint8_t", "uint32_t", "int32_t", "int16_t", "int8_t"):
                ns_name = ns_map.get(entry.metadata["namespace"], str(entry.metadata["namespace"]))
                resolved[(ns_name, entry.key)] = entry.data["value"]
    return out, resolved


def fmt_eui(b) -> str:
    return ":".join(f"{x:02x}" for x in reversed(bytes(b)))


def cstr(b) -> str:
    if isinstance(b, bytes):
        return b.split(b"\0", 1)[0].decode("utf-8", "replace")
    return bytes(b).split(b"\0", 1)[0].decode("utf-8", "replace")


def eui_nonzero(b) -> bool:
    return any(bytes(b))


def parse_devices(blob: bytes, count: int | None) -> list[dict]:
    rec = ctypes.sizeof(ZbDevice)
    if count is None:
        if len(blob) % rec != 0:
            raise SystemExit(f"devs5 size {len(blob)} not multiple of {rec}")
        count = len(blob) // rec
    elif count * rec > len(blob):
        raise SystemExit(f"dev_n={count} needs {count*rec}B but blob is {len(blob)}B")
    devices = []
    for i in range(count):
        d = ZbDevice.from_buffer_copy(blob[i * rec : (i + 1) * rec])
        if not d.used:
            continue
        btn_names = []
        for bi in range(ZB_REMOTE_MAX_BUTTONS):
            btn_names.append(cstr(bytes(d.btn_name[bi])))
        targets = []
        for ti in range(min(int(d.target_count), ZB_REMOTE_MAX_TARGETS)):
            if eui_nonzero(d.targets[ti]):
                targets.append(fmt_eui(d.targets[ti]))
        tgroups = [int(d.target_groups[gi]) for gi in range(min(int(d.target_group_count), ZB_REMOTE_MAX_TARGETS))]
        sticky = int(d.hk_sticky_kind)
        devices.append(
            {
                "eui64": fmt_eui(d.eui64),
                "node_id": int(d.node_id),
                "name": cstr(d.name),
                "manufacturer": cstr(d.manufacturer),
                "model": cstr(d.model),
                "firmware": cstr(d.firmware),
                "kind": KIND.get(sticky, "unknown") if sticky else (
                    "sensor" if d.has_temp or d.has_humidity else
                    "remote" if d.remote_bound else
                    "light" if d.has_level else
                    "switch" if d.has_onoff else "unknown"
                ),
                "homekit_expose": bool(d.homekit_expose),
                "remote_type": int(d.remote_type),
                "button_modes": [int(d.btn_mode[bi]) for bi in range(ZB_REMOTE_MAX_BUTTONS)],
                "button_names": btn_names,
                "targets": targets,
                "target_groups": tgroups,
                "hk_sticky_kind": sticky,
            }
        )
    return devices


def parse_groups(blob: bytes) -> list[dict]:
    rec = ctypes.sizeof(GroupPersist)
    if len(blob) % rec != 0:
        # packed array is always GROUP_MAX rows
        if len(blob) < rec:
            return []
    n = min(GROUP_MAX, len(blob) // rec)
    groups = []
    mode_temp = {0: "off", 1: "heat", 4: "cool", 5: "auto"}
    mode_hum = {0: "off", 2: "humidify", 3: "dehumidify"}
    pf = {0: "previous", 1: "on", 2: "off"}
    for i in range(n):
        g = GroupPersist.from_buffer_copy(blob[i * rec : (i + 1) * rec])
        if not g.used:
            continue
        o: dict = {
            "name": cstr(g.name),
            "type": "thermostat" if g.type == 1 else "general",
            "homekit_expose": bool(g.homekit_expose),
        }
        if g.type == 1:
            o["regulation"] = "humidity" if g.thermo_kind == 1 else "temperature"
            o["sensor_eui"] = fmt_eui(g.sensor_eui) if eui_nonzero(g.sensor_eui) else ""
            o["humidity_sensor_eui"] = (
                fmt_eui(g.humidity_sensor_eui) if eui_nonzero(g.humidity_sensor_eui) else ""
            )
            o["switch_eui"] = fmt_eui(g.switch_eui) if eui_nonzero(g.switch_eui) else ""
            o["cooler_eui"] = fmt_eui(g.cooler_eui) if eui_nonzero(g.cooler_eui) else ""
            o["target_c"] = float(g.target_c)
            o["target_humidity"] = float(g.target_humidity_pct)
            o["gap_c"] = float(g.gap_c)
            o["hysteresis_c"] = float(g.hysteresis_c)
            o["humidity_force_heat"] = bool(g.humidity_force_heat)
            if g.thermo_kind == 1:
                o["mode"] = mode_hum.get(int(g.mode), "off")
            else:
                o["mode"] = mode_temp.get(int(g.mode), "heat")
        else:
            members = []
            for mi in range(min(int(g.member_count), GROUP_MAX_MEMBERS)):
                if eui_nonzero(g.members[mi]):
                    members.append(fmt_eui(g.members[mi]))
            o["members"] = members
            o["power_fail"] = pf.get(int(g.power_fail_mode), "previous")
        groups.append(o)
    return groups


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="Serial port of the ESP (reads NVS at 0x9000)")
    ap.add_argument("--nvs", type=Path, help="Existing NVS partition dump (.bin)")
    ap.add_argument("-o", "--output", type=Path, default=Path("icc-backup.json"))
    ap.add_argument("--keep-nvs", type=Path, help="Also save the raw NVS dump here")
    args = ap.parse_args()

    if not args.port and not args.nvs:
        ap.error("Provide --port or --nvs")

    nvs_path = args.nvs
    tmp = None
    if args.port:
        tmp = Path(args.keep_nvs) if args.keep_nvs else Path("/tmp/icc-nvs-dump.bin")
        dump_nvs_from_port(args.port, tmp)
        nvs_path = tmp
    assert nvs_path is not None

    blobs, scalars = assemble_blobs(nvs_path)

    dev_blob = blobs.get(("zb_net", "devs5"))
    if not dev_blob:
        raise SystemExit("No zb_net:devs5 blob in NVS — nothing to migrate")
    dev_n = scalars.get(("zb_net", "dev_n"))
    devices = parse_devices(dev_blob, int(dev_n) if dev_n is not None else None)

    groups: list[dict] = []
    grp_blob = blobs.get(("thermo", "grp4"))
    if grp_blob:
        groups = parse_groups(grp_blob)

    payload = {
        "format": "icc1-gateway-backup",
        "version": 1,
        "exported_at": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source": "nvs",
        "fw_version": "",
        "board": {"chip": "esp32c3", "name": "ESP32-C3", "note": "from NVS dump"},
        "devices": devices,
        "groups": groups,
    }
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.output}: {len(devices)} devices, {len(groups)} groups")
    for d in devices:
        print(f"  - {d['name'] or '(unnamed)'}  {d['eui64']}  hk={d['homekit_expose']}  {d['kind']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
