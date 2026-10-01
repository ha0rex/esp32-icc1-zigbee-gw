#!/usr/bin/env bash
# Pull a migration backup from a running gateway.
#
# HTTP mode (needs ICC connected for devices — groups work either way):
#   ./tools/backup-from-gateway.sh <gateway-host-or-ip> [outfile.json]
#
# USB/NVS mode (C3 without ICC — reads flash directly):
#   ./tools/backup-from-gateway.sh --usb [/dev/cu.usbmodem*] [outfile.json]
set -euo pipefail

if [[ "${1:-}" == "--usb" || "${1:-}" == "-u" ]]; then
  shift
  PORT="${1:-}"
  if [[ -z "$PORT" || "$PORT" == *.json ]]; then
    OUT="${PORT:-icc-backup.json}"
    PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true)
  else
    shift || true
    OUT="${1:-icc-backup.json}"
  fi
  if [[ -z "$PORT" ]]; then
    echo "No /dev/cu.usbmodem* found — plug the ESP in via USB." >&2
    exit 1
  fi
  ROOT="$(cd "$(dirname "$0")/.." && pwd)"
  # shellcheck disable=SC1090
  if [[ -z "${IDF_PATH:-}" && -f "$HOME/esp/esp-idf/export.sh" ]]; then
    # Quiet export for esptool + nvs_parser
    set +u
    # shellcheck source=/dev/null
    . "$HOME/esp/esp-idf/export.sh" >/dev/null
    set -u
  fi
  exec python3 "$ROOT/tools/backup-from-nvs.py" --port "$PORT" -o "$OUT"
fi

HOST="${1:-}"
OUT="${2:-icc-backup.json}"
if [[ -z "$HOST" ]]; then
  echo "Usage:" >&2
  echo "  $0 <gateway-host-or-ip> [outfile.json]" >&2
  echo "  $0 --usb [/dev/cu.usbmodem*] [outfile.json]   # NVS dump, no ICC needed" >&2
  exit 1
fi
URL="http://${HOST%/}/api/status"
TMP="$(mktemp)"
curl -fsS "$URL" -o "$TMP"
python3 - "$TMP" "$OUT" <<'PY'
import json, sys, datetime
src, out = sys.argv[1], sys.argv[2]
st = json.load(open(src, encoding="utf-8"))
devices = []
for d in st.get("devices") or []:
    devices.append({
        "eui64": d.get("eui64"),
        "name": d.get("name") or "",
        "homekit_expose": bool(d.get("homekit_expose")),
        "kind": d.get("kind") or "",
        "remote_type": int(d.get("remote_type") or 0),
        "button_modes": d.get("button_modes") or [],
        "button_names": d.get("button_names") or [],
        "targets": d.get("targets") or [],
        "target_groups": d.get("target_groups") or [],
    })
groups = []
for g in st.get("groups") or []:
    o = {
        "name": g.get("name") or "",
        "type": g.get("type") or "general",
        "homekit_expose": bool(g.get("homekit_expose")),
    }
    if g.get("type") == "thermostat":
        o.update({
            "regulation": g.get("regulation") or "temperature",
            "sensor_eui": g.get("sensor_eui") or "",
            "humidity_sensor_eui": g.get("humidity_sensor_eui") or "",
            "switch_eui": g.get("switch_eui") or "",
            "cooler_eui": g.get("cooler_eui") or "",
            "target_c": g.get("target_c"),
            "target_humidity": g.get("target_humidity"),
            "gap_c": g.get("gap_c"),
            "hysteresis_c": g.get("hysteresis_c"),
            "humidity_force_heat": bool(g.get("humidity_force_heat")),
            "mode": g.get("mode") or "heat",
        })
    else:
        o["members"] = g.get("members") or []
        o["power_fail"] = g.get("power_fail") or "previous"
    groups.append(o)
payload = {
    "format": "icc1-gateway-backup",
    "version": 1,
    "exported_at": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "fw_version": st.get("fw_version") or "",
    "board": st.get("board"),
    "devices": devices,
    "groups": groups,
}
with open(out, "w", encoding="utf-8") as f:
    json.dump(payload, f, indent=2)
    f.write("\n")
print(f"Wrote {out}: {len(devices)} devices, {len(groups)} groups")
if not devices:
    print("Note: 0 devices over HTTP usually means the ICC is offline — device NVS is only", file=sys.stderr)
    print("loaded after ASH connects. Use: ./tools/backup-from-gateway.sh --usb", file=sys.stderr)
PY
rm -f "$TMP"
