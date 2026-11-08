"""Fetch Home Assistant history for the UFH controller and the room thermostats into a CSV per entity.

Reads the HA token from ~/.config/ufh/ha_token (never printed). Usage:
  python3 fetch.py OUT_DIR [HOURS]
"""
import json, os, sys, urllib.request, urllib.parse, datetime as dt

HA = os.environ.get("HA_URL", "http://homeassistant.local:8123")
TOKEN = open(os.path.expanduser("~/.config/ufh/ha_token")).read().strip()
P = "heating_ufh_plc_"
ENTITIES = {
    # rooms: setpoint and heating action from the climate entity; temperature from its sensor (finer
    # resolution on the re-flashed Nests)
    "upstairs": "climate.upstairs_upstairs",
    "kitchen": "climate.nest_02aa01ac2613049n",
    "booth": "climate.booth_booth",
    "living": "climate.nest_02aa01ac38130bgw",
    "upstairs_t": "sensor.upstairs_upstairs_temperature",
    "kitchen_t": "sensor.nest_02aa01ac2613049n_temperature",
    "booth_t": "sensor.booth_booth_temperature",
    "living_t": "sensor.nest_02aa01ac38130bgw_temperature",
    "outdoor": "weather.forecast_home",
    # controller
    "boiler": f"sensor.{P}boiler_flow_temperature",
    "mixed": f"sensor.{P}mixed_flow_temperature",
    "ret": f"sensor.{P}return_temperature",
    "hot_share": f"sensor.{P}mixer_hot_share",
    "status": f"sensor.{P}heating_status",
    **{f"di{i}": f"binary_sensor.{P}input_{i}" for i in range(1, 5)},
    **{f"r{i}": f"switch.{P}relay_{i}" for i in (1, 2, 4, 5, 6, 7, 8)},
}


def get(path):
    req = urllib.request.Request(HA + path, headers={"Authorization": f"Bearer {TOKEN}"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.load(r)


def main():
    out, hours = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 48
    os.makedirs(out, exist_ok=True)
    start = (dt.datetime.now(dt.timezone.utc) - dt.timedelta(hours=hours)).isoformat()
    for name, ent in ENTITIES.items():
        # (without end_time HA returns only the first day after start)
        q = urllib.parse.urlencode({"filter_entity_id": ent, "end_time": dt.datetime.now(dt.timezone.utc).isoformat()})
        data = get(f"/api/history/period/{urllib.parse.quote(start)}?{q}")
        rows = data[0] if data else []
        with open(os.path.join(out, f"{name}.csv"), "w") as f:
            if ent.startswith("weather."):
                f.write("time,value\n")
                for s in rows:
                    f.write(f"{s['last_changed']},{s.get('attributes', {}).get('temperature', '')}\n")
            elif ent.startswith("climate."):
                f.write("time,temp,setpoint,action\n")
                for s in rows:
                    a = s.get("attributes", {})
                    f.write(f"{s['last_changed']},{a.get('current_temperature','')},{a.get('temperature','')},{a.get('hvac_action','')}\n")
            else:
                f.write("time,value\n")
                for s in rows:
                    f.write(f"{s['last_changed']},{s['state']}\n")
        print(f"{name:10s} {len(rows):6d} points" + (f"  from {rows[0]['last_changed'][:16]}" if rows else ""))


main()
