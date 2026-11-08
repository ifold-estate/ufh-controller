"""Per-room response to underfloor heating: gain, lag and heat loss, from Home Assistant history.

Model, per room, on a 10-minute grid:
    dT/dt = gain * Q(t - lag) - (T - T_out) / tau
  Q    = heat delivered to the room's zones: (mixed - return) while the floor pump runs and the room's
         zone valve(s) are open, divided among all open valves (each relay serves two loops, so the
         flow splits about evenly). Units: °C of flow/return difference.
  gain = °C per hour the room warms per °C of (mixed - return) it receives.
  tau  = heat-loss time constant (hours): how fast the room cools towards outdoors with no heat.
  lag  = delay between heat going into the slab and the room responding (scanned 0-6 h).
Fitted by least squares for each lag; the lag with the best fit wins.

Usage: python3 analyze.py DATA_DIR [OUT_PNG]     (DATA_DIR from fetch.py)
       python3 analyze.py --selftest
"""
import csv
import datetime as dt
import math
import os
import sys

import numpy as np

STEP_H = 1 / 6  # 10-minute grid
# Rooms: thermostat, the controller input it drives, and its zone relays (8 Nov 2026 map; relay 6
# follows the kitchen since 23:40 that evening).
ROOMS = {
    "upstairs": {"di": "di1", "relays": ["r7"]},          # T4 playroom: zones 7, 8
    "kitchen": {"di": "di2", "relays": ["r8", "r6"]},     # N1: zones 2, 3 (+ 4, 5 via relay 6)
    "booth": {"di": "di3", "relays": ["r4"]},             # N2: zones 1, 6
    "living": {"di": "di4", "relays": ["r5"]},            # N3 lounge: zones 9, 10
}
ALL_RELAYS = ["r4", "r5", "r6", "r7", "r8"]


def load(path):
    """[(t_hours_since_epoch, value)] from a fetch.py CSV; non-numeric states map on/off to 1/0."""
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for row in csv.DictReader(f):
            v = row.get("value", row.get("temp"))
            if v in ("on", "off"):
                v = 1.0 if v == "on" else 0.0
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            t = dt.datetime.fromisoformat(row["time"]).timestamp() / 3600
            out.append((t, v))
    return out


def on_grid(series, grid, hold=True):
    """Sample a step series (value holds until the next change) onto the grid; NaN before the first."""
    vals = np.full(len(grid), np.nan)
    if not series:
        return vals
    ts = np.array([s[0] for s in series])
    vs = np.array([s[1] for s in series])
    idx = np.searchsorted(ts, grid, side="right") - 1
    ok = idx >= 0
    vals[ok] = vs[idx[ok]]
    return vals


def fit(T, Q, Tout, max_lag_h=6.0):
    """Least-squares fit of dT/dt = gain*Q(t-lag) - (T-Tout)/tau. Returns dict or None."""
    dT = np.diff(T) / STEP_H
    best = None
    for k in range(0, min(int(max_lag_h / STEP_H) + 1, len(T) - 1)):
        Ql = np.concatenate([np.full(k, np.nan), Q[:len(Q) - k]])[:-1]
        X = np.column_stack([Ql, -(T[:-1] - Tout[:-1])])
        ok = np.isfinite(X).all(axis=1) & np.isfinite(dT)
        if ok.sum() < 36:  # need 6 h of usable data
            continue
        coef, res, *_ = np.linalg.lstsq(X[ok], dT[ok], rcond=None)
        pred = X[ok] @ coef
        sse = float(((dT[ok] - pred) ** 2).sum())
        var = float(((dT[ok] - dT[ok].mean()) ** 2).sum()) or 1e-9
        r2 = 1 - sse / var
        if best is None or sse < best["sse"]:
            best = {"lag_h": k * STEP_H, "gain": coef[0], "tau_h": 1 / coef[1] if coef[1] > 0 else math.inf,
                    "r2": r2, "sse": sse, "n_h": ok.sum() * STEP_H}
    return best


def build(data_dir):
    s = {name[:-4]: load(os.path.join(data_dir, name)) for name in os.listdir(data_dir) if name.endswith(".csv")}
    starts = [v[0][0] for k, v in s.items() if v and k.endswith("_t")]
    if not starts:
        return None, s
    t0, t1 = min(starts), max(v[-1][0] for v in s.values() if v)
    grid = np.arange(t0, t1, STEP_H)
    g = {k: on_grid(v, grid) for k, v in s.items()}
    pump = np.nan_to_num(g.get("r2", np.zeros(len(grid))))
    dtemp = np.clip(np.nan_to_num(g["mixed"] - g["ret"]), 0, None)
    n_open = sum(np.nan_to_num(g.get(r, np.zeros(len(grid)))) for r in ALL_RELAYS)
    rooms = {}
    for room, cfg in ROOMS.items():
        if f"{room}_t" not in g:
            continue
        mine = sum(np.nan_to_num(g.get(r, np.zeros(len(grid)))) for r in cfg["relays"])
        Q = np.where(n_open > 0, pump * dtemp * mine / np.maximum(n_open, 1), 0.0)
        rooms[room] = {"T": g[f"{room}_t"], "Q": Q, "call": g.get(cfg["di"]), "set": on_grid(
            [(t, v) for t, v in load(os.path.join(data_dir, f"{room}.csv"))], grid)}
    return {"grid": grid, "Tout": g.get("outdoor", np.full(len(grid), np.nan)), "rooms": rooms}, s


def selftest():
    rng = np.random.default_rng(1)
    n = int(72 / STEP_H)
    Tout = 6 + 3 * np.sin(np.arange(n) * STEP_H * 2 * math.pi / 24)
    Q = np.zeros(n)
    for start in range(0, n, 48):  # 3 h of heat every 8 h
        Q[start:start + 18] = 8
    gain, tau, lag = 0.04, 30.0, 2.0
    k = int(lag / STEP_H)
    T = np.zeros(n)
    T[0] = 18
    for i in range(1, n):
        q = Q[i - 1 - k] if i - 1 - k >= 0 else 0
        T[i] = T[i - 1] + STEP_H * (gain * q - (T[i - 1] - Tout[i - 1]) / tau)
    T = np.round(T + rng.normal(0, 0.03, n), 1)  # thermostat resolution 0.1 °C
    r = fit(T, Q, Tout)
    print(f"self-test: true gain {gain} tau {tau} h lag {lag} h -> fitted gain {r['gain']:.3f} "
          f"tau {r['tau_h']:.0f} h lag {r['lag_h']:.1f} h (R² {r['r2']:.2f})")
    assert abs(r["gain"] - gain) / gain < 0.25 and abs(r["lag_h"] - lag) <= 0.5 and 15 < r["tau_h"] < 60


def main():
    if sys.argv[1] == "--selftest":
        selftest()
        return
    d, raw = build(sys.argv[1])
    if d is None:
        print("no room temperature data yet")
        return
    span = (d["grid"][-1] - d["grid"][0])
    print(f"room data: {span:.1f} h, outdoor {np.nanmin(d['Tout']):.1f}-{np.nanmax(d['Tout']):.1f} °C")
    print(" room       temp now  heat in (°C·h)  fit: gain °C/h per °C   lag h   loss tau h   R²   data h")
    for room, r in d["rooms"].items():
        heat = float(np.nansum(r["Q"]) * STEP_H)
        res = fit(r["T"], r["Q"], d["Tout"])
        now = r["T"][np.isfinite(r["T"])][-1] if np.isfinite(r["T"]).any() else float("nan")
        if res:
            print(f" {room:9s} {now:6.1f}     {heat:8.1f}        {res['gain']:8.3f}            {res['lag_h']:4.1f}"
                  f"    {res['tau_h']:6.0f}    {res['r2']:4.2f}  {res['n_h']:5.0f}")
        else:
            print(f" {room:9s} {now:6.1f}     {heat:8.1f}        (not enough data to fit)")
    if len(sys.argv) > 2:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [dt.datetime.fromtimestamp(x * 3600) for x in d["grid"]]
        fig, axs = plt.subplots(len(d["rooms"]), 1, figsize=(11, 2.4 * len(d["rooms"])), dpi=110, sharex=True)
        for ax, (room, r) in zip(np.atleast_1d(axs), d["rooms"].items()):
            ax.plot(t, r["T"], color="#c92a2a", lw=1.5, label="room")
            ax.plot(t, r["set"], color="#868e96", lw=1, ls="--", label="setpoint")
            ax2 = ax.twinx()
            ax2.fill_between(t, 0, r["Q"], color="#ffa94d", alpha=0.5, step="post", label="heat in")
            ax2.set_ylim(0, 30)
            ax2.set_ylabel("ΔT in, °C")
            ax.set_ylabel("°C")
            ax.set_title(room)
            ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(sys.argv[2])


main()
