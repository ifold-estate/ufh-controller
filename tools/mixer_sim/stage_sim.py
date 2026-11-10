"""Compare band control with a two-stage heating phase, on a plant built from the 8 Nov 2026 tests.

Plant (measured 8 Nov):
  - Our CFH opens a spring-return 2-port zone valve: open after 15 s, shut within 2 s of CFH off.
  - The valve's end switch starts the upstream controller's timer: after 3 min 50 s of continuous
    call it runs the boiler pump (and the burner, if the boiler's own thermostat wants heat). The
    timer resets whenever the call drops. Water then takes ~20 s to reach our probes.
  - 2-port valve open but boiler pump off: our floor pump pulls boiler water at a lower share
    ("pulled" curve: ~22 % at 40 % open, ~50 % at 61 %). Boiler pump on: "pumped" curve (single
    zone, 8 Nov: ~66 % at 21 %). Valve shut: no boiler water.
  - The boiler holds stored heat; pulling replaces its water with return water (pulse 2: water from
    the boiler fell 61 -> 50 °C in 3 min). Burner heats it while the boiler pump runs and its
    thermostat calls (on below 52 °C, off above 62 °C).
  - Our boiler-flow probe sits on our branch: follows the water when it flows, cools slowly when not.

Strategies:
  band   : current firmware. CFH on below 35 °C mix, off at 42 °C; valve set for boiler max 70 °C.
  staged : heating phase starts below 35 °C mix and ends when the return reaches RET_END.
           1. Scavenge: while the boiler holds useful heat, call in pulses of 3 min 15 s (under the
              timer) with 30 s gaps, valve set for the pulled curve to aim the mix at ~41 °C.
           2. Full call once pulses no longer bring warm water: pull during the wait, close sharply
              when the boiler pump starts (mix rising > 0.15 °C/s), then hold the mix under 42 °C
              with a coarse ceiling (move only outside 38-43 °C).
Usage: python3 stage_sim.py [OUT_PNG]
"""
import math
import sys

PUMPED = [(0.00, 0.10), (0.05, 0.15), (0.10, 0.24), (0.15, 0.42), (0.21, 0.66), (0.30, 0.85), (0.40, 0.95),
          (1.00, 1.00)]
# The firmware's curve (measured with all five zones open on 7 Nov): what band control assumes.
FIRMWARE = [(0.00, 0.12), (0.10, 0.17), (0.15, 0.20), (0.21, 0.30), (0.27, 0.44), (0.32, 0.71), (0.35, 0.85),
            (0.38, 0.97), (0.42, 1.00), (1.00, 1.00)]
PULLED = [(0.00, 0.00), (0.20, 0.05), (0.40, 0.22), (0.61, 0.50), (0.80, 0.60), (1.00, 0.65)]


def interp(curve, x):
    x = min(max(x, 0.0), 1.0)
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if x <= x1:
            return y0 + (x - x0) / (x1 - x0) * (y1 - y0)
    return curve[-1][1]


def inverse(curve, y):
    if y <= curve[0][1]:
        return curve[0][0]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if y <= y1:
            return x0 + (y - y0) / (y1 - y0) * (x1 - x0) if y1 > y0 else x0
    return 1.0


class Plant:
    def __init__(self, boiler_start=60.0):
        self.tb = boiler_start      # boiler water
        self.probe_b = 30.0         # our boiler-flow probe (on our branch)
        self.ts = 22.0              # slab
        self.tr = 24.0
        self.ret_delay = [24.0] * 180
        self.mix = 24.0
        self.probe_m = 24.0
        self.v = 0.1                # valve opening
        self.cfh_on_for = 0.0       # s our CFH has been on continuously
        self.pump_on = False        # boiler pump (upstream)
        self.burner = False
        self.stat_on = True
        self.flow_delay = []        # boiler-side flow regime in transit to our mixer (20 s)
        # counters
        self.pump_starts = 0
        self.burner_s = 0.0
        self.burner_starts = 0
        self.free_heat = 0.0
        self.heat = 0.0

    def step(self, dt, cfh, target):
        # Servo: 8 s full stroke; lands within +/-1.5 % of the target.
        if abs(target - self.v) > 0.015:
            self.v += math.copysign(min(abs(target - self.v), dt / 8), target - self.v)
        # 2-port valve and upstream timer.
        self.cfh_on_for = self.cfh_on_for + dt if cfh else 0.0
        valve_open = self.cfh_on_for >= 15
        pump_now = self.cfh_on_for >= 15 + 230
        if pump_now and not self.pump_on:
            self.pump_starts += 1
        self.pump_on = pump_now
        regime = 0 if not valve_open else (2 if self.pump_on else 1)
        self.flow_delay.append(regime)
        regime = self.flow_delay.pop(0) if len(self.flow_delay) > 20 else 0
        h = 0.0 if regime == 0 else interp(PULLED if regime == 1 else PUMPED, self.v)
        # Boiler: burner while the pump runs and its thermostat calls; pulling draws its water down.
        if self.tb > 62:
            self.stat_on = False
        elif self.tb < 52:
            self.stat_on = True
        burner = self.pump_on and self.stat_on
        if burner and not self.burner:
            self.burner_starts += 1
        self.burner = burner
        if burner:
            self.tb += 0.12 * dt          # ~7 °C/min net with our draw
            self.burner_s += dt
        self.tb += 0.0045 * h * (self.tr - self.tb) * dt + (18 - self.tb) * dt / 20000
        # Probe on our branch: follows the boiler water when it flows, cools when it doesn't.
        if regime:
            self.probe_b += (self.tb - self.probe_b) * dt / 20
        else:
            self.probe_b += (22 - self.probe_b) * dt / 1500
        self.mix = h * self.tb + (1 - h) * self.tr
        self.probe_m += (self.mix - self.probe_m) * dt / 15
        # Floor (as band_sim): slab warms over hours, rooms at 20 °C.
        out = self.ts + (self.mix - self.ts) * 0.6
        self.ret_delay.append(out)
        self.tr = self.ret_delay.pop(0)
        heat = self.mix - out
        self.ts += (heat * 6.7e-5 - (self.ts - 20) * 5e-5) * dt
        self.heat += heat
        if regime == 1:
            self.free_heat += heat * h  # the boiler-water share of what went in, while unpumped
        return regime


def run(strategy, hours=8, boiler_start=60.0, low=35.0, high=42.0, ret_end=33.0):
    p = Plant(boiler_start)
    dt = 1.0
    cfh = False
    target = 0.1
    moves, last_target, calls = 0, target, 0
    log = []
    state = "IDLE"
    t_state = 0.0
    prev_mix = p.probe_m
    over45 = 0
    last_adjust = 0.0
    for k in range(int(hours * 3600)):
        t = k * dt
        m = p.probe_m
        rate = (m - prev_mix) / dt
        prev_mix = m
        t_state += dt
        if strategy == "band":
            if not cfh and m <= low:
                cfh, calls = True, calls + 1
            elif cfh and m >= high:
                cfh = False
            hot = max(70.0, p.probe_b) if cfh else max(p.probe_b, p.tr + 3)
            want = inverse(FIRMWARE, min(1.0, max(0.0, (high - p.tr) / (hot - p.tr))))
            if want < target - 0.01 or abs(want - target) > 0.08 or (cfh and t_state < 2):
                target = want
        else:
            if state == "IDLE":
                cfh = False
                if m <= low:
                    # Stored heat worth pulling? (the probe reads low after a long idle, so judge by
                    # what the last pulse/call brought; start by scavenging and see)
                    state, t_state = "PULSE", 0.0
            elif state == "PULSE":
                cfh = t_state < 195
                # Aim the mix at ~41 °C with the pulled curve, for the boiler water now arriving.
                tbw = max(p.probe_b, p.tr + 3)
                want = inverse(PULLED, min(0.65, max(0.0, (41 - p.tr) / (tbw - p.tr))))
                if abs(want - target) > 0.03:  # the positioner's deadband
                    target = want
                if t_state >= 195 + 30:
                    if p.tr >= ret_end:
                        state = "IDLE"
                    elif p.probe_b < p.tr + 8:      # pulses no longer bring warm water: full call
                        state, t_state = "CALL", 0.0
                    else:
                        t_state = 0.0               # another pulse
                if t_state < 1:
                    calls += 1
            elif state == "CALL":
                cfh = True
                if t_state < 1:
                    calls += 1
                if t_state < 260 and rate < 0.15:
                    target = 0.61                   # pull hard while waiting for the boiler pump
                elif t_state < 300 and rate >= 0.15:
                    t_state = 300
                    last_adjust = 300                   # pump start seen: close to the pumped setting
                    target = inverse(PUMPED, min(1.0, max(0.0, (41 - p.tr) / (max(p.tb, 55) - p.tr))))
                else:
                    if t_state < 300:
                        t_state = 300
                        last_adjust = 300
                        target = inverse(PUMPED, min(1.0, max(0.0, (41 - p.tr) / (max(p.tb, 55) - p.tr))))
                    # Coarse ceiling, at most one step every 30 s.
                    if t_state - last_adjust >= 30:
                        if m > 43:
                            target, last_adjust = max(0.0, target - 0.03), t_state
                        elif m < 38:
                            target, last_adjust = min(0.5, target + 0.02), t_state
                if p.tr >= ret_end and t_state > 900:
                    state, t_state, cfh = "IDLE", 0.0, False
        if abs(target - last_target) > 1e-6:
            moves += 1
            last_target = target
        p.step(dt, cfh, target)
        if p.probe_m > 45:
            over45 += 1
        if k % 30 == 0:
            log.append((t / 3600, p.tb, p.probe_b, p.mix, p.tr, p.ts, cfh, p.pump_on, p.burner))
    return {
        "log": log, "heat": p.heat, "free": p.free_heat, "pump_starts": p.pump_starts / hours,
        "burner_h": p.burner_s / 3600, "burner_starts": p.burner_starts / hours, "calls": calls / hours,
        "moves": moves / hours, "over45": over45,
        "mix_min": min(r[3] for r in log[40:]), "mix_max": max(r[3] for r in log[40:]),
        "slab": log[-1][5],
    }


def main():
    print(" strategy  boiler start | heat  free%  CFH calls/h  boiler-pump starts/h  burner h (starts/h)  "
          "servo moves/h  mix range   >45 °C  slab")
    res = {}
    for start in (60.0, 20.0):
        for strat in ("band", "staged"):
            r = run(strat, boiler_start=start)
            res[(strat, start)] = r
            print(f" {strat:7s}  {start:4.0f} °C     | {r['heat']/1e3:4.0f}  {100*r['free']/max(r['heat'],1):4.0f}   "
                  f"{r['calls']:5.1f}        {r['pump_starts']:5.1f}               {r['burner_h']:4.1f} ({r['burner_starts']:4.1f})"
                  f"          {r['moves']:5.1f}        {r['mix_min']:4.1f}-{r['mix_max']:4.1f}  {r['over45']:4d} s  {r['slab']:.1f}")
    if len(sys.argv) > 1:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(11, 7.5), dpi=120, sharex=True)
        for ax, (name, r) in zip(axs, (("Current band control (35-42 °C on the mix)", res[("band", 60.0)]),
                                       ("Two-stage: free pulses, then a full call until the return is warm",
                                        res[("staged", 60.0)]))):
            L = r["log"]
            t = [x[0] for x in L]
            ax.plot(t, [x[1] for x in L], color="#e8590c", lw=1, label="boiler water")
            ax.plot(t, [x[3] for x in L], color="#2b8a3e", lw=1.3, label="mixed flow")
            ax.plot(t, [x[4] for x in L], color="#1c7ed6", lw=1, label="return")
            ax.plot(t, [x[5] for x in L], color="#868e96", lw=1, ls="--", label="floor slab")
            ax.fill_between(t, 0, 4, where=[x[6] for x in L], color="#ffa94d", alpha=0.6, step="post", label="our CFH")
            ax.fill_between(t, 4, 8, where=[x[7] for x in L], color="#74c0fc", alpha=0.7, step="post",
                            label="boiler pump")
            ax.fill_between(t, 8, 12, where=[x[8] for x in L], color="#fa5252", alpha=0.6, step="post",
                            label="burner")
            ax.axhline(45, color="#c92a2a", lw=0.8, ls=":")
            ax.set_ylim(0, 75)
            ax.set_ylabel("°C")
            ax.set_title(f"{name}: heat {r['heat']/1e3:.0f}, {r['pump_starts']:.1f} boiler-pump starts/h, "
                         f"{r['moves']:.0f} servo moves/h, {100*r['free']/max(r['heat'],1):.0f} % free")
            ax.grid(alpha=0.3)
            ax.legend(loc="upper right", fontsize=7.5, ncol=7)
        axs[-1].set_xlabel("hours (boiler starts at 60 °C)")
        fig.tight_layout()
        fig.savefig(sys.argv[1])


if __name__ == "__main__":
    main()
