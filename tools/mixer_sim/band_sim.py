"""Simulate "band" control of the UFH mixer against fine regulation, over 8 hours of floor warm-up.

Band control (Jon's proposal, 7 Nov 2026): let the mixed flow swing over a ~10 °C band instead of
holding it to +/-1 °C.
  - HEAT: CFH on. The valve is set coarsely so the mix can't exceed the top of the band even with the
    boiler at its hottest; it is only re-set when that opening has drifted by more than a few %.
  - FREE-WHEEL: when the mix reaches the top of the band, CFH off; the valve still limits the mix to
    the top of the band, now for the boiler's actual (falling) temperature, so it opens gradually and
    the floor takes the boiler's leftover heat while the loop circulates and cools.
  - Back to HEAT when the mix falls to the bottom of the band.

Plant (from the 6-7 Nov tests): boiler with a ~2 min upstream delay and high thermal mass, the
measured valve curve, floor loops with a 3 min transport delay, and a slab that warms over hours.
Usage: python3 band_sim.py [OUT_PNG]
"""
import math
import sys

# Measured valve curve: (opening, share of boiler water); see mixer_control.h.
CURVE = [(0.00, 0.12), (0.10, 0.17), (0.15, 0.20), (0.21, 0.30), (0.27, 0.44), (0.32, 0.71), (0.35, 0.85),
         (0.38, 0.97), (0.42, 1.00), (1.00, 1.00)]


def share(o):
    o = min(max(o, 0.0), 1.0)
    for (o0, s0), (o1, s1) in zip(CURVE, CURVE[1:]):
        if o <= o1:
            return s0 + (o - o0) / (o1 - o0) * (s1 - s0)
    return 1.0


def opening(sh):
    if sh <= CURVE[0][1]:
        return 0.0
    for (o0, s0), (o1, s1) in zip(CURVE, CURVE[1:]):
        if sh <= s1:
            return o0 + (sh - s0) / (s1 - s0) * (o1 - o0) if s1 > s0 else o0
    return 0.42


class Plant:
    def __init__(self):
        self.tb = 20.0          # boiler flow, °C
        self.ts = 16.0          # floor slab, °C
        self.tr = 16.0          # return, °C
        self.probe = 20.0       # mixed-flow probe (lags the water)
        self.fire_delay = []    # CFH requests in transit through the upstream controller
        self.ret_delay = [16.0] * 180
        self.v = 0.0            # actual valve opening (servo moves it)
        self.mix = 20.0

    def step(self, dt, cfh, target_open):
        # Valve: moves at ~1/8 of the stroke per second (8 s full stroke); positioning resolution
        # ~3 % (smallest move), so it lands within +/-1.5 % of the target.
        if abs(target_open - self.v) > 0.015:
            self.v += math.copysign(min(abs(target_open - self.v), dt / 8), target_open - self.v)
        # Boiler: fires 2 min after CFH, heads for 70 °C (thermal mass, tau 5 min); when off it cools
        # slowly, faster when the mixer is drawing its water.
        self.fire_delay.append(cfh)
        firing = self.fire_delay.pop(0) if len(self.fire_delay) > 120 else False
        h = share(self.v)
        if firing:
            self.tb += (70 - self.tb) * dt / 300
        else:
            self.tb += (self.tr - self.tb) * dt * (0.2 + h) / 900 + (20 - self.tb) * dt / 7200
        self.mix = h * self.tb + (1 - h) * self.tr
        # Floor: the water gives up a share of its excess over the slab; the slab warms on that heat
        # and loses to the rooms (20 °C). Return comes back 3 min later.
        out = self.ts + (self.mix - self.ts) * 0.6
        self.ret_delay.append(out)
        self.tr = self.ret_delay.pop(0)
        heat = self.mix - out                     # ∝ heat into the floor
        # Slab: settles ~27 °C with 40 °C flow, time constant ~3.5 h; rooms at 20 °C take the heat.
        self.ts += (heat * 6.7e-5 - (self.ts - 20) * 5e-5) * dt
        self.probe += (self.mix - self.probe) * dt / 15
        return heat


def run(mode, low=33.0, high=43.0, setpoint=40.0, hours=8, boiler_max=70.0):
    p = Plant()
    dt = 1.0
    state = "HEAT"
    target = opening((high - 16) / (boiler_max - 16))
    cfh_cycles, moves, last_target, trips = 0, 0, target, 0
    integ = 0.0
    log = []
    heat_total = 0.0
    in_band = 0
    cfh = True
    for k in range(int(hours * 3600)):
        t = k * dt
        m = p.probe
        if mode == "band":
            if state == "HEAT" and m >= high:
                state, cfh = "FREE", False
                cfh_cycles += 1
            elif state == "FREE" and m <= low:
                state, cfh = "HEAT", True
            if state == "HEAT":
                # Coarse: the opening that keeps the mix at the top of the band with the boiler at
                # its hottest; only re-set when it has drifted by more than 3 % of travel.
                want = opening(min(1.0, max(0.0, (high - p.tr) / (boiler_max - p.tr))))
                if abs(want - target) > 0.03:
                    target = want
            else:
                # Free-wheel: limit the mix to the top of the band for the boiler's actual (falling)
                # temperature, so the leftover heat is used without overshooting.
                tb = max(p.tb, p.tr + 3)
                want = opening(min(1.0, max(0.0, (high - p.tr) / (tb - p.tr))))
                if abs(want - target) > 0.03:
                    target = want
        else:  # "fine": PI on the mix at the setpoint, share space, deadband 3 % of travel
            cfh = True
            if m > 45:
                trips += 1 if cfh else 0
            ff = min(1.0, max(0.0, (setpoint - p.tr) / (max(p.tb, setpoint + 5) - p.tr)))
            err = setpoint - m
            integ = min(max(integ + err * dt / 90, -12), 12)
            sh = min(1.0, max(0.0, ff + 0.025 * (err + integ)))
            want = opening(sh)
            if abs(want - target) > 0.03:
                target = want
        if abs(target - last_target) > 1e-6:
            moves += 1
            last_target = target
        if p.probe > 45:
            trips += 1
        heat_total += p.step(dt, cfh, target)
        if low - 0.5 <= p.mix <= high + 0.5:
            in_band += 1
        if k % 30 == 0:
            log.append((t / 3600, p.tb, p.mix, p.tr, p.ts, cfh, p.v))
    hrs = hours
    return {
        "log": log, "cfh_cycles_per_h": cfh_cycles / hrs, "moves_per_h": moves / hrs,
        "trips": trips, "heat": heat_total, "in_band": in_band / (hours * 3600),
        "mix_max": max(r[2] for r in log[60:]), "mix_min": min(r[2] for r in log[60:]),
    }


def main():
    fine = run("fine")
    band = run("band", low=35.0, high=42.0)
    for name, r in (("fine (40 °C, +/-1)", fine), ("band 35-42 °C", band)):
        print(f"{name:20s}  CFH cycles/h {r['cfh_cycles_per_h']:4.1f}  servo moves/h {r['moves_per_h']:5.1f}  "
              f"mix {r['mix_min']:.1f}-{r['mix_max']:.1f} °C  over 45 °C: {r['trips']} s  "
              f"heat into floor {r['heat']/1e3:.0f} (rel)")
    # Late phase: floor warm
    for name, r in (("fine", fine), ("band", band)):
        late = [x for x in r["log"] if x[0] > 6]
        print(f"  {name} hours 6-8: return {late[0][3]:.1f}->{late[-1][3]:.1f} °C, slab {late[-1][4]:.1f} °C")
    if len(sys.argv) > 1:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(10, 7), dpi=120, sharex=True)
        for ax, (name, r) in zip(axs, (("Fine control at 40 °C", fine), ("Band control 35-42 °C", band))):
            L = r["log"]
            t = [x[0] for x in L]
            ax.plot(t, [x[1] for x in L], color="#e8590c", lw=1, label="boiler flow")
            ax.plot(t, [x[2] for x in L], color="#2b8a3e", lw=1.4, label="mixed flow")
            ax.plot(t, [x[3] for x in L], color="#1c7ed6", lw=1, label="return")
            ax.plot(t, [x[4] for x in L], color="#868e96", lw=1, ls="--", label="floor slab")
            on = [x[5] for x in L]
            ax.fill_between(t, 0, 5, where=on, color="#ffa94d", alpha=0.6, step="post", label="CFH on")
            ax.axhline(45, color="#c92a2a", lw=0.8, ls=":")
            ax.set_ylim(0, 75)
            ax.set_ylabel("°C")
            ax.set_title(f"{name}: {r['cfh_cycles_per_h']:.1f} CFH cycles/h, {r['moves_per_h']:.0f} servo moves/h")
            ax.grid(alpha=0.3)
            ax.legend(loc="upper right", fontsize=8, ncol=5)
        axs[1].set_xlabel("hours")
        fig.tight_layout()
        fig.savefig(sys.argv[1])


main()
