"""Compare the current staged mode with average-temperature control (PI on the mix, continuous call).
Plant: tools/mixer_sim/stage_sim.py (8 Nov 2026 measurements). Thermostats assumed to call throughout.
Servo: moves smaller than min_move are not made; each move lands within +/-1.5 % (random).
Needs stage_sim.py's main() call wrapped in `if __name__ == "__main__":` so it can be imported.
Usage: python3 avg_sim.py [OUT_PNG]"""
import os, random, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stage_sim import Plant, PULLED, PUMPED, interp, inverse

def sim(strategy, hours=8, boiler_start=60.0, sp=40.0, min_move=0.03, kp=0.01, ti=180.0, seed=1,
        pulses=True, peak_cut=46.0):
    random.seed(seed)
    p = Plant(boiler_start)
    dt = 1.0
    cfh, target, cmd = False, 0.1, 0.1
    moves = 0
    state, t_state = ("PULSE" if pulses else "WAIT"), 0.0
    end_m = 0.0; end_sat = False
    prev = p.probe_m
    integ = 0.0
    pulse_sum = pulse_n = 0
    pumped = False
    flow_t = 0.0
    last_adj = 0.0
    mixes, over45, over43_run, max43 = [], 0, 0, 0
    trace = []
    calls = 0
    for k in range(int(hours * 3600)):
        t = k * dt
        m = p.probe_m
        rate = (m - prev) / dt
        prev = m
        t_state += dt
        if strategy == "staged":   # firmware v0.9 staged, band 35-42, return target 33
            low, high, ret_end = 35.0, 42.0, 33.0
            aim, ceil = high - 1, high + 1
            if state == "IDLE":
                cfh = False
                if m <= low: state, t_state = "PULSE", 0.0
            elif state == "PULSE":
                if t_state >= 225:
                    t_state = 0.0
                    if p.tr >= ret_end: state = "IDLE"
                    elif p.probe_b < p.tr + 8: state = "WAIT"
                cfh = state == "PULSE" and t_state < 195
                if state == "PULSE" and cfh:
                    want = inverse(PULLED, min(0.65, max(0.0, (aim - p.tr) / max(max(p.probe_b, 60) - p.tr, 3))))
                    if m > ceil: cmd = max(0.0, cmd - 0.05)
                    elif want < cmd or m < aim - 1: cmd = want
            elif state == "WAIT":
                cfh = True
                cmd = 0.61
                if rate >= 0.15 or t_state >= 300:
                    state, t_state, last_adj = "CALL", 0.0, 0.0
                    cmd = inverse(PUMPED, min(1.0, max(0.0, (aim - p.tr) / (max(p.probe_b, 55) - p.tr))))
            elif state == "CALL":
                cfh = True
                if m > ceil + 2: cmd = max(0.0, cmd - 0.03); last_adj = t_state
                elif t_state - last_adj >= 30:
                    if m > ceil: cmd, last_adj = max(0.0, cmd - 0.03), t_state
                    elif m < aim - 3: cmd, last_adj = min(0.5, cmd + 0.02), t_state
                if p.tr >= ret_end and t_state >= 900: state, t_state, cfh = "IDLE", 0.0, False
        else:   # average control
            regime_flow = cfh and p.cfh_on_for >= 35
            flow_t = flow_t + dt if regime_flow else 0.0
            if state == "PULSE":
                if t_state >= 225:
                    pulse_sum = pulse_n = 0
                    t_state = 0.0
                    # Stored heat can't hold the target any more: at the end of the pulse the valve was at
                    # its limit and the mix still short.
                    if end_sat and end_m < sp - 1.5: state = "WAIT"
                cfh = state == "PULSE" and t_state < 195
                if state == "PULSE" and cfh: pulse_sum += m; pulse_n += 1
            if state == "WAIT":
                cfh = True
                if rate >= 0.15 or t_state >= 300:
                    state, t_state = "CALL", 0.0
                    pumped = True
            if state == "CALL":
                cfh = True
            curve = PUMPED if pumped else PULLED
            cap = 1.0 if pumped else 0.65
            if state == "WAIT" and not pumped:
                cmd = 0.61
            elif cfh and k % 5 == 0:
                # Feed-forward from the boiler water actually arriving (assume >= 55 C only until it has flowed 45 s).
                tb = p.probe_b if flow_t >= 45 else max(p.probe_b, 55.0)
                ff = (sp - p.tr) / max(tb - p.tr, 3.0)
                err = sp - m
                share = ff + kp * err + integ / ti * kp
                sat_hi, sat_lo = share >= cap and err > 0, share <= 0 and err < 0
                if flow_t >= 45 and not sat_hi and not sat_lo:
                    integ = max(-30 * ti, min(30 * ti, integ + err * 5))
                share = min(cap, max(0.0, ff + kp * err + integ / ti * kp))
                cmd = inverse(curve, share)
                if m > peak_cut: cmd = max(0.0, target - 0.05)
                if state == "PULSE" and 150 <= t_state < 195:
                    end_m, end_sat = m, share >= cap - 1e-3
            if not cfh and not pumped:
                pass
        # Servo: skip moves under min_move; land within +/-1.5 %.
        if abs(cmd - target) >= min_move:
            target = min(1.0, max(0.0, cmd + random.uniform(-0.015, 0.015)))
            moves += 1
        p.step(dt, cfh, target)
        if t > 1800:   # after the first half hour
            mixes.append(p.probe_m)
            if p.probe_m > 45: over45 += 1
            over43_run = over43_run + 1 if p.probe_m > sp + 3 else 0
            max43 = max(max43, over43_run)
        if k % 20 == 0: trace.append((t / 3600, p.tb, p.mix, p.tr, cfh, p.pump_on, target))
    mean = sum(mixes) / len(mixes)
    return dict(mean=mean, coil=None, over45=over45, max_over=max43, moves=moves / hours, pump_starts=p.pump_starts / hours,
                heat=p.heat / 1e3, free=100 * p.free_heat / max(p.heat, 1), lo=min(mixes), hi=max(mixes), trace=trace, ret=p.tr)

if __name__ == "__main__":
    rows = [("staged (now)", dict(strategy="staged")),
            ("avg PI, pulses first", dict(strategy="avg")),
            ("avg PI, no pulses", dict(strategy="avg", pulses=False)),
            ("avg PI, min move 6%", dict(strategy="avg", min_move=0.06)),
            ("avg PI, min move 2%", dict(strategy="avg", min_move=0.02))]
    print(f"{'strategy':24s} {'start':>5s} | {'mean mix':>8s} {'range':>11s} {'>45 s/8h':>9s} {'longest >43':>11s} {'moves/h':>7s} {'pump st/h':>9s} {'heat':>5s} {'free%':>5s}")
    res = {}
    for start in (60.0, 20.0):
        for name, kw in rows:
            r = sim(boiler_start=start, **kw)
            res[(name, start)] = r
            print(f"{name:24s} {start:5.0f} | {r['mean']:8.1f} {r['lo']:5.1f}-{r['hi']:4.1f} {r['over45']:9d} {r['max_over']:9d} s {r['moves']:7.0f} {r['pump_starts']:9.1f} {r['heat']:5.0f} {r['free']:5.0f}")
    if len(sys.argv) > 1:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, axs = plt.subplots(2, 1, figsize=(11, 7), dpi=110, sharex=True)
        for ax, name in zip(axs, ("staged (now)", "avg PI, pulses first")):
            r = res[(name, 60.0)]; L = r["trace"]; tt = [x[0] for x in L]
            ax.plot(tt, [x[1] for x in L], color="#e8590c", lw=1, label="boiler water")
            ax.plot(tt, [x[2] for x in L], color="#2b8a3e", lw=1.2, label="mixed flow")
            ax.plot(tt, [x[3] for x in L], color="#1c7ed6", lw=1, label="return")
            ax.plot(tt, [100 * x[6] for x in L], color="#868e96", lw=0.8, label="valve %")
            ax.fill_between(tt, 0, 4, where=[x[4] for x in L], color="#ffa94d", alpha=0.6, step="post", label="our CFH")
            ax.fill_between(tt, 4, 8, where=[x[5] for x in L], color="#74c0fc", alpha=0.7, step="post", label="boiler pump")
            ax.axhline(40, color="#2b8a3e", lw=0.6, ls=":"); ax.axhline(45, color="#c92a2a", lw=0.6, ls=":")
            ax.set_ylim(0, 75); ax.grid(alpha=0.3); ax.legend(fontsize=7, ncol=7, loc="upper right")
            ax.set_title(f"{name}: mean mix {r['mean']:.1f} °C, {r['moves']:.0f} servo moves/h, {r['pump_starts']:.1f} boiler-pump starts/h")
        axs[-1].set_xlabel("hours"); fig.tight_layout(); fig.savefig(sys.argv[1])
