// Closed-loop check of mixer_control.h against a plant modelled on the 6 Nov 2026 test.
// Build: c++ -std=c++17 -O2 -o sim sim.cpp && ./sim [kp ti]
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include "../../mixer_control.h"

using namespace mixer_control;

struct Result {
  float overshoot, mae_late, max_mix;
  int moves;
};

struct Valve {
  // Motor-side lever position l (0..1, hard stops) and valve position v, which lags l by the play.
  float l = 0.6f, v = 0.6f, play = 0;
  float stroke_s = 40;  // true cold->hot run time
};

// valve_curve: 0 linear, 1 S-curve. stroke_err: configured stroke / true stroke. play: lever play
// (fraction of stroke). calibrate: measure the stroke first instead of trusting the setting.
Result run(float kp, float ti, int valve_curve, float stroke_err, float play, bool calibrate, bool print) {
  Settings cfg;
  cfg.setpoint = 40;
  cfg.kp = kp;
  cfg.ti = ti;
  Controller c;
  Positioner pos;
  Valve vv;
  vv.play = play;
  PosConfig pc;
  pc.stroke_s = vv.stroke_s * stroke_err;
  if (calibrate) pos.request_calibration(); else pos.request_home();
  const float dt = 0.1f;
  float tb = 18, tr = 16, probe = 18, p = 1;
  std::deque<float> probe_delay(50, 18.0f), ret_delay(1800, 16.0f);
  float overshoot = 0, mae = 0, max_mix = 0;
  int n_late = 0, moves = 0;
  // Executor state
  bool running = false;
  Positioner::Move mv;
  float run_t = 0;
  for (int k = 0; k < 36000 * 1.5; k++) {
    const float t = k * dt;
    // Boiler (CFH at 120 s, after homing), radiators at 1800 s as before.
    float tb_target = t < 120 ? 18 : 62;
    if (t >= 1820) p = 0.75f;
    if (t >= 1920 && t < 2220) tb_target = 48;
    tb += (tb_target - tb) * dt / (t >= 1920 && t < 2220 ? 120 : 300);
    // Executor: motor runs at 1/stroke per second; stalls at the stops.
    bool at_end = false;
    if (running) {
      run_t += dt;
      float nl = vv.l + mv.dir * dt / vv.stroke_s;
      if (nl <= 0 || nl >= 1) { nl = std::clamp(nl, 0.0f, 1.0f); at_end = true; }
      vv.l = nl;
      float half = vv.play / 2;
      if (vv.l > vv.v + half) vv.v = vv.l - half;
      if (vv.l < vv.v - half) vv.v = vv.l + half;
      if (at_end || run_t >= mv.seconds) {
        running = false;
        pos.done(mv, run_t, at_end, pc);
        float st = pos.take_measured_stroke();
        if (st > 0) { pc.stroke_s = st; if (print) printf("   calibrated stroke %.1f s (true %.1f)\n", st, vv.stroke_s); }
      }
    }
    float u = std::clamp(vv.v, 0.0f, 1.0f);
    float h = valve_curve ? u * u * (3 - 2 * u) : u;
    h = p * h / (p * h + (1 - h) + 1e-6f);
    float mix = h * tb + (1 - h) * tr;
    ret_delay.push_back(mix);
    float back = ret_delay.front();
    ret_delay.pop_front();
    tr += ((back - 9) - tr) * dt / 1200;
    probe_delay.push_back(mix);
    probe += (probe_delay.front() - probe) * dt / 15;
    probe_delay.pop_front();
    if (k % 50 == 0) {  // controller every 5 s
      Inputs in;
      in.boiler = std::round(tb * 16) / 16;
      in.ret = std::round(tr * 16) / 16;
      in.mixed = std::round(probe * 16) / 16;
      in.pump_on = true;
      in.automatic = pos.known();
      in.dt = 5;
      c.update(in, cfg);
    }
    if (k % 10 == 0 && !running) {  // positioner every 1 s
      if (pos.next(c.opening(), pos.known(), pc, &mv)) { running = true; run_t = 0; moves++; }
    }
    if (t > 600) overshoot = std::max(overshoot, mix - cfg.setpoint);
    if (t > 300) max_mix = std::max(max_mix, mix);
    if (t > 900 && t < 1800) { mae += std::fabs(mix - cfg.setpoint); n_late++; }
    if (print && k % 600 == 0)
      printf("%4.0f min  boiler %5.1f  return %5.1f  mixed %5.1f  target %3.0f%%  est %3.0f%%  valve %3.0f%%  %s\n",
             t / 60, tb, tr, mix, c.opening() * 100, pos.position() * 100, vv.v * 100, status_text(c.status()));
  }
  return {overshoot, mae / n_late, max_mix, moves};
}

// Over-temperature latch and missing-probe rules.
void safety_checks() {
  Settings cfg;  // limit 45
  Controller c;
  auto step = [&](float mixed, bool automatic) {
    Inputs in;
    in.boiler = 60; in.ret = 30; in.mixed = mixed; in.pump_on = true; in.automatic = automatic; in.dt = 5;
    return c.update(in, cfg);
  };
  step(40, true);
  assert(!c.over_temp());
  assert(step(45.5f, true) && c.over_temp() && c.opening() == 0);  // trips, valve to cold
  step(41, true);
  assert(c.over_temp());                                             // latched until 5 °C under
  step(39.9f, true);
  assert(!c.over_temp());
  step(46, false);
  assert(c.over_temp());                                             // trips in manual too
  step(30, false);
  assert(!c.over_temp());
  for (int i = 0; i < 5; i++) step(NAN, false);
  assert(!c.over_temp());                                            // 25 s missing: not yet
  step(NAN, false);
  assert(c.over_temp());                                             // 30 s missing: trips
  step(42, false);
  assert(c.over_temp());                                             // back, but not 5 °C under
  step(38, false);
  assert(!c.over_temp());
  printf("safety checks passed\n\n");
}

int main(int argc, char **argv) {
  safety_checks();
  float kp = argc > 2 ? atof(argv[1]) : 2.5f, ti = argc > 2 ? atof(argv[2]) : 90;
  printf("S-curve valve, calibrated stroke, 3%% lever play (backlash not compensated):\n");
  run(kp, ti, 1, 1.0f, 0.03f, true, true);
  printf("\n case                                   | max mix  overshoot  err(15-30min)  moves/hr\n");
  struct Case { const char *name; int curve; float err, play; bool cal; };
  for (Case cs : {Case{"linear, exact stroke, no play", 0, 1.0f, 0.0f, false},
                  Case{"S-curve, calibrated, 3% play", 1, 1.0f, 0.03f, true},
                  Case{"S-curve, stroke set 15% long, 3% play", 1, 1.15f, 0.03f, false},
                  Case{"S-curve, stroke set 15% short, 6% play", 1, 0.85f, 0.06f, false}}) {
    Result r = run(kp, ti, cs.curve, cs.err, cs.play, cs.cal, false);
    printf(" %-38s |  %5.1f    %5.1f      %5.2f        %3.0f\n", cs.name, r.max_mix, r.overshoot, r.mae_late,
           r.moves / 1.5f);
  }
}
