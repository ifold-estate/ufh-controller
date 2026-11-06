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
};

// valve_curve: 0 = linear blend, 1 = strongly non-linear (feed-forward is wrong)
Result run(float kp, float ti, int valve_curve, bool print) {
  Settings cfg;
  cfg.setpoint = 40;
  cfg.kp = kp;
  cfg.ti = ti;
  Controller c;
  c.set_opening(0.5f);
  const float dt = 1;  // plant step, s
  float tb = 18, tr = 16, probe = 18, p = 1;
  std::deque<float> probe_delay(5, 18.0f), ret_delay(180, 16.0f);
  float mixed_meas = 18, overshoot = 0, mae = 0, max_mix = 0;
  int n_late = 0;
  for (int t = 0; t < 3600; t++) {
    // Boiler: CFH at t=0, heats towards 62 (tau 150 s). Radiators start at 1700 s: boiler-side
    // pressure drops (p) and their cold water dips the boiler 12 °C, recovering by ~2700 s.
    float tb_target = 62;
    if (t >= 1700) p = 0.75f;
    if (t >= 1800 && t < 2100) tb_target = 48;
    tb += (tb_target - tb) * dt / (t >= 1800 && t < 2100 ? 120 : 300);
    // Valve: share of boiler water for opening u.
    float u = c.opening();
    float h = valve_curve ? u * u * (3 - 2 * u) : u;  // S-curve if non-linear
    h = p * h / (p * h + (1 - h) + 1e-6f);
    float mix = h * tb + (1 - h) * tr;
    // Floor: return comes back 3 min later, ~9 °C cooler than the flow, slab lag 20 min.
    ret_delay.push_back(mix);
    float back = ret_delay.front();
    ret_delay.pop_front();
    tr += ((back - 9) - tr) * dt / 1200;
    // Probe: 5 s transport + 15 s thermal lag; DS18B20 read every 5 s (1/16 °C steps).
    probe_delay.push_back(mix);
    probe += (probe_delay.front() - probe) * dt / 15;
    probe_delay.pop_front();
    if (t % 5 == 0) {
      mixed_meas = std::round(probe * 16) / 16;
      Inputs in;
      in.boiler = std::round(tb * 16) / 16;
      in.ret = std::round(tr * 16) / 16;
      in.mixed = mixed_meas;
      in.pump_on = true;
      in.automatic = true;
      in.dt = 5;
      c.update(in, cfg);
    }
    if (t > 300) overshoot = std::max(overshoot, mix - cfg.setpoint);
    max_mix = std::max(max_mix, mix);
    if (t > 600 && t < 1700) { mae += std::fabs(mix - cfg.setpoint); n_late++; }
    if (print && t % 60 == 0)
      printf("%4d min  boiler %5.1f  return %5.1f  mixed %5.1f  opening %3.0f%%  %s\n", t / 60, tb, tr, mix,
             c.opening() * 100, status_text(c.status()));
  }
  return {overshoot, mae / n_late, max_mix};
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
  float kp = argc > 2 ? atof(argv[1]) : 1.0f, ti = argc > 2 ? atof(argv[2]) : 120;
  run(kp, ti, 1, true);
  printf("\n kp   ti  | linear valve: max-mix overshoot err(10-28min) | S-curve valve: same\n");
  for (float k : {0.5f, 1.0f, 2.0f, 3.0f})
    for (float i : {60.0f, 120.0f, 240.0f}) {
      Result a = run(k, i, 0, false), b = run(k, i, 1, false);
      printf(" %.1f %4.0f  |        %5.1f  %5.1f  %5.2f            |        %5.1f  %5.1f  %5.2f\n", k, i,
             a.max_mix, a.overshoot, a.mae_late, b.max_mix, b.overshoot, b.mae_late);
    }
}
