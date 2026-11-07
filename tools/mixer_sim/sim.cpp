// Closed-loop check of mixer_control.h against a plant modelled on the 6 Nov 2026 test.
// Build: c++ -std=c++17 -O2 -o sim sim.cpp && ./sim [kp ti]
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include "../../mixer_control.h"

using namespace mixer_control;

struct Result {
  float overshoot, mae_late, max_mix, track_err, cal_cold_err, cal_stroke_err;
  int moves;
  bool cal_ok;
};

// Servo + gearbox + valve, as measured on 7 Nov 2026 (angles in servo degrees).
struct Servo {
  float th = 105;          // shaft angle; potentiometer window is 0..210 of each turn
  float th_cold = -37, th_hot = 393;  // end stops (stroke 430°), valve at 1/3 when th = 105
  float play = 15;         // lever play (servo degrees)
  float valve_th = 105;    // valve side of the play
  float vel = 0;           // deg/s
  unsigned rng = 12345;
  int raw() {
    float ph = std::fmod(th, 360.0f);
    if (ph < 0) ph += 360;
    if (ph < 210) return std::clamp(int(std::lround(1023 - ph * 1024 / 210)), 0, 1023);
    if (ph < 230) return 0;      // sticks just past the bottom edge
    if (ph > 340) return 1019;   // and just before the top edge
    rng = rng * 1103515245 + 12345;
    return (rng >> 16) % 1024;   // anything in between
  }
  float opening() const { return (valve_th - th_cold) / (th_hot - th_cold); }
  bool stiff() const { float o = (th - th_cold) / (th_hot - th_cold); return o > 0.35f && o < 0.65f; }
  // duty in %; returns true if pushing against an end stop
  bool step(float duty, float dt) {
    const float stat = stiff() ? 35 : 15, dyn = stiff() ? 20 : 10;
    float target_vel = 0;
    if (std::fabs(duty) > (std::fabs(vel) > 1 ? dyn : stat))
      target_vel = (duty > 0 ? 1 : -1) * 4.5f * (std::fabs(duty) - dyn);  // ~90 deg/s at 30 % unloaded
    vel += (target_vel - vel) * std::min(1.0f, dt / 0.08f);  // spin-up / coast-down ~80 ms
    float nth = th + vel * dt;
    bool at_end = false;
    if (nth <= th_cold) { nth = th_cold; vel = 0; at_end = duty < 0; }
    if (nth >= th_hot) { nth = th_hot; vel = 0; at_end = duty > 0; }
    th = nth;
    if (th > valve_th + play / 2) valve_th = th - play / 2;
    if (th < valve_th - play / 2) valve_th = th + play / 2;
    valve_th = std::clamp(valve_th, th_cold, th_hot);
    return at_end;
  }
};

// One full scenario: calibrate, then control the mixed flow against the 6 Nov plant.
Result run(float kp, float ti, int valve_curve, float play_deg, bool print, float boiler_tau = 300) {
  Settings cfg;
  cfg.setpoint = 40;
  cfg.kp = kp;
  cfg.ti = ti;
  Controller c;
  Positioner pz;
  PosConfig pc;
  Servo sv;
  sv.play = play_deg;
  const float duty_pct = 30, kick_pct = 60;
  pz.request_calibration();
  const float dt = 0.02f;
  float tb = 18, tr = 16, probe = 18, p = 1;
  std::deque<float> probe_delay(250, 18.0f), ret_delay(9000, 16.0f);
  float overshoot = 0, mae = 0, max_mix = 0, track = 0;
  int n_late = 0, n_track = 0, moves = 0;
  // Executor (as in scs_servo): drive, poll every 100 ms, stop check, 600 ms coast.
  enum { IDLE, DRIVE, COAST } ex = IDLE;
  Positioner::Move mv;
  float t_run = 0, t_coast = 0, t_poll = 0;
  bool stopped = false;
  const float t_end = 5400;
  bool control_started = false;
  float t_cal_done = -1;
  for (float t = 0; t < t_end; t += dt) {
    float duty = 0;
    if (ex == DRIVE) duty = mv.dir * (t_run < 0.2f ? kick_pct : duty_pct);
    sv.step(duty, dt);
    if (ex != IDLE) {
      t_poll += dt;
      if (ex == DRIVE) t_run += dt; else t_coast += dt;
      if (t_poll >= 0.1f) {
        pz.tracker().sample(sv.raw(), mv.dir, t_poll, ex == DRIVE);
        pz.watch_calibration();
        t_poll = 0;
        if (ex == DRIVE && pz.should_stop(mv, pc)) { stopped = true; ex = COAST; t_coast = 0; }
        else if (ex == COAST && t_coast >= 0.6f) {
          ex = IDLE;
          // stalled = stopped early while driving into an end
          pz.done(mv, stopped && pz.stopped_on_stall(), sv.raw(), &pc);
          if (pz.take_calibrated()) {
            t_cal_done = t;
            if (print) printf("   calibrated at %.0f s: cold_x %.0f stroke_x %.0f (true stroke %.0f)\n", t, pc.cold_x,
                              pc.stroke_x, (sv.th_hot - sv.th_cold) * 1024 / 210);
          }
        }
      }
      if (ex == DRIVE && t_run >= mv.max_s) { stopped = false; ex = COAST; t_coast = 0; }
    }
    if (pz.known()) control_started = true;
    // Plant: CFH once the valve is under control.
    float tb_target = control_started ? 62 : 18;
    if (t >= 1900) p = 0.75f;
    if (t >= 2000 && t < 2300) tb_target = 48;
    tb += (tb_target - tb) * dt / (t >= 2000 && t < 2300 ? 120 : boiler_tau);
    float u = std::clamp(sv.opening(), 0.0f, 1.0f);
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
    static float t_ctl = 0, t_pos = 0;
    if ((t_ctl += dt) >= 5) {
      t_ctl = 0;
      Inputs in;
      in.boiler = std::round(tb * 16) / 16;
      in.ret = std::round(tr * 16) / 16;
      in.mixed = std::round(probe * 16) / 16;
      in.pump_on = true;
      in.automatic = pz.known();
      in.dt = 5;
      c.update(in, cfg);
    }
    if ((t_pos += dt) >= 1 && ex == IDLE) {
      t_pos = 0;
      if (pz.next(c.opening(), pz.known(), pc, &mv)) { ex = DRIVE; t_run = 0; t_poll = 0; stopped = false; moves++; }
    }
    if (pz.known() && ex == IDLE) {
      track = std::max(track, std::fabs(pz.opening(pc) - sv.opening()));
      n_track++;
    }
    if (t > 900) overshoot = std::max(overshoot, mix - cfg.setpoint);
    if (control_started) max_mix = std::max(max_mix, mix);
    if (t > 1200 && t < 1900) { mae += std::fabs(mix - cfg.setpoint); n_late++; }
    if (print && std::fmod(t, 300) < dt)
      printf("%4.0f min  boiler %5.1f  ret %5.1f  mixed %5.1f  target %3.0f%%  est %3.0f%%  valve %3.0f%%  raw %4d %s\n",
             t / 60, tb, tr, mix, c.opening() * 100, pz.known() ? pz.opening(pc) * 100 : -1, sv.opening() * 100,
             sv.raw(), status_text(c.status()));
  }
  const float true_stroke = (sv.th_hot - sv.th_cold) * 1024 / 210;
  Result r;
  r.overshoot = overshoot;
  r.mae_late = n_late ? mae / n_late : NAN;
  r.max_mix = max_mix;
  r.track_err = track;
  r.cal_ok = t_cal_done > 0;
  r.cal_stroke_err = (pc.stroke_x - true_stroke) / true_stroke;
  r.cal_cold_err = 0;
  r.moves = moves;
  return r;
}

// Over-temperature latch and missing-probe rules, and idle tracking.
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
  // Tracker follows movement it didn't cause, through the gap (sticky edges, random middle).
  {
    Servo sv;
    sv.th = 150;  // in window 0
    Tracker tr;
    tr.set(150 * 1024 / 210.0f);
    for (int i = 0; i < 100; i++) {  // pushed hot ~2.5 deg per 0.25 s poll, right through the gap
      sv.th += 2.5f;
      tr.sample(sv.raw(), 0, 0.25f, false);
    }
    const float truth = sv.th * 1024 / 210;
    assert(tr.known() && !tr.in_gap() && std::fabs(tr.x() - truth) < 30);
    for (int i = 0; i < 100; i++) {  // and back again
      sv.th -= 2.5f;
      tr.sample(sv.raw(), 0, 0.25f, false);
    }
    assert(!tr.in_gap() && std::fabs(tr.x() - sv.th * 1024 / 210) < 30);
    // Sitting still in the gap with random readings must not invent an exit.
    sv.th = 290;
    tr.set(290 * 1024 / 210.0f);
    for (int i = 0; i < 4000; i++) tr.sample(sv.raw(), 0, 0.25f, false);
    assert(tr.in_gap());
  }
  printf("safety checks passed\n\n");
}

int main(int argc, char **argv) {
  safety_checks();
  float kp = argc > 2 ? atof(argv[1]) : 2.5f, ti = argc > 2 ? atof(argv[2]) : 90;
  printf("S-curve valve, 15 deg lever play, boiler already hot (as on 7 Nov):\n");
  run(kp, ti, 1, 15, true, 60);
  printf("\n case                     | cal  stroke err | max mix  overshoot  err(20-32min) | worst est. error  moves/hr\n");
  for (float tau : {300.0f, 60.0f})
  for (float play : {0.0f, 15.0f, 30.0f})
    for (int curve : {0, 1}) {
      Result r = run(kp, ti, curve, play, false, tau);
      printf(" %s %-7s valve, play %2.0f deg | %s   %+5.1f%%    |  %5.1f    %5.1f       %5.2f       |      %4.1f%%        %3.0f\n",
             tau < 100 ? "hot boiler " : "cold boiler", curve ? "S-curve" : "linear", play, r.cal_ok ? "ok " : "BAD", r.cal_stroke_err * 100, r.max_mix,
             r.overshoot, r.mae_late, r.track_err * 100, r.moves / 1.5f);
    }
}
