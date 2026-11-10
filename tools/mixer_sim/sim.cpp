// Closed-loop check of mixer_control.h against a plant modelled on the 6 Nov 2026 test.
// Build: c++ -std=c++17 -O2 -o sim sim.cpp && ./sim [kp ti]
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include "../../mixer_control.h"
#include "../../heating_demand.h"

using namespace mixer_control;

struct Result {
  float overshoot, mae_late, max_mix, track_err, cal_cold_err, cal_stroke_err;
  int moves;
  bool cal_ok;
};

// Servo + gearbox + valve, as measured on 7 Nov 2026 (angles in servo degrees).
struct Servo {
  float th = 105;          // shaft angle; potentiometer window is 0..210 of each turn
  float th_cold = -20, th_hot = 408;  // end stops, just beyond the cam end switches (x 0 and 1956)
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
  // Cam switches as measured: state from the true shaft position, changing at the "going hot" edge
  // when moving hot and the "going cold" edge when moving cold.
  uint8_t cam_state = 0b1010;
  void update_cam() {
    const float x = th * 1024 / 210;
    for (const auto &e : cam::EDGES) {
      const bool hot = cam::zone(e.to) && cam::zone(e.from) && cam::zone(e.to)->lo > cam::zone(e.from)->lo;
      if (e.from == cam_state && ((hot && x >= e.x) || (!hot && x <= e.x))) { cam_state = e.to; return; }
    }
  }
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
    update_cam();
    if (th > valve_th + play / 2) valve_th = th - play / 2;
    if (th < valve_th - play / 2) valve_th = th + play / 2;
    valve_th = std::clamp(valve_th, th_cold, th_hot);
    return at_end;
  }
};

// One full scenario: calibrate, then control the mixed flow against the 6 Nov plant.
Result run(float kp, float ti, int valve_curve, float play_deg, bool print, float boiler_tau = 300, bool use_cam = false, bool inject_turn_loss = false) {
  Settings cfg;
  cfg.setpoint = 40;
  cfg.kp = kp;
  cfg.ti = ti;
  // valve_curve: 0 linear, 1 S-curve (controller linear); 2 measured curve (controller uses it);
  // 3 valve 15 % steeper than measured (controller uses the measured curve); 4 measured valve but
  // the controller assumes linear (as on 7 Nov).
  cfg.valve_curve = valve_curve == 2 || valve_curve == 3;
  Controller c;
  Positioner pz;
  PosConfig pc;
  Servo sv;
  sv.play = play_deg;
  const float duty_pct = 30, kick_pct = 60;
  if (use_cam) {
    pc.cold_x = 0;
    pc.stroke_x = 1956;  // switch to switch
    pz.request_home();
  } else {
    pz.request_calibration();
  }
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
    if (ex == DRIVE) duty = mv.dir * (t_run < pc.kick_s ? kick_pct : duty_pct);
    sv.step(duty, dt);
    if (ex != IDLE) {
      t_poll += dt;
      if (ex == DRIVE) t_run += dt; else t_coast += dt;
      if (t_poll >= 0.1f) {
        pz.tracker().sample(sv.raw(), mv.dir, t_poll, ex == DRIVE);
        pz.watch_calibration();
        if (use_cam) pz.cam(sv.cam_state);
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
    // Fault injection: the tracker loses a whole turn (as in the 7 Nov retest).
    if (use_cam && inject_turn_loss && std::fabs(t - 2500) < dt / 2) pz.tracker().set(pz.tracker().x() - TURN);
    // Plant: CFH once the valve is under control.
    float tb_target = control_started ? 62 : 18;
    if (t >= 1900) p = 0.75f;
    if (t >= 2000 && t < 2300) tb_target = 48;
    tb += (tb_target - tb) * dt / (t >= 2000 && t < 2300 ? 120 : boiler_tau);
    float u = std::clamp(sv.opening(), 0.0f, 1.0f);
    float h = valve_curve == 0 ? u
              : valve_curve == 1 ? u * u * (3 - 2 * u)
              : valve_curve == 3 ? valve::share(std::min(1.0f, u * 1.15f))
                                 : valve::share(u);
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
  r.cal_ok = use_cam || t_cal_done > 0;
  r.cal_stroke_err = use_cam ? 0 : (pc.stroke_x - true_stroke) / true_stroke;
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
  // Band mode: CFH follows the band, the cap holds it off, valve keeps the mix under band_high.
  {
    Settings cfg;
    cfg.band = true;
    Controller c;
    auto step = [&](float boiler, float mixed, float ret) {
      Inputs in;
      in.boiler = boiler; in.mixed = mixed; in.ret = ret; in.pump_on = true; in.automatic = true; in.dt = 5;
      c.update(in, cfg);
    };
    step(20, 20, 18);
    assert(c.cfh() && c.status() == Status::HEATING);
    // valve: share = (42-18)/(70-18) = 0.46 -> opening on the curve
    assert(std::fabs(c.share() - 24.0f / 52) < 0.01f);
    step(60, 42.5, 30);
    assert(!c.cfh() && c.status() == Status::FREE_WHEEL);  // top of the band: free-wheel
    // free-wheel valve: (42-30)/(60-30) = 0.4 for the boiler's actual temperature
    assert(std::fabs(c.share() - 0.4f) < 0.01f);
    step(50, 38, 30);
    assert(!c.cfh());                                       // still in the band
    step(45, 34.5, 30);
    assert(c.cfh());                                        // bottom: heat again
    cfg.cap = true;                                         // cap 50, restart 40
    step(50.5, 37, 30);
    assert(!c.cfh() && c.status() == Status::BOILER_CAP);
    step(45, 37, 30);
    assert(!c.cfh());                                       // not yet 10 below
    step(39.5, 37, 30);
    assert(c.cfh());
    step(80, 37, 30);                                       // radiators drove the boiler hot
    assert(std::fabs(c.share() - 12.0f / 50) < 0.01f);      // valve sized for 80 °C
    step(80, 46, 30);
    assert(!c.cfh() && c.over_temp());                      // safety still wins
  }
  // Staged mode: pulses while stored heat lasts, then a full call; pump start closes the valve.
  {
    Settings cfg;
    cfg.staged = true;  // band_low 35, band_high 42, ret_target 33, pulse 195 s, gap 30 s
    Controller c;
    auto step = [&](float boiler, float mixed, float ret, float dt = 5) {
      Inputs in;
      in.boiler = boiler; in.mixed = mixed; in.ret = ret; in.pump_on = true; in.automatic = true; in.dt = dt;
      c.update(in, cfg);
    };
    step(60, 40, 28);
    assert(!c.cfh() && c.status() == Status::FREE_WHEEL);
    step(60, 34.9, 28);                                    // bottom of the band: start pulsing
    assert(c.cfh() && c.status() == Status::PULSING);
    // pulled curve aiming at 41 °C: share (41-28)/(60-28) = 0.41 -> opening ~0.52
    assert(c.opening() > 0.45f && c.opening() < 0.58f);
    // The boiler pump starts for another system during the pulse: mix +1.5 °C in 5 s.
    { const float before = c.opening(); step(60, 36.5f, 29); step(60, 38, 29);
      assert(c.opening() < before - 0.15f); }                // straight onto the pumped curve
    for (int i = 0; i < 37; i++) step(58, 38, 29);         // 195 s: the pulse ends
    assert(!c.cfh() && c.status() == Status::PULSING);     // in the 30 s gap
    for (int i = 0; i < 6; i++) step(55, 37, 29);          // gap over, boiler still warm: next pulse
    assert(c.cfh() && c.status() == Status::PULSING);
    for (int i = 0; i < 39; i++) step(40, 36, 33.0f - 0.5f);  // boiler water now only ~7.5 over return
    for (int i = 0; i < 6; i++) step(36, 35, 32);
    assert(c.cfh() && c.status() == Status::CALL_WAIT);    // stored heat used up: full call
    assert(std::fabs(c.opening() - cfg.wait_open) < 0.04f);
    step(37, 35.5, 32);
    step(45, 37.0, 32);                                    // +1.5 °C in 5 s = 0.3 °C/s: boiler pump
    assert(c.status() == Status::CALLING);
    assert(c.opening() < 0.25f);                           // closed sharply onto the pumped curve
    const float o = c.opening();
    for (int i = 0; i < 7; i++) step(62, 44, 30);          // over the 43 °C ceiling for 35 s
    assert(c.opening() < o);                               // stepped down
    for (int i = 0; i < 200; i++) step(60, 41, 33.5f);     // return reaches target after >= 15 min
    assert(!c.cfh() && c.status() == Status::FREE_WHEEL);
  }
  // Thermostat demand: staggered valves, pump after the actuators open, overrun, relay 6 modes.
  {
    using heating_demand::Demand; using heating_demand::R6Mode;
    using heating_demand::R4; using heating_demand::R5; using heating_demand::R6; using heating_demand::R7; using heating_demand::R8;
    heating_demand::Settings ds;
    Demand d;
    bool calls[4] = {false, false, false, false};
    auto run_for = [&](float sec) { for (int i = 0; i < int(sec); i++) d.update(calls, 1, ds); };
    run_for(5);
    assert(!d.pump() && !d.active() && !d.valve(R4) && !d.valve(R7));
    calls[0] = calls[3] = true;  // playroom and lounge call
    run_for(1);
    assert(d.valve(R7) != d.valve(R5));               // one valve at a time
    run_for(30);
    assert(d.valve(R7) && d.valve(R5));               // the second after the stagger
    assert(!d.pump());                                 // actuators still opening
    run_for(95);
    assert(d.pump());                                  // 2 min after the first opened
    calls[0] = false;                                  // playroom satisfied, lounge still calling
    run_for(1);
    assert(!d.valve(R7) && d.valve(R5) && d.pump());
    calls[3] = false;                                  // last call ends: overrun, valves stay open
    run_for(1);
    assert(d.pump() && d.valve(R5) && !d.active() && d.overrun());
    run_for(180);
    assert(!d.pump() && d.valve(R5));                  // pump stopped, valve closes 10 s later
    run_for(11);
    assert(!d.valve(R5));
    ds.r6 = R6Mode::N1_OR_N2;                          // kitchen centre follows N1 or N2
    calls[2] = true;
    run_for(40);
    assert(d.valve(R4) && d.valve(R6) && !d.valve(R8));
  }
  // On the cold end switch, with the estimate drifted a few % open: no more closing moves (8 Nov 19:47).
  {
    Positioner pz;
    PosConfig pc;
    pc.cold_x = -75;
    pc.stroke_x = 1956;
    pz.cam(cam::COLD_END);
    pz.cam(cam::COLD_END);
    pz.request_home();
    Positioner::Move m;
    assert(pz.next(0, true, pc, &m) && m.dir < 0);
    pz.done(m, true, 1000, &pc);
    assert(pz.known());
    pz.tracker().set(120);  // drifted: the estimate says ~10 % open
    assert(pz.opening(pc) > 0.05f);
    for (int i = 0; i < 50; i++) assert(!pz.next(0, true, pc, &m));
    assert(pz.next(0.5f, true, pc, &m) && m.dir > 0);  // opening still works
  }
  // Average mode (10 Nov 2026): continuous call, PI on the mix for a 40 °C average.
  {
    Settings cfg;
    cfg.average = true;
    cfg.setpoint = 40;
    cfg.limit = 50;
    auto mk = [&](Controller &c) {
      return [&c, &cfg](float boiler, float mixed, float ret, bool cfh_on = true, float dt = 5) {
        Inputs in;
        in.boiler = boiler; in.mixed = mixed; in.ret = ret; in.pump_on = true; in.automatic = true;
        in.cfh_on = cfh_on; in.dt = dt;
        return c.update(in, cfg);
      };
    };
    // No thermostat calling: the controller wants CFH, but it's held off; no valve moves.
    { Controller c; auto step = mk(c);
      assert(!step(60, 30, 28, false) && c.cfh() && c.status() == Status::NO_CALL); }
    // Waiting for the boiler pump: pulled curve, never wider than wait_open (61 %), even with a cold
    // boiler. The pump start (mix +1 °C in 5 s) goes straight onto the pumped curve.
    { Controller c; auto step = mk(c);
      for (int i = 0; i < 12; i++) step(30, 29, 28);
      assert(c.status() == Status::CALL_WAIT && c.opening() <= cfg.wait_open + 1e-3f && c.opening() > 0.55f);
      step(60, 30, 28); step(60, 31, 28);
      assert(c.avg_pumped() && c.status() == Status::CALLING && c.opening() < 0.25f); }
    // Over the wait limit without a visible pump start (cold boiler): taken as pumped anyway.
    { Controller c; auto step = mk(c);
      for (int i = 0; i < 61; i++) step(32, 30, 29);
      assert(c.avg_pumped()); }
    // Holds the average on a valve 25 % steeper than the curve it assumes: the integral finds it.
    { Controller c; auto step = mk(c);
      float mix = 30, sum = 0; int n = 0; float lo = 99, hi = 0;
      for (int i = 0; i < 6 * 720; i++) {          // 6 h
        const float boiler = 60 + 6 * std::sin(i * 5 / 600.0f), ret = 33;  // burner cycling
        const float truth = valve::share_on(valve::PUMPED, std::min(1.0f, c.opening() * 1.25f));
        mix += (ret + truth * (boiler - ret) - mix) * 5 / 30;  // probe lag
        step(boiler, mix, ret);
        if (i > 720) { sum += mix; n++; lo = std::min(lo, mix); hi = std::max(hi, mix); }
      }
      assert(std::fabs(sum / n - 40) < 0.5f && hi < 46 && !c.over_temp());
      printf("   average mode: mean %.2f °C, range %.1f-%.1f\n", sum / n, lo, hi); }
    // Anti-windup: 30 min pinned open with a cold boiler, then hot water: no lingering push to hot.
    { Controller c; auto step = mk(c);
      for (int i = 0; i < 360; i++) step(32, 30, 29);
      step(62, 30, 29);
      const float ff = (40 - 29.0f) / (62 - 29.0f);
      for (int i = 0; i < 12; i++) step(62, 40, 29);
      assert(std::fabs(c.share() - ff) < 0.05f); }
    // Soft limit: over 46 °C the valve closes 5 % per update, whatever the PI wants.
    { Controller c; auto step = mk(c);
      for (int i = 0; i < 70; i++) step(40, 30, 29);   // pumped, opening well up
      const float o = c.opening();
      step(60, 47, 29);
      assert(c.opening() <= o - 0.05f + 1e-4f);
      for (int i = 0; i < 40 && c.opening() > 0; i++) step(70, 47, 29);
      assert(c.opening() == 0 && c.cfh());              // shut; CFH still on...
      for (int i = 0; i < 23; i++) step(70, 47, 29);
      assert(c.cfh());
      step(70, 47, 29);                                  // ...until 2 min over with the valve shut
      assert(!c.cfh() && c.status() == Status::COOLING);
      step(70, 41, 29, false);
      assert(!c.cfh());                                  // until under the setpoint
      step(70, 39.5f, 29, false);
      assert(c.cfh()); }
    // Hard guard unchanged: 50 °C latches over-temp, valve to cold.
    { Controller c; auto step = mk(c);
      for (int i = 0; i < 70; i++) step(40, 30, 29);
      step(70, 50.5f, 29);
      assert(c.over_temp() && c.opening() == 0); }
  }
  printf("safety checks passed\n\n");
}

int main(int argc, char **argv) {
  safety_checks();
  float kp = argc > 2 ? atof(argv[1]) : 2.5f, ti = argc > 2 ? atof(argv[2]) : 90;
  printf("Measured valve curve, 5 deg play, hot boiler, cam switches:\n");
  run(kp, ti, 2, 5, true, 60, true, false);
  printf("\n case                     | cal  stroke err | max mix  overshoot  err(20-32min) | worst est. error  moves/hr\n");
  const char *vnames[] = {"linear", "S-curve", "measured, ctl uses curve", "15% steeper, ctl uses curve",
                          "measured, ctl assumes linear"};
  for (int vc : {2, 3, 4}) for (float tau : {300.0f, 60.0f}) {
    Result r = run(kp, ti, vc, 5, false, tau, true, false);
    printf(" CAM %s %-28s |          |  %5.1f    %5.1f       %5.2f       |      %4.1f%%        %3.0f\n",
           tau < 100 ? "hot " : "cold", vnames[vc], r.max_mix, r.overshoot, r.mae_late, r.track_err * 100, r.moves / 1.5f);
  }
  for (bool lose : {true}) for (float tau : {60.0f}) {
    Result r = run(kp, ti, 2, 5, false, tau, true, lose);
    printf(" CAM %s %-28s |          |  %5.1f    %5.1f       %5.2f       |      %4.1f%%        %3.0f\n",
           tau < 100 ? "hot " : "cold", "measured + turn lost", r.max_mix, r.overshoot, r.mae_late, r.track_err * 100,
           r.moves / 1.5f);
  }
  for (float tau : {300.0f, 60.0f})
  for (float play : {0.0f, 15.0f, 30.0f})
    for (int curve : {0, 1}) {
      Result r = run(kp, ti, curve, play, false, tau);
      printf(" %s %-7s valve, play %2.0f deg | %s   %+5.1f%%    |  %5.1f    %5.1f       %5.2f       |      %4.1f%%        %3.0f\n",
             tau < 100 ? "hot boiler " : "cold boiler", curve ? "S-curve" : "linear", play, r.cal_ok ? "ok " : "BAD", r.cal_stroke_err * 100, r.max_mix,
             r.overshoot, r.mae_late, r.track_err * 100, r.moves / 1.5f);
    }
}
