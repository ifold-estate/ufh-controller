// Mixed-flow temperature control for the UFH mixer valve, plus the flow over-temperature cut-out.
//
// Opening (0 = all return / cold, 1 = all boiler flow / hot) = feed-forward + PI trim.
//   - Feed-forward: the ideal blend, (setpoint - return) / (boiler - return). It reacts at once to the
//     boiler and return temperatures, which swing a lot (tests on 6 Nov 2026: return +16 °C as the
//     floor warms, boiler flow -12 °C when radiators elsewhere start).
//   - PI trim on the mixed-flow probe corrects what the blend formula gets wrong (valve
//     characteristic, boiler-side pressure changing when other circuits run).
//   - The integral only runs while it can help: not while the output is pinned at an end and the error
//     pushes further, and not while the boiler is too cold to reach the setpoint.
//
// Safety, independent of the auto switch: mixed flow over the limit, or no mixed reading for 30 s,
// latches "over-temperature": CFH must be off until the mix is 5 °C under the limit (and a reading is
// back). The caller enforces that, and in auto mode also drives the valve to cold.
//
// Plain C++ so tools/mixer_sim can exercise it on a PC.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mixer_control {

// Measured valve characteristic (7 Nov 2026, docs/valve_curve_2026-11-07.png): share of boiler
// water in the mix against opening (fraction of the travel between the cam end switches). Nearly all
// the mixing happens between ~10 % and ~40 %; above that the valve passes almost only boiler water,
// and even closed it lets ~12 % through. The controller works in terms of share and converts
// through this curve, which also makes its corrections gentle where the valve is steep.
namespace valve {
struct Pt {
  float opening, share;
};
constexpr Pt CURVE[] = {{0.00f, 0.12f}, {0.10f, 0.17f}, {0.15f, 0.20f}, {0.21f, 0.30f}, {0.27f, 0.44f},
                        {0.32f, 0.71f}, {0.35f, 0.85f}, {0.38f, 0.97f}, {0.42f, 1.00f}, {1.00f, 1.00f}};
constexpr int N = sizeof(CURVE) / sizeof(CURVE[0]);
inline float share(float opening) {
  if (opening <= CURVE[0].opening) return CURVE[0].share;
  for (int i = 1; i < N; i++)
    if (opening <= CURVE[i].opening) {
      const float f = (opening - CURVE[i - 1].opening) / (CURVE[i].opening - CURVE[i - 1].opening);
      return CURVE[i - 1].share + f * (CURVE[i].share - CURVE[i - 1].share);
    }
  return CURVE[N - 1].share;
}
// Smallest opening giving at least this share.
inline float opening(float share) {
  if (share <= CURVE[0].share) return CURVE[0].opening;
  for (int i = 1; i < N; i++)
    if (share <= CURVE[i].share) {
      const float f = (share - CURVE[i - 1].share) / (CURVE[i].share - CURVE[i - 1].share);
      return CURVE[i - 1].opening + f * (CURVE[i].opening - CURVE[i - 1].opening);
    }
  for (int i = 0; i < N; i++)
    if (CURVE[i].share >= CURVE[N - 1].share) return CURVE[i].opening;
  return 1;
}
// Measured 8 Nov 2026 with one zone open. "Pulled": 2-port zone valve open but the boiler pump not
// running, so only our pump draws boiler water. "Pumped": the boiler pump running (steeper than the
// five-zone CURVE above; used as the conservative case).
constexpr Pt PULLED[] = {{0.00f, 0.00f}, {0.20f, 0.05f}, {0.40f, 0.22f}, {0.61f, 0.50f}, {0.80f, 0.60f}, {1.00f, 0.65f}};
constexpr Pt PUMPED[] = {{0.00f, 0.10f}, {0.05f, 0.15f}, {0.10f, 0.24f}, {0.15f, 0.42f}, {0.21f, 0.66f},
                         {0.30f, 0.85f}, {0.40f, 0.95f}, {1.00f, 1.00f}};
template<size_t M> float opening_on(const Pt (&c)[M], float share) {
  if (share <= c[0].share) return c[0].opening;
  for (size_t i = 1; i < M; i++)
    if (share <= c[i].share)
      return c[i].share > c[i - 1].share
                 ? c[i - 1].opening + (share - c[i - 1].share) / (c[i].share - c[i - 1].share) * (c[i].opening - c[i - 1].opening)
                 : c[i - 1].opening;
  return c[M - 1].opening;
}
}  // namespace valve

struct Settings {
  float setpoint = 35;    // °C mixed flow
  float limit = 45;       // °C over-temperature cut-out
  float kp = 1.0;         // % boiler-water share per °C error
  float ti = 120;         // s integral time
  float max_step = 0.05;  // max share change per update (fraction)
  bool valve_curve = true;  // convert share -> opening through the measured curve (false: linear)
  // Band mode (simulated 7 Nov 2026, tools/mixer_sim/band_sim.py): instead of holding the setpoint,
  // let the mix swing between band_low and band_high by switching CFH (the controller owns CFH),
  // with the valve set coarsely so the mix can't pass band_high.
  bool band = false;
  float band_low = 35, band_high = 42;
  float boiler_max = 70;       // hottest the boiler is assumed to get when we call for heat
  bool cap = false;            // also drop CFH when the boiler flow reaches cap_temp...
  float cap_temp = 50;
  float cap_restart = 10;      // ...and call again this far below it
  float cap_overshoot = 8;     // the boiler keeps heating ~2 min after CFH drops
  float band_reset = 0.08f;    // re-open the valve only when its ideal opening is this much higher
  // Staged mode (simulated 8 Nov 2026, tools/mixer_sim/stage_sim.py): the heating phase starts at
  // band_low and ends when the return reaches ret_target. First free pulses: CFH calls shorter than
  // the upstream controller's 3 min 50 s timer (which resets when the call drops) open the 2-port
  // valve so our pump pulls the boiler's stored heat without starting the boiler; then, once the
  // boiler water arriving is no longer warm, one full call that waits with the valve open, closes
  // sharply when the boiler pump starts, and holds the mix under the ceiling (band_high).
  bool staged = false;
  float ret_target = 33;
  float pulse_s = 195, gap_s = 30;
  float stored_margin = 8;     // boiler water this much above the return still counts as stored heat
  float wait_open = 0.61f;     // valve while the full call waits for the boiler pump
  float pump_rate = 0.15f;     // mix rising this fast (°C/s) = the boiler pump has started
  float max_wait_s = 300;
  float min_call_s = 900;
  float boiler_expect_rise = 20;  // °C over setpoint the boiler is assumed to reach while it warms
};

struct Inputs {
  float boiler = NAN, mixed = NAN, ret = NAN;  // °C, NAN when unavailable
  bool pump_on = false;                        // floor pump running
  bool automatic = false;                      // "Mixer auto" switch
  float dt = 5;                                // s since the last update
};

enum class Status : uint8_t {
  OFF, PUMP_OFF, PROBE_FAULT, OVER_TEMP, WAITING_BOILER, WARMING, HOLDING, COOLING,
  HEATING, BOILER_CAP, FREE_WHEEL,  // band mode
  PULSING, CALL_WAIT, CALLING,      // staged mode
};

inline const char *status_text(Status s) {
  switch (s) {
    case Status::OFF: return "Idle";
    case Status::PUMP_OFF: return "Pump off";
    case Status::PROBE_FAULT: return "Probe fault";
    case Status::OVER_TEMP: return "Over-temp";
    case Status::WAITING_BOILER: return "Waiting";
    case Status::WARMING: return "Warming";
    case Status::HOLDING: return "Holding";
    case Status::COOLING: return "Cooling";
    case Status::HEATING: return "Heating";
    case Status::BOILER_CAP: return "Boiler cap";
    case Status::FREE_WHEEL: return "Free-wheel";
    case Status::PULSING: return "Pulsing";
    case Status::CALL_WAIT: return "Call (waiting)";
    case Status::CALLING: return "Calling";
  }
  return "";
}

class Controller {
 public:
  // Returns true if the valve opening should be driven to opening() this update.
  bool update(const Inputs &in, const Settings &cfg) {
    // --- Safety (always) ---
    if (std::isnan(in.mixed)) {
      missing_s_ += in.dt;
    } else {
      missing_s_ = 0;
    }
    if (!std::isnan(in.mixed) && in.mixed > cfg.limit) over_temp_ = true;
    if (missing_s_ >= 30) over_temp_ = true;
    if (over_temp_ && !std::isnan(in.mixed) && in.mixed < cfg.limit - 5) over_temp_ = false;

    // --- Control ---
    cfh_ = false;
    if (!in.automatic) {
      integral_ = 0;
      running_ = false;
      band_started_ = false;
      stage_ = Stage::IDLE;
      stage_ = Stage::IDLE;
      status_ = over_temp_ ? Status::OVER_TEMP : Status::OFF;
      return false;
    }
    if (over_temp_) {
      integral_ = 0;
      running_ = false;
      opening_ = share_ = 0;
      status_ = Status::OVER_TEMP;
      return true;
    }
    if (std::isnan(in.mixed)) {
      status_ = Status::PROBE_FAULT;
      return false;  // hold position; the 30 s rule above cuts CFH if it persists
    }
    if (!in.pump_on) {
      integral_ = 0;
      running_ = false;
      band_started_ = false;
      status_ = Status::PUMP_OFF;
      return false;  // hold position, ready for the next start
    }

    if (cfg.staged) return staged_(in, cfg);
    if (cfg.band) return band_(in, cfg);

    const float sp = cfg.setpoint;
    const bool boiler_cold = !std::isnan(in.boiler) && in.boiler < sp + 2;
    // Boiler flow rising quickly (warm-up): the mix will follow on its own, and the mixed probe lags.
    if (!std::isnan(in.boiler) && !std::isnan(last_boiler_) && in.dt > 0)
      boiler_rate_ = 0.6f * boiler_rate_ + 0.4f * (in.boiler - last_boiler_) / in.dt;
    last_boiler_ = in.boiler;
    const bool boiler_rising = boiler_rate_ > 2.0f / 60;  // > 2 °C/min
    // Blend for the boiler temperature it will soon deliver, not the current one: while it's cold,
    // assume it will reach setpoint + 20 °C (opening fully gains nothing, and lets the first hot water
    // straight through); that assumption fades out smoothly between setpoint and setpoint + 10 °C
    // (no jump at the hand-over), and while it's still rising use where it will be in 2 minutes.
    const float expect = std::max(in.boiler, sp + cfg.boiler_expect_rise);
    const float predicted = std::clamp(in.boiler + std::max(0.0f, boiler_rate_) * 120, in.boiler, expect);
    const float w = std::clamp((sp + 10 - in.boiler) / 10, 0.0f, 1.0f);
    const float boiler_for_ff = std::max(predicted, in.boiler + (expect - in.boiler) * w);
    const bool have_ff = !std::isnan(in.boiler) && !std::isnan(in.ret) && boiler_for_ff - in.ret > 3;
    float ff = have_ff ? (sp - in.ret) / (boiler_for_ff - in.ret) : share_;
    ff = std::clamp(ff, 0.0f, 1.0f);

    const float err = sp - in.mixed;  // positive: too cold, more boiler water
    const float kp = cfg.kp / 100.0f;
    // While waiting for the boiler, only correct downwards (a cold mix is the boiler's fault, but a
    // mix over the setpoint must still be cut back).
    float out = boiler_cold ? ff + kp * std::min(err, 0.0f) : ff + kp * (err + integral_ / cfg.ti);

    // Integrate only when it helps (anti-windup), and not towards hot while the boiler is warming up.
    const bool pinned_hot = out >= 1 && err > 0, pinned_cold = out <= 0 && err < 0;
    if (!boiler_cold && !pinned_hot && !pinned_cold && running_ && !(boiler_rising && err > 0)) {
      integral_ += err * in.dt;
      // The trim may move the share at most +/-30 % from the blend formula.
      const float lim = 0.3f * cfg.ti / std::max(kp, 1e-4f);
      integral_ = std::clamp(integral_, -lim, lim);
      out = ff + kp * (err + integral_ / cfg.ti);
    }
    // While the boiler is warming up, the trim may reduce the share but not raise it past the blend.
    if (boiler_rising) out = std::min(out, ff);
    out = std::clamp(out, 0.0f, 1.0f);
    // Raising the share is rate-limited; cutting it isn't once the mix is more than 2 °C over the
    // setpoint, so a sudden rush of hot water is cut back at once.
    if (running_) {
      const float max_cut = err < -2 ? 1.0f : cfg.max_step;
      out = std::clamp(out, share_ - max_cut, share_ + cfg.max_step);
    }
    share_ = out;
    opening_ = cfg.valve_curve ? valve::opening(share_) : share_;
    running_ = true;

    if (boiler_cold) status_ = Status::WAITING_BOILER;
    else if (err > 1) status_ = Status::WARMING;
    else if (err < -1) status_ = Status::COOLING;
    else status_ = Status::HOLDING;
    return true;
  }

  float opening() const { return opening_; }
  bool over_temp() const { return over_temp_; }
  Status status() const { return status_; }
  // Start from the valve's actual position (e.g. after switching to auto).
  void set_opening(float o, bool curve = true) {
    opening_ = std::clamp(o, 0.0f, 1.0f);
    share_ = curve ? valve::share(opening_) : opening_;
  }
  float share() const { return share_; }
  // Band mode: whether CFH should be on (the caller switches the relay while in auto band mode).
  bool cfh() const { return cfh_; }

 protected:
  // Band mode: heating phase (CFH wanted) or free-wheel; boiler cap holding CFH off.
  bool band_(const Inputs &in, const Settings &cfg) {
    if (!band_started_) {  // start heating unless the mix is already at the top of the band
      heat_ = in.mixed < cfg.band_high;
      cap_off_ = false;
      band_started_ = true;
    }
    if (heat_ && in.mixed >= cfg.band_high) heat_ = false;
    else if (!heat_ && in.mixed <= cfg.band_low) heat_ = true;
    if (!heat_ || !cfg.cap || std::isnan(in.boiler)) cap_off_ = false;
    else if (in.boiler >= cfg.cap_temp) cap_off_ = true;
    else if (in.boiler <= cfg.cap_temp - cfg.cap_restart) cap_off_ = false;
    cfh_ = heat_ && !cap_off_;

    // Valve: keep the mix at or under the top of the band. Heating: for the hottest the boiler may
    // get (assumed max, or cap + overshoot, or its actual temperature if hotter: radiators or hot
    // water may drive it up). Free-wheel: for its actual, falling temperature, so the leftover heat
    // is used. Without boiler/return readings, hold.
    if (!std::isnan(in.boiler) && !std::isnan(in.ret)) {
      float hot = heat_ ? (cfg.cap ? cfg.cap_temp + cfg.cap_overshoot : cfg.boiler_max) : in.boiler;
      hot = std::max({hot, in.boiler, in.ret + 3});
      const float share = std::clamp((cfg.band_high - in.ret) / (hot - in.ret), 0.0f, 1.0f);
      const float open = cfg.valve_curve ? valve::opening(share) : share;
      // Re-set the valve at each phase change, when it would need to close (never let the mix run
      // hot), or when the opening it should have has drifted a lot; otherwise leave it (8 Nov: following
      // every small drift moved the servo ~60 times an hour).
      if (heat_ != band_heat_was_ || open < opening_ - 0.01f || std::fabs(open - opening_) > cfg.band_reset) {
        share_ = share;
        opening_ = open;
      }
    }
    band_heat_was_ = heat_;
    running_ = true;
    status_ = !heat_ ? Status::FREE_WHEEL : cap_off_ ? Status::BOILER_CAP : Status::HEATING;
    return true;
  }

  // Staged mode.
  enum class Stage : uint8_t { IDLE, PULSE, CALL_WAIT, CALL };
  void set_open_(float o) {
    o = std::clamp(o, 0.0f, 1.0f);
    if (std::fabs(o - opening_) > 0.03f) opening_ = o;  // ignore what the positioner would ignore
  }
  bool staged_(const Inputs &in, const Settings &cfg) {
    stage_t_ += in.dt;
    const float rate = std::isnan(last_mixed_) || in.dt <= 0 ? 0 : (in.mixed - last_mixed_) / in.dt;
    last_mixed_ = in.mixed;
    const bool have = !std::isnan(in.boiler) && !std::isnan(in.ret);
    const float aim = cfg.band_high - 1, ceiling = cfg.band_high + 1;
    auto aim_share = [&](float boiler) { return have ? (aim - in.ret) / std::max(boiler - in.ret, 3.0f) : 0.3f; };
    switch (stage_) {
      case Stage::IDLE:  // free-wheel: branch shut, valve left where it is
        cfh_ = false;
        if (in.mixed > cfg.band_low) break;
        stage_ = Stage::PULSE;  // bottom of the band: start a pulse straight away
        stage_t_ = 0;
        [[fallthrough]];
      case Stage::PULSE:
        if (stage_t_ >= cfg.pulse_s + cfg.gap_s) {  // end of a gap: decide what comes next
          stage_t_ = 0;
          if (have && in.ret >= cfg.ret_target) { stage_ = Stage::IDLE; cfh_ = false; break; }
          if (!have || in.boiler < in.ret + cfg.stored_margin) {  // stored heat used up: full call
            stage_ = Stage::CALL_WAIT;
            cfh_ = true;
            set_open_(cfg.wait_open);
            break;
          }
        }
        cfh_ = stage_t_ < cfg.pulse_s;
        if (cfh_) set_open_(valve::opening_on(valve::PULLED, std::clamp(aim_share(in.boiler), 0.0f, 0.65f)));
        break;
      case Stage::CALL_WAIT:  // full call: pull hard until the boiler pump starts
        cfh_ = true;
        set_open_(cfg.wait_open);
        if (rate >= cfg.pump_rate || stage_t_ >= cfg.max_wait_s) {
          stage_ = Stage::CALL;
          call_t_ = stage_t_;
          stage_t_ = 0;
          opening_ = valve::opening_on(valve::PUMPED, std::clamp(aim_share(std::max(in.boiler, 55.0f)), 0.0f, 1.0f));
          last_adjust_ = 0;
        }
        break;
      case Stage::CALL:  // boiler pumping: coarse ceiling, at most one step every 30 s
        cfh_ = true;
        if (in.mixed > ceiling + 2) {
          opening_ = std::max(0.0f, opening_ - 0.03f);  // well over: step every update
          last_adjust_ = stage_t_;
        } else if (stage_t_ - last_adjust_ >= 30) {
          if (in.mixed > ceiling) { opening_ = std::max(0.0f, opening_ - 0.03f); last_adjust_ = stage_t_; }
          else if (in.mixed < aim - 3) { opening_ = std::min(0.5f, opening_ + 0.02f); last_adjust_ = stage_t_; }
        }
        if (have && in.ret >= cfg.ret_target && call_t_ + stage_t_ >= cfg.min_call_s) {
          stage_ = Stage::IDLE;
          stage_t_ = 0;
          cfh_ = false;
        }
        break;
    }
    share_ = opening_;  // (reporting only)
    running_ = true;
    status_ = stage_ == Stage::IDLE ? Status::FREE_WHEEL
            : stage_ == Stage::PULSE ? Status::PULSING
            : stage_ == Stage::CALL_WAIT ? Status::CALL_WAIT : Status::CALLING;
    return true;
  }

  Stage stage_ = Stage::IDLE;
  float stage_t_ = 0, call_t_ = 0, last_adjust_ = 0, last_mixed_ = NAN;

  bool cfh_ = false, heat_ = false, cap_off_ = false, band_started_ = false, band_heat_was_ = false;
  float opening_ = 0;
  float share_ = 0;     // boiler-water share asked for
  float integral_ = 0;  // °C·s
  float missing_s_ = 0;
  float last_boiler_ = NAN, boiler_rate_ = 0;  // °C/s, smoothed
  bool over_temp_ = false;
  bool running_ = false;
  Status status_ = Status::OFF;
};

// ----------------------------------------------------------------------------------------------
// Servo shaft position for the SC15 in continuous (motor) mode, driving the mixer through the old
// actuator's gearbox (stroke ~430° of servo rotation, bench 7 Nov 2026).
//
// The SC15's potentiometer reads 0..1023 over ~210° and is blind for the other ~150° of each turn:
// near the edges the reading sticks (~0 or ~1019), and inside the gap it can be any value (a stalled
// reading of 811 was seen there). Towards hot the reading counts down. Axis x, counts, hot positive:
// in window k, x = k*TURN + (1023 - raw); the gap below window k spans x = k*TURN - GAP .. k*TURN.
constexpr int TURN = 1755;              // counts per turn at 1024 counts per 210°
constexpr int WIN = 1023, GAP = TURN - WIN;

class Tracker {
 public:
  // Position now known to be x (e.g. at the cold end after homing), in the gap or in a window.
  void set(float x) {
    x_ = x;
    known_ = true;
    provisional_ = false;
    consistent_ = 0;
    first_ = -1;
    still_ = 0;
    pending_ = -1;
    int k = int(std::floor(x / TURN));
    float in_turn = x - k * TURN;  // 0..TURN: window 0..WIN, then gap to the next window
    if (in_turn <= WIN) {
      in_gap_ = false; k_ = k; last_raw_ = WIN - int(in_turn);
    } else {
      in_gap_ = true; gap_k_ = k + 1; gap_travel_ = 0;
    }
  }
  void forget() { known_ = false; }
  // Position unknown but inside the gap below window `gap_k` (e.g. at a cold end found by homing).
  void set_in_gap(int gap_k) {
    known_ = true; in_gap_ = true; provisional_ = true; gap_k_ = gap_k; x_ = gap_k * TURN - GAP / 2.0f;
    gap_travel_ = 0; pending_ = first_ = -1; consistent_ = 0; still_ = 0;
  }

  // Feed every reading. dir: drive direction (+1 hot, -1 cold, 0 idle); dt: s since the last one;
  // driving: false while coasting after a stop (the gap estimate then holds; see coast()).
  void sample(int raw, int dir, float dt, bool driving = true) {
    if (!known_) return;
    const bool inside = raw >= 6 && raw <= 1016;
    if (!in_gap_) {
      if (probation_ > 0 && !(inside && std::abs(raw - last_raw_) <= 150)) {
        // The readings didn't carry on from the gap exit: it was a false one. Back into the gap.
        in_gap_ = true;
        x_ = saved_x_;
        gap_k_ = saved_gap_k_;
        gap_travel_ = saved_travel_;
        provisional_ = saved_provisional_;
        exits_--;
        probation_ = 0;
        pending_ = first_ = -1;
        consistent_ = 0;
        return;
      }
      if (inside && std::abs(raw - last_raw_) <= 150) {
        if (probation_ > 0) probation_--;
        const int d = raw - last_raw_;
        still_ = std::abs(d) <= 2 ? still_ + 1 : 0;
        if (dt > 0 && std::abs(d) > 2) speed_ = 0.7f * speed_ + 0.3f * (std::abs(d) / dt);
        last_raw_ = raw;
        x_ = k_ * TURN + (WIN - raw);
        return;
      }
      // Left the window: into the gap below it (reading heading for 0) or above it (heading for 1023).
      in_gap_ = true;
      gap_k_ = last_raw_ < 512 ? k_ + 1 : k_;
      gap_travel_ = 0;
      pending_ = -1;
      still_ = 0;
    }
    // In the gap. Leave it only on readings moving the right way: moving hot the reading falls,
    // moving cold it rises, by 5-150 counts per poll. Two in a row are enough when the first is at
    // the edge where the window starts (just under 1023 going hot, just over 0 going cold); otherwise
    // three. Readings stuck in the gap, or random ones, don't pass.
    const bool inside_now = raw >= 6 && raw <= 1016;
    const bool inferred = dir == 0;
    // Not driving (moved by something else, or coasting after a run): take the direction from the
    // readings themselves.
    if (dir == 0 && inside_now && pending_ >= 0) {
      if (pending_ - raw >= 5 && pending_ - raw <= 150) dir = 1;
      else if (raw - pending_ >= 5 && raw - pending_ <= 150) dir = -1;
    }
    int step = 0;
    if (inside_now && dir != 0 && pending_ >= 0) step = dir > 0 ? pending_ - raw : raw - pending_;
    if (step >= 5 && step <= 150) {
      consistent_++;
    } else {
      consistent_ = 0;
      first_ = inside_now ? raw : -1;  // a new run of readings may start here
    }
    pending_ = inside_now ? raw : -1;
    if (dir == 0) return;  // nothing to go on yet
    const bool from_edge = dir > 0 ? first_ >= 940 : (first_ >= 0 && first_ <= 80);
    // Moving on its own: there's no estimate to check against, so only accept a clear exit from the
    // expected edge (3 readings in a row).
    const int need = provisional_ ? (from_edge ? 2 : 4) : (from_edge ? 2 : 3);
    if (inferred && !from_edge) return;
    const int new_k = dir > 0 ? gap_k_ : gap_k_ - 1;
    const float exact = new_k * TURN + (WIN - raw);
    // Once the position is trusted, the estimate must also have got at least a quarter of the way
    // across the gap towards this side (it lags when friction changes, so no tighter than that).
    const float lo = gap_k_ * TURN - GAP, hi = gap_k_ * TURN;
    const bool plausible =
        provisional_ || inferred || (dir > 0 ? x_ >= lo + 0.25f * GAP : x_ <= hi - 0.25f * GAP);
    if (consistent_ >= need && plausible) {
      saved_x_ = x_; saved_gap_k_ = gap_k_; saved_travel_ = gap_travel_; saved_provisional_ = provisional_;
      probation_ = 2;
      in_gap_ = false;
      provisional_ = false;
      k_ = new_k;
      last_raw_ = raw;
      exit_error_ = x_ - exact;
      last_gap_travel_ = gap_travel_;
      x_ = exact;
      exits_++;
      consistent_ = 0;
      return;
    }
    // Estimate from the last measured speed (at least a typical one), kept inside the gap.
    if (!driving) return;
    const float v = std::max(speed_, 300.0f);
    x_ += dir * v * dt;
    gap_travel_ += dir * v * dt;
    x_ = std::clamp(x_, float(gap_k_ * TURN - GAP), float(gap_k_ * TURN));
  }

  bool known() const { return known_; }
  bool in_gap() const { return in_gap_; }
  float x() const { return x_; }
  // Driving but the reading hasn't moved for `polls` readings (only detectable in a window).
  bool stalled(int polls = 3) const { return !in_gap_ && still_ >= polls; }
  int gap_exits() const { return exits_; }
  float last_exit_error() const { return exit_error_; }  // estimate minus truth at the last gap exit
  float last_gap_travel() const { return last_gap_travel_; }  // estimated drive inside that gap
  int last_raw() const { return last_raw_; }
  void set_speed(float counts_per_s) { speed_ = counts_per_s; }
  void reset_still() { still_ = 0; }
  // After a stop: if still in the gap, allow for the coast once.
  void coast(int dir, float dist) {
    if (!in_gap_) return;
    x_ = std::clamp(x_ + dir * dist, float(gap_k_ * TURN - GAP), float(gap_k_ * TURN));
    gap_travel_ += dir * dist;
  }

 protected:
  float x_ = 0, speed_ = 350;  // counts/s, last measured (bench: 150-500 at 30 %)
  float gap_travel_ = 0, last_gap_travel_ = 0, exit_error_ = 0;
  float saved_x_ = 0, saved_travel_ = 0;  // state before the last gap exit, while it's on probation
  int saved_gap_k_ = 0, probation_ = 0;
  bool saved_provisional_ = false;
  int k_ = 0, gap_k_ = 0, last_raw_ = 0, pending_ = -1, first_ = -1, consistent_ = 0, still_ = 0, exits_ = 0;
  bool in_gap_ = true, known_ = false, provisional_ = false;
};

// The gearbox cam's four switches (DI5-DI8; bit 0 = DI5) form a code that changes one switch at a
// time along the stroke, so they give an absolute zone at any moment, and an exact reference at every
// change. Measured 7 Nov 2026 over three sweeps (repeatable to ~10-25 counts; changes going cold come
// 35-110 counts before the same change going hot). DI5 off = cold end switch, DI8 on (after DI7) =
// hot end switch, as in the original mains actuator; DI6 and DI7 were its auxiliaries.
namespace cam {
constexpr uint8_t COLD_END = 0b1010, HOT_END = 0b1101;  // DI8 DI7 DI6 DI5
struct Zone {
  uint8_t state;
  float lo, hi;  // x range in which this state can be seen (either direction)
};
constexpr Zone ZONES[] = {
    {0b1010, -1e9f, 65},  {0b1011, 0, 645},     {0b0011, 535, 1025},
    {0b0111, 945, 1795},  {0b0101, 1757, 1956}, {0b1101, 1885, 1e9f},
};
struct Edge {
  uint8_t from, to;
  float x;  // shaft position where it changes
};
constexpr Edge EDGES[] = {
    // going hot
    {0b1010, 0b1011, 65}, {0b1011, 0b0011, 645}, {0b0011, 0b0111, 1025}, {0b0111, 0b0101, 1795},
    {0b0101, 0b1101, 1956},
    // going cold
    {0b1101, 0b0101, 1885}, {0b0101, 0b0111, 1757}, {0b0111, 0b0011, 945}, {0b0011, 0b1011, 535},
    {0b1011, 0b1010, 0},
};
inline const Zone *zone(uint8_t state) {
  for (const auto &z : ZONES)
    if (z.state == state) return &z;
  return nullptr;
}
inline const Edge *edge(uint8_t from, uint8_t to) {
  for (const auto &e : EDGES)
    if (e.from == from && e.to == to) return &e;
  return nullptr;
}
}  // namespace cam

// Moves the valve to a target opening using the tracked shaft position.
//   - Opening = (x - cold_x) / stroke_x, with the lever play allowed for: after moving hot the valve
//     trails the shaft by play/2, after moving cold it leads by play/2.
//   - Homing: drive cold for home_s at drive power (pushing gently on the cold stop), then x = cold_x.
//     If the cold end is inside a window, a stall ends it sooner. Done when auto turns on, at pump
//     start and whenever the position is lost.
//   - A move runs towards the target and is stopped (by the caller's monitor) coast_x short of it.
//   - Calibration: home, then drive hot to the hot end (stall in a window), measuring cold_x (from
//     where the reading first leaves the gap, if the cold end is in one) and stroke_x.
struct PosConfig {
  float cold_x = -300;    // x at the cold end
  float stroke_x = 0;     // x(hot end) - x(cold end); 0 = not calibrated
  float play_x = 0;       // lever play, counts
  float coast_x = 50;     // the shaft coasts on this far after the drive stops
  float deadband = 0.03f; // of the stroke
  float home_s = 12;      // cold homing run time (s)
  float max_run_s = 6;    // longest single move
  float kick_s = 0.2f;    // stiction-breaking kick at the start of each move (the caller drives it)
  int settle_ticks = 30;  // calls to next() after a move during which reversing needs twice the deadband
};

enum class PosMode : uint8_t { IDLE, HOMING, CAL_HOME, CAL_MEASURE, CAL_RETURN, READY };

class Positioner {
 public:
  struct Move {
    int dir = 0;          // +1 hot, -1 cold
    float target_x = 0;   // stop here (minus coast); ignored for to_end moves
    float max_s = 0;
    bool to_end = false;  // run until an end (stall) or max_s
  };

  Tracker &tracker() { return tr_; }

  // Feed the cam switch state with every servo reading. Returns a short note when it corrected the
  // position (for logging), or nullptr.
  const char *cam(uint8_t state) {
    use_cam_ = true;
    // All four off never happens on the cam: it's the switches' common or supply dropping out.
    if (state == 0) return nullptr;
    const char *note = nullptr;
    // A change counts only once it has held for two readings (wiring blips last one reading or so);
    // the shaft's movement since the first sighting is kept when snapping to the edge.
    if (cam_ != 0xFF && state != cam_) {
      if (state != pending_) {
        pending_ = state;
        pending_x_ = tr_.x();
        return nullptr;
      }
    }
    pending_ = 0xFF;
    if (cam_ != 0xFF && state != cam_) {
      if (const cam::Edge *e = cam::edge(cam_, state)) {
        const float moved = tr_.x() - pending_x_;
        const float err = tr_.known() ? pending_x_ - e->x : 0;
        tr_.set(e->x + moved);
        if (std::fabs(err) > 60) note = "cam edge";
        last_cam_err_ = err;
      } else {
        note = "cam code jumped";  // two switches at once: bounce or a fault; zone check still applies
      }
    }
    cam_ = state;
    // Zone check: never further than a margin outside the zone the switches show.
    if (const cam::Zone *z = cam::zone(state)) {
      const float margin = 60, x = tr_.x();
      if (!tr_.known()) {
        // Unknown (e.g. after a restart): start mid-zone, or at its finite edge.
        float mid = (z->lo < -1e8f) ? z->hi - 100 : (z->hi > 1e8f) ? z->lo + 50 : (z->lo + z->hi) / 2;
        tr_.set(mid);
        cam_coarse_ = true;
      } else if (x < z->lo - margin || x > z->hi + margin) {
        last_cam_err_ = x < z->lo ? x - z->lo : x - z->hi;
        tr_.set(std::clamp(x, z->lo, z->hi));
        note = "cam zone";
      }
    } else if (state != cam_bad_) {
      note = "cam code unknown";
      cam_bad_ = state;
    }
    return note;
  }
  bool cam_end(int dir) const { return dir > 0 ? cam_ == cam::HOT_END : cam_ == cam::COLD_END; }
  float last_cam_error() const { return last_cam_err_; }
  void request_home() {
    if (mode_ != PosMode::CAL_HOME && mode_ != PosMode::CAL_MEASURE && mode_ != PosMode::CAL_RETURN)
      mode_ = PosMode::HOMING;
  }
  void request_calibration() { mode_ = PosMode::CAL_HOME; cal_failed_ = false; first_exit_seen_ = false; }
  void lost() { tr_.forget(); if (mode_ == PosMode::READY) mode_ = PosMode::IDLE; }

  // Shaft positions at the ends: after driving into an end the valve trails the shaft by half the play.
  static float cold_shaft_x(const PosConfig &c) { return c.cold_x - c.play_x / 2; }
  static float hot_shaft_x(const PosConfig &c) { return c.cold_x + c.stroke_x + c.play_x / 2; }

  float opening(const PosConfig &c) const {
    if (c.stroke_x <= 0) return NAN;
    float valve_x = tr_.x() - last_dir_ * c.play_x / 2;
    return std::clamp((valve_x - c.cold_x) / c.stroke_x, 0.0f, 1.0f);
  }
  bool known() const { return mode_ == PosMode::READY && tr_.known(); }

  // Next move towards `target` opening, or false. Call only while the servo is idle.
  bool next(float target, bool move_valve, const PosConfig &c, Move *m) {
    if (idle_ticks_ < 1000000) idle_ticks_++;
    if (!next_(target, move_valve, c, m)) return false;
    idle_ticks_ = 0;
    polls_ = 0;
    stop_stall_ = false;
    tr_.reset_still();
    return true;
  }

  // Polled by the servo driver during a move: true = stop now. Stalls count only after the kick and
  // spin-up (4 polls, 0.4 s).
  bool should_stop(const Move &m, const PosConfig &c) {
    polls_++;
    // With the cam switches, the ends are the end switches (as for the original actuator).
    if (use_cam_ && cam_end(m.dir)) {
      stop_stall_ = true;
      return true;
    }
    if (polls_ >= 4 && tr_.stalled()) {  // at an end (or jammed)
      stop_stall_ = true;
      return true;
    }
    if (m.to_end || !tr_.known()) return false;
    return m.dir > 0 ? tr_.x() >= m.target_x - c.coast_x : tr_.x() <= m.target_x + c.coast_x;
  }
  // Whether the last move was stopped by a stall (decided when it stopped, not after the coast: the
  // shaft springs back a little once the drive is off).
  bool stopped_on_stall() const { return stop_stall_; }

 protected:
  bool next_(float target, bool move_valve, const PosConfig &c, Move *m) {
    switch (mode_) {
      case PosMode::CAL_HOME:
      case PosMode::CAL_RETURN: *m = {-1, 0, c.home_s, true}; return true;
      case PosMode::CAL_MEASURE: *m = {+1, 0, 60, true}; return true;
      case PosMode::HOMING:
        if (c.stroke_x <= 0) return false;
        *m = {-1, 0, c.home_s, true};
        return true;
      case PosMode::IDLE: return false;
      case PosMode::READY: break;
    }
    if (!move_valve || c.stroke_x <= 0) return false;
    target = std::clamp(target, 0.0f, 1.0f);
    const float now = opening(c);
    if (std::fabs(target - now) < c.deadband) return false;
    // Targets at the ends: drive into the end (re-syncs at the hot end, pushes the cold end shut).
    const int dir = target > now ? 1 : -1;
    // Don't reverse straight back after a move unless clearly needed: the smallest move (kick plus
    // coast) can be bigger than the deadband, which otherwise hunts between two positions.
    // Only reversals towards hot are held back: closing (towards cold) is never delayed.
    if (dir > 0 && last_dir_ < 0 && idle_ticks_ < c.settle_ticks && std::fabs(target - now) < 2 * c.deadband)
      return false;
    if (target <= 0.001f) { *m = {-1, 0, c.home_s, true}; return true; }
    if (target >= 0.999f) { *m = {+1, 0, c.home_s, true}; return true; }
    // Shaft target: allow for the play when this move's direction differs from the last one.
    float tx = c.cold_x + target * c.stroke_x + dir * c.play_x / 2;
    *m = {dir, tx, c.max_run_s, false};
    return true;
  }


 public:
  // After a move (including the coast): stalled = it stopped on a stall. `raw` = latest reading.
  void done(const Move &m, bool stalled, int raw, PosConfig *c) {
    last_dir_ = m.dir;
    if (!stalled) tr_.coast(m.dir, c->coast_x);
    switch (mode_) {
      case PosMode::CAL_HOME:
        // At the cold end. Frame: window 0 is the first window met going hot. If the reading is
        // valid here the cold end is in that window; otherwise it's in the gap below it, and its
        // place is worked out when the reading first leaves the gap.
        // A single reading here can't be trusted (the gap gives random or stuck values), so start
        // "in the gap below window 0" and let the first consistent movement place it.
        (void) raw;
        tr_.set_in_gap(0);
        cold_in_gap_ = true;
        exits_at_home_ = tr_.gap_exits();
        x_at_home_ = tr_.x();
        mode_ = PosMode::CAL_MEASURE;
        return;
      case PosMode::CAL_MEASURE: {
        if (!stalled || tr_.in_gap()) { fail_cal_(); return; }
        float cold_x = x_at_home_;
        if (cold_in_gap_) {
          if (!first_exit_seen_) { fail_cal_(); return; }
          cold_x = first_exit_x_ - first_exit_travel_;
        }
        c->cold_x = cold_x;
        c->stroke_x = tr_.x() - cold_x;
        calibrated_ = true;
        mode_ = PosMode::CAL_RETURN;
        return;
      }
      case PosMode::CAL_RETURN:
      case PosMode::HOMING:
        // With the cam, the end switch already set the position (and the coast is tracked).
        if (!use_cam_ || !tr_.known()) tr_.set(cold_shaft_x(*c));
        last_dir_ = -1;
        mode_ = PosMode::READY;
        return;
      default: break;
    }
    // A run to an end (timed, or stopped on a stall) is now at that end: the sensor may not be able to
    // see it (cold end in the gap; short pushes at the hot end never leave the gap), so reset there.
    if (m.to_end && c->stroke_x > 0 && !use_cam_) tr_.set(m.dir > 0 ? hot_shaft_x(*c) : cold_shaft_x(*c));
  }
  // Call after every sample during calibration: notes where the reading first leaves the gap.
  void watch_calibration() {
    if (mode_ == PosMode::CAL_MEASURE && cold_in_gap_ && !first_exit_seen_ && tr_.gap_exits() > exits_at_home_) {
      first_exit_seen_ = true;
      first_exit_x_ = tr_.x();
      first_exit_travel_ = tr_.last_gap_travel();
    }
  }

  PosMode mode() const { return mode_; }
  bool cal_failed() const { return cal_failed_; }
  bool take_calibrated() { bool c = calibrated_; calibrated_ = false; return c; }

 protected:
  void fail_cal_() { cal_failed_ = true; tr_.forget(); mode_ = PosMode::IDLE; }

  Tracker tr_;
  int last_dir_ = -1, exits_at_home_ = 0, polls_ = 0, idle_ticks_ = 1000000;
  uint8_t cam_ = 0xFF, cam_bad_ = 0xFF, pending_ = 0xFF;
  float pending_x_ = 0;
  bool use_cam_ = false, cam_coarse_ = false;
  float last_cam_err_ = 0;
  bool stop_stall_ = false;
  float x_at_home_ = 0, first_exit_x_ = 0, first_exit_travel_ = 0;
  bool cal_failed_ = false, calibrated_ = false, cold_in_gap_ = false, first_exit_seen_ = false;
  PosMode mode_ = PosMode::IDLE;
};

// The one controller and positioner on the device (ESPHome globals are declared before includes, so
// they can't be globals).
inline Controller &instance() {
  static Controller c;
  return c;
}
inline Positioner &positioner() {
  static Positioner p;
  return p;
}
inline PosConfig &pos_config() {
  static PosConfig c;
  return c;
}
// The positioner's move in progress (the servo driver also runs plain jogs, which have none).
inline Positioner::Move &current_move() {
  static Positioner::Move m;
  return m;
}
inline bool &move_active() {
  static bool a = false;
  return a;
}
// Manual target opening (0..1) used while auto is off; NAN = none (the valve stays put).
inline float &manual_target() {
  static float t = NAN;
  return t;
}

}  // namespace mixer_control
