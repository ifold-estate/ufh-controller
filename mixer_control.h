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

struct Settings {
  float setpoint = 35;    // °C mixed flow
  float limit = 45;       // °C over-temperature cut-out
  float kp = 1.0;         // % opening per °C error
  float ti = 120;         // s integral time
  float max_step = 0.05;  // max opening change per update (fraction)
};

struct Inputs {
  float boiler = NAN, mixed = NAN, ret = NAN;  // °C, NAN when unavailable
  bool pump_on = false;                        // floor pump running
  bool automatic = false;                      // "Mixer auto" switch
  float dt = 5;                                // s since the last update
};

enum class Status : uint8_t { OFF, PUMP_OFF, PROBE_FAULT, OVER_TEMP, WAITING_BOILER, WARMING, HOLDING, COOLING };

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
    if (!in.automatic) {
      integral_ = 0;
      running_ = false;
      status_ = over_temp_ ? Status::OVER_TEMP : Status::OFF;
      return false;
    }
    if (over_temp_) {
      integral_ = 0;
      running_ = false;
      opening_ = 0;
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
      status_ = Status::PUMP_OFF;
      return false;  // hold position, ready for the next start
    }

    const float sp = cfg.setpoint;
    const bool have_ff = !std::isnan(in.boiler) && !std::isnan(in.ret) && in.boiler - in.ret > 3;
    float ff = have_ff ? (sp - in.ret) / (in.boiler - in.ret) : opening_;
    ff = std::clamp(ff, 0.0f, 1.0f);
    const bool boiler_cold = !std::isnan(in.boiler) && in.boiler < sp + 2;

    const float err = sp - in.mixed;  // positive: too cold, open towards hot
    const float kp = cfg.kp / 100.0f;
    float out = ff + kp * (err + integral_ / cfg.ti);

    // Integrate only when it helps (anti-windup).
    const bool pinned_hot = out >= 1 && err > 0, pinned_cold = out <= 0 && err < 0;
    if (!boiler_cold && !pinned_hot && !pinned_cold && running_) {
      integral_ += err * in.dt;
      // The trim may move the valve at most +/-30 % from the blend formula.
      const float lim = 0.3f * cfg.ti / std::max(kp, 1e-4f);
      integral_ = std::clamp(integral_, -lim, lim);
      out = ff + kp * (err + integral_ / cfg.ti);
    }
    out = std::clamp(out, 0.0f, 1.0f);
    if (running_) out = std::clamp(out, opening_ - cfg.max_step, opening_ + cfg.max_step);
    opening_ = out;
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
  void set_opening(float o) { opening_ = std::clamp(o, 0.0f, 1.0f); }

 protected:
  float opening_ = 0;
  float integral_ = 0;  // °C·s
  float missing_s_ = 0;
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
    int step = 0;
    if (inside_now && dir != 0 && pending_ >= 0) step = dir > 0 ? pending_ - raw : raw - pending_;
    if (step >= 5 && step <= 150) {
      consistent_++;
    } else {
      consistent_ = 0;
      first_ = (inside_now && dir != 0) ? raw : -1;  // a new run of readings may start here
    }
    pending_ = inside_now ? raw : -1;
    const bool from_edge = dir > 0 ? first_ >= 940 : (first_ >= 0 && first_ <= 80);
    const int need = provisional_ ? (from_edge ? 2 : 4) : (from_edge ? 2 : 3);
    const int new_k = dir > 0 ? gap_k_ : gap_k_ - 1;
    const float exact = new_k * TURN + (WIN - raw);
    // Once the position is trusted, the estimate must also have got at least a quarter of the way
    // across the gap towards this side (it lags when friction changes, so no tighter than that).
    const float lo = gap_k_ * TURN - GAP, hi = gap_k_ * TURN;
    const bool plausible = provisional_ || (dir > 0 ? x_ >= lo + 0.25f * GAP : x_ <= hi - 0.25f * GAP);
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
  void request_home() {
    if (mode_ != PosMode::CAL_HOME && mode_ != PosMode::CAL_MEASURE && mode_ != PosMode::CAL_RETURN)
      mode_ = PosMode::HOMING;
  }
  void request_calibration() { mode_ = PosMode::CAL_HOME; cal_failed_ = false; first_exit_seen_ = false; }
  void lost() { tr_.forget(); if (mode_ == PosMode::READY) mode_ = PosMode::IDLE; }

  float opening(const PosConfig &c) const {
    if (c.stroke_x <= 0) return NAN;
    float valve_x = tr_.x() - last_dir_ * c.play_x / 2;
    return std::clamp((valve_x - c.cold_x) / c.stroke_x, 0.0f, 1.0f);
  }
  bool known() const { return mode_ == PosMode::READY && tr_.known(); }

  // Next move towards `target` opening, or false. Call only while the servo is idle.
  bool next(float target, bool move_valve, const PosConfig &c, Move *m) {
    if (!next_(target, move_valve, c, m)) return false;
    polls_ = 0;
    tr_.reset_still();
    return true;
  }

  // Polled by the servo driver during a move: true = stop now. Stalls count only after the kick and
  // spin-up (4 polls, 0.4 s).
  bool should_stop(const Move &m, const PosConfig &c) {
    polls_++;
    if (polls_ >= 4 && tr_.stalled()) return true;  // at an end (or jammed)
    if (m.to_end || !tr_.known()) return false;
    return m.dir > 0 ? tr_.x() >= m.target_x - c.coast_x : tr_.x() <= m.target_x + c.coast_x;
  }
  bool stalled_now() const { return polls_ >= 4 && tr_.stalled(); }

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
    if (target <= 0.001f) { *m = {-1, 0, c.max_run_s, true}; return true; }
    if (target >= 0.999f) { *m = {+1, 0, c.max_run_s, true}; return true; }
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
        tr_.set(c->cold_x);
        last_dir_ = -1;
        mode_ = PosMode::READY;
        return;
      default: break;
    }
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
  int last_dir_ = -1, exits_at_home_ = 0, polls_ = 0;
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

}  // namespace mixer_control
