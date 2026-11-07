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

// Valve position for a servo in continuous (motor) mode, which can't report an absolute position
// through the ~3.5:1 gearbox. Position is estimated from run time:
//   - Homing: drive to the cold end (stall or limit switch), position 0. Done when auto turns on and
//     at every pump start, so timing errors never build up.
//   - Calibration: cold end -> hot end, timing the stroke; then back to cold.
//   - Normal moves run for |target - position| x stroke time. Reversing first takes up the lever's
//     play (backlash time), which doesn't count as movement.
//   - Reaching an end (stall/limit) snaps the estimate to 0 or 1; a target at an end drives a little
//     past it once to re-sync.
struct PosConfig {
  float stroke_s = 0;    // full cold->hot run time; 0 = not calibrated
  float backlash_s = 0;  // run time to take up the play when reversing
  float deadband = 0.03f;
  float max_run_s = 5;   // longest single move, so control stays responsive
};

enum class PosMode : uint8_t { IDLE, HOMING, CAL_HOME, CAL_MEASURE, CAL_RETURN, READY };

class Positioner {
 public:
  struct Move {
    int dir = 0;         // +1 towards hot, -1 towards cold
    float seconds = 0;   // run time
    bool to_end = false; // expected to finish at an end (stall/limit)
  };

  void request_home() {
    if (mode_ != PosMode::CAL_HOME && mode_ != PosMode::CAL_MEASURE && mode_ != PosMode::CAL_RETURN)
      mode_ = PosMode::HOMING;
  }
  void request_calibration() { mode_ = PosMode::CAL_HOME; cal_failed_ = false; }

  // Next move towards `target` (0..1), or false if none is needed. Call only while the servo is idle.
  // `move_valve` false: only homing/calibration moves, no control moves.
  bool next(float target, bool move_valve, const PosConfig &c, Move *m) {
    const float cal_s = 120;  // generous limit for an end-to-end run while calibrating
    switch (mode_) {
      case PosMode::CAL_HOME: *m = {-1, cal_s, true}; return true;
      case PosMode::CAL_MEASURE: *m = {+1, cal_s, true}; return true;
      case PosMode::CAL_RETURN: *m = {-1, cal_s, true}; return true;
      case PosMode::HOMING:
        if (c.stroke_s <= 0) return false;
        *m = {-1, c.stroke_s * 1.3f + c.backlash_s, true};
        return true;
      case PosMode::IDLE: return false;
      case PosMode::READY: break;
    }
    if (!move_valve || c.stroke_s <= 0) return false;
    target = std::clamp(target, 0.0f, 1.0f);
    if (target > 0.05f) cold_resynced_ = false;
    if (target < 0.95f) hot_resynced_ = false;
    float extra = (last_dir_ != 0) ? c.backlash_s : 0;
    if (target <= 0.001f && pos_ <= 0.001f && !cold_resynced_) {
      cold_resynced_ = true;
      *m = {-1, 0.15f * c.stroke_s + (last_dir_ > 0 ? extra : 0), true};
      return true;
    }
    if (target >= 0.999f && pos_ >= 0.999f && !hot_resynced_) {
      hot_resynced_ = true;
      *m = {+1, 0.15f * c.stroke_s + (last_dir_ < 0 ? extra : 0), true};
      return true;
    }
    const float e = target - pos_;
    if (std::fabs(e) < c.deadband) return false;
    const int dir = e > 0 ? 1 : -1;
    float secs = std::min(std::fabs(e) * c.stroke_s, c.max_run_s);
    if (last_dir_ != 0 && dir != last_dir_) secs += c.backlash_s;
    *m = {dir, secs, false};
    return true;
  }

  // Report how a move went: how long it ran and whether it ended at an end (stall or limit switch).
  void done(const Move &m, float ran_s, bool at_end, const PosConfig &c) {
    const bool reversed = last_dir_ != 0 && m.dir != last_dir_;
    last_dir_ = m.dir;
    switch (mode_) {
      case PosMode::CAL_HOME:
        if (at_end) { mode_ = PosMode::CAL_MEASURE; pos_ = 0; } else { fail_cal_(); }
        return;
      case PosMode::CAL_MEASURE:
        if (at_end) { measured_stroke_ = std::max(0.0f, ran_s - c.backlash_s); pos_ = 1; mode_ = PosMode::CAL_RETURN; }
        else { fail_cal_(); }
        return;
      case PosMode::CAL_RETURN:
      case PosMode::HOMING:
        pos_ = 0;  // ran long enough towards cold to be there even without an end signal
        homed_ = true;
        cold_resynced_ = true;
        mode_ = PosMode::READY;
        return;
      default: break;
    }
    if (at_end) {
      pos_ = m.dir > 0 ? 1 : 0;
      return;
    }
    if (c.stroke_s <= 0) return;
    const float moved = std::max(0.0f, ran_s - (reversed ? c.backlash_s : 0)) / c.stroke_s;
    pos_ = std::clamp(pos_ + m.dir * moved, 0.0f, 1.0f);
  }

  // Abandon any move in progress (e.g. servo went offline): position no longer trusted.
  void lost() { homed_ = false; if (mode_ == PosMode::READY) mode_ = PosMode::IDLE; }

  bool known() const { return homed_ && mode_ == PosMode::READY; }
  float position() const { return pos_; }
  PosMode mode() const { return mode_; }
  bool cal_failed() const { return cal_failed_; }
  // A newly measured stroke time (once), for the caller to store.
  float take_measured_stroke() { float s = measured_stroke_; measured_stroke_ = 0; return s; }

 protected:
  void fail_cal_() { cal_failed_ = true; homed_ = false; mode_ = PosMode::IDLE; }

  float pos_ = 0;
  int last_dir_ = 0;
  bool homed_ = false, cal_failed_ = false, cold_resynced_ = false, hot_resynced_ = false;
  float measured_stroke_ = 0;
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

}  // namespace mixer_control
