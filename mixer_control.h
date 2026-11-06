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

}  // namespace mixer_control
