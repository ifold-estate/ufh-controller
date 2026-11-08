// Thermostat-driven demand: which zone valves are open, and when the floor pump runs.
//
// Map (Jon, 8 Nov 2026): DI1 playroom T4 -> relay 7 (zones 7, 8); DI2 kitchen N1 -> relay 8 (2, 3);
// DI3 booth N2 -> relay 4 (1, 6); DI4 lounge N3 -> relay 5 (9, 10); relay 6 (kitchen centre, 4, 5)
// per "Relay 6 mode".
//   - A zone valve opens while its thermostat calls. Openings are staggered (several cold thermal
//     actuators at once blew the 24 Vac fuse on 6 Nov).
//   - The pump runs once a valve has been open long enough for its actuator to open, never against
//     closed valves.
//   - When the last call ends the pump runs on (overrun) with the last valves still open, then stops;
//     the valves close shortly after.
//   - The caller lets CFH on only while active() (some thermostat calling).
// Plain C++ so tools/mixer_sim can test it.
#pragma once

#include <cstdint>

namespace heating_demand {

// Zone outputs, in this order.
enum Zone : uint8_t { R4, R5, R6, R7, R8, NZONES };
enum class R6Mode : uint8_t { OFF, N1, N1_OR_N2, ON };

struct Settings {
  float stagger_s = 30;     // between valve openings
  float actuator_s = 120;   // thermal actuator opening time before the pump may run
  float overrun_s = 180;    // pump run-on after the last call
  float close_delay_s = 10; // valves close this long after the pump stops
  R6Mode r6 = R6Mode::OFF;
};

class Demand {
 public:
  // calls[0..3] = DI1..DI4 (T4 playroom, N1 kitchen, N2 booth, N3 lounge). dt in seconds.
  void update(const bool calls[4], float dt, const Settings &c) {
    bool want[NZONES] = {};
    want[R7] = calls[0];
    want[R8] = calls[1];
    want[R4] = calls[2];
    want[R5] = calls[3];
    switch (c.r6) {
      case R6Mode::OFF: break;
      case R6Mode::N1: want[R6] = calls[1]; break;
      case R6Mode::N1_OR_N2: want[R6] = calls[1] || calls[2]; break;
      case R6Mode::ON: want[R6] = calls[0] || calls[1] || calls[2] || calls[3]; break;
    }
    active_ = calls[0] || calls[1] || calls[2] || calls[3];

    since_open_ += dt;
    bool any_open = false;
    for (int z = 0; z < NZONES; z++) {
      if (open_[z]) open_for_[z] += dt;
      if (want[z] && !open_[z] && since_open_ >= c.stagger_s) {  // one opening per stagger interval
        open_[z] = true;
        open_for_[z] = 0;
        since_open_ = 0;
      }
    }

    if (active_) {
      overrun_ = 0;
      ending_ = false;
      // Close valves whose call ended, as long as another valve stays open for the pump.
      for (int z = 0; z < NZONES; z++)
        if (open_[z] && !want[z] && others_open_(z)) open_[z] = false;
    } else if (any_open_()) {
      // Last call ended: run the pump on with the valves as they are, then close.
      overrun_ += dt;
      if (overrun_ >= c.overrun_s + c.close_delay_s)
        for (bool &o : open_) o = false;
    }

    for (int z = 0; z < NZONES; z++) any_open = any_open || (open_[z] && open_for_[z] >= c.actuator_s);
    pump_ = any_open && (active_ || overrun_ < c.overrun_s);
  }

  bool valve(int z) const { return open_[z]; }
  bool pump() const { return pump_; }
  bool active() const { return active_; }
  bool overrun() const { return !active_ && pump_; }

 protected:
  bool others_open_(int z) const {
    for (int i = 0; i < NZONES; i++)
      if (i != z && open_[i]) return true;
    return false;
  }
  bool any_open_() const {
    for (bool o : open_)
      if (o) return true;
    return false;
  }

  bool open_[NZONES] = {};
  float open_for_[NZONES] = {};
  float since_open_ = 1e9f, overrun_ = 0;
  bool pump_ = false, active_ = false, ending_ = false;
};

inline Demand &instance() {
  static Demand d;
  return d;
}

}  // namespace heating_demand
