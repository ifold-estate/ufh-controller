#pragma once

#include <functional>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

namespace esphome {
namespace scs_servo {

// Feetech SCS-series (SCSCL) memory table. Multi-byte values are big-endian.
static const uint8_t SCS_INST_PING = 0x01;
static const uint8_t SCS_INST_READ = 0x02;
static const uint8_t SCS_INST_WRITE = 0x03;

static const uint8_t SCS_REG_MIN_ANGLE_LIMIT = 9;  // 2 bytes, then max angle limit (2); both 0 = motor mode
static const uint8_t SCS_REG_TORQUE_ENABLE = 40;
static const uint8_t SCS_REG_GOAL_POSITION = 42;  // 2 bytes
static const uint8_t SCS_REG_GOAL_TIME = 44;      // 2 bytes; in motor mode the drive (PWM), bit 10 = direction
static const uint8_t SCS_REG_GOAL_SPEED = 46;     // 2 bytes
static const uint8_t SCS_REG_PRESENT_POSITION = 56;  // 2 bytes, then speed(2), load(2), voltage(1), temp(1)
static const uint8_t SCS_REG_LOCK = 48;           // 0 = EEPROM writes persist, 1 = locked
static const uint8_t SCS_REG_MOVING = 66;
static const uint8_t SCS_REG_PRESENT_CURRENT = 69;  // 2 bytes, bit 15 = sign

// Result of a timed motor-mode run (see start_run).
struct RunResult {
  uint32_t ms{0};          // how long it actually drove
  bool stalled{false};     // stopped because the current stayed over the stall threshold
  bool failed{false};      // lost contact with the servo during the run
  int peak_current{0};     // highest |current| seen (raw units)
};

class SCSServo : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_servo_id(uint8_t id) { this->servo_id_ = id; }
  void set_response_timeout(uint32_t ms) { this->response_timeout_ms_ = ms; }
  void set_position_sensor(sensor::Sensor *s) { this->position_sensor_ = s; }
  void set_speed_sensor(sensor::Sensor *s) { this->speed_sensor_ = s; }
  void set_load_sensor(sensor::Sensor *s) { this->load_sensor_ = s; }
  void set_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { this->current_sensor_ = s; }
#ifdef USE_SWITCH
  // The servo turns its own torque on when it gets a goal position, so the switch is refreshed from
  // the torque register on every poll rather than trusting the last command.
  void set_torque_switch(switch_::Switch *s) { this->torque_switch_ = s; }
#endif

  void loop() override;

  // Public API for automations / lambdas.
  bool ping();
  // Motor (continuous rotation) mode: angle limits 0/0. Position mode: 0/1023. Stored in the servo's
  // EEPROM, so this is a one-off change.
  bool set_motor_mode(bool motor);
  // Motor mode drive: -1000..1000 (open-loop PWM duty, sign = direction). 0 stops.
  bool set_pwm(int duty);
  // Drive at `duty` for up to `max_ms`, polling every 100 ms. Stops early if the current stays at or
  // above `stall_current` (raw units; 0 = no stall detection). Every poll is logged at DEBUG.
  bool start_run(int duty, uint32_t max_ms, int stall_current);
  void stop_run();
  // Optional: called every poll during a run with the drive direction (+1/-1); returning true ends
  // the run as if stalled. For end-of-travel limit switches.
  void set_end_check(std::function<bool(int)> f) { this->end_check_ = std::move(f); }
  bool is_running() const { return this->running_; }
  const RunResult &last_run() const { return this->last_run_; }
  bool set_goal_position(uint16_t position);
  bool set_torque(bool enable);
  bool is_online() const { return this->online_; }
  int last_position() const { return this->last_position_; }
  float last_load_percent() const { return this->last_load_; }

 protected:
  bool transact_(uint8_t instruction, const std::vector<uint8_t> &params, std::vector<uint8_t> *reply,
                 size_t reply_params);
  bool read_registers_(uint8_t address, uint8_t length, std::vector<uint8_t> *data);
  bool write_registers_(uint8_t address, const std::vector<uint8_t> &data);

  uint8_t servo_id_{1};
  uint32_t response_timeout_ms_{20};
  bool online_{false};
  uint32_t error_count_{0};
  std::string last_failure_;  // last logged failed-reply summary, to avoid repeating it
  void finish_run_(bool stalled, bool failed);

  int last_position_{-1};
  float last_load_{0};

  bool running_{false};
  int run_duty_{0};
  uint32_t run_start_{0}, run_max_ms_{0}, last_poll_{0};
  int run_stall_current_{0};
  uint8_t stall_count_{0}, poll_failures_{0};
  RunResult run_{}, last_run_{};
  std::function<bool(int)> end_check_;

  sensor::Sensor *position_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *load_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
#ifdef USE_SWITCH
  switch_::Switch *torque_switch_{nullptr};
#endif
};

#ifdef USE_NUMBER
class SCSGoalNumber : public number::Number, public Component {
 public:
  void set_parent(SCSServo *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;
  SCSServo *parent_{nullptr};
};
#endif

#ifdef USE_SWITCH
class SCSTorqueSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(SCSServo *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  SCSServo *parent_{nullptr};
};
#endif

}  // namespace scs_servo
}  // namespace esphome
