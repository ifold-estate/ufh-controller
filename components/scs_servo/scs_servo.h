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

// Result of a motor-mode run (see start_run).
struct RunResult {
  uint32_t ms{0};          // how long it drove (not counting the coast afterwards)
  bool stopped{false};     // ended by the stop check rather than the time limit
  bool failed{false};      // lost contact with the servo during the run
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
  // Drive at `duty` for up to `max_ms`, starting with `kick_duty` for `kick_ms` (to break stiction),
  // polling every 100 ms; then keep polling for 600 ms while the shaft coasts. Every poll is logged
  // at DEBUG. is_running() stays true until the coast is over.
  bool start_run(int duty, uint32_t max_ms, int kick_duty = 0, uint32_t kick_ms = 0);
  void stop_run();
  // Called with every position reading (runs, coasts and idle polls): the drive direction then
  // (+1/-1 while driving or coasting, 0 when idle), seconds since the last reading, and whether the
  // motor is being driven (false while coasting or idle).
  void set_sample_callback(std::function<void(int raw, int dir, float dt, bool driving)> f) {
    this->sample_cb_ = std::move(f);
  }
  // Polled during a run, after each sample: return true to stop driving now.
  void set_stop_check(std::function<bool()> f) { this->stop_check_ = std::move(f); }
  bool is_running() const { return this->running_; }
  // Direction of the run in progress (+1/-1, including its coast), 0 when idle.
  int run_direction() const { return this->running_ ? (this->run_duty_ > 0 ? 1 : -1) : 0; }
  // Milliseconds since the current run started (0 when idle).
  uint32_t run_elapsed_ms() const { return this->running_ ? millis() - this->run_start_ : 0; }
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
  void finish_run_(bool stopped, bool failed);
  bool poll_motion_(int dir);  // read, log, feed the sample callback

  int last_position_{-1};
  float last_load_{0};

  bool running_{false}, coasting_{false}, kicking_{false};
  int run_duty_{0};
  uint32_t run_start_{0}, run_max_ms_{0}, kick_ms_{0}, last_poll_{0}, last_sample_{0}, coast_start_{0};
  uint8_t poll_failures_{0};
  RunResult run_{}, last_run_{};
  std::function<void(int, int, float, bool)> sample_cb_;
  std::function<bool()> stop_check_;

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
