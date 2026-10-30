#pragma once

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

static const uint8_t SCS_REG_TORQUE_ENABLE = 40;
static const uint8_t SCS_REG_GOAL_POSITION = 42;  // 2 bytes
static const uint8_t SCS_REG_GOAL_TIME = 44;      // 2 bytes
static const uint8_t SCS_REG_GOAL_SPEED = 46;     // 2 bytes
static const uint8_t SCS_REG_PRESENT_POSITION = 56;  // 2 bytes, then speed(2), load(2), voltage(1), temp(1)
static const uint8_t SCS_REG_MOVING = 66;

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

  // Public API for automations / lambdas.
  bool ping();
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
  int last_position_{-1};
  float last_load_{0};

  sensor::Sensor *position_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *load_sensor_{nullptr};
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
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
