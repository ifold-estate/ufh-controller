#include "scs_servo.h"

#include <algorithm>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace scs_servo {

static const char *const TAG = "scs_servo";

void SCSServo::setup() {
  this->online_ = this->ping();
  if (!this->online_) {
    ESP_LOGW(TAG, "Servo ID %u did not answer ping (check wiring, power, baud rate and ID)", this->servo_id_);
  }
}

void SCSServo::dump_config() {
  ESP_LOGCONFIG(TAG, "SCS servo:");
  ESP_LOGCONFIG(TAG, "  ID: %u", this->servo_id_);
  ESP_LOGCONFIG(TAG, "  Response timeout: %u ms", (unsigned) this->response_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Online: %s", YESNO(this->online_));
  if (!this->last_failure_.empty())
    ESP_LOGCONFIG(TAG, "  Last failed reply: %s", this->last_failure_.c_str());
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Position", this->position_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_SENSOR("  ", "Load", this->load_sensor_);
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void SCSServo::update() {
  if (this->running_)
    return;  // loop() is polling at 100 ms during a run
  std::vector<uint8_t> d;
  if (!this->read_registers_(SCS_REG_PRESENT_POSITION, 15, &d)) {
    if (this->online_) {
      ESP_LOGW(TAG, "Servo ID %u stopped responding", this->servo_id_);
    }
    this->online_ = false;
    this->error_count_++;
    return;
  }
  this->online_ = true;

  uint16_t pos = (uint16_t(d[0]) << 8) | d[1];
  uint16_t spd_raw = (uint16_t(d[2]) << 8) | d[3];
  uint16_t load_raw = (uint16_t(d[4]) << 8) | d[5];
  uint8_t volt_raw = d[6];
  uint8_t temp_raw = d[7];

  // Sign is carried in a direction bit: bit 15 for speed, bit 10 for load.
  int speed = (spd_raw & 0x8000) ? -int(spd_raw & 0x7FFF) : int(spd_raw);
  int load = (load_raw & 0x0400) ? -int(load_raw & 0x03FF) : int(load_raw & 0x03FF);

  this->last_position_ = pos;
  this->last_load_ = load / 10.0f;  // 0..1000 -> 0..100 %

  if (this->position_sensor_ != nullptr)
    this->position_sensor_->publish_state(pos);
  if (this->speed_sensor_ != nullptr)
    this->speed_sensor_->publish_state(speed);
  if (this->load_sensor_ != nullptr)
    this->load_sensor_->publish_state(this->last_load_);
  if (this->voltage_sensor_ != nullptr)
    this->voltage_sensor_->publish_state(volt_raw / 10.0f);
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temp_raw);
  if (this->current_sensor_ != nullptr) {
    uint16_t cur_raw = (uint16_t(d[13]) << 8) | d[14];
    this->current_sensor_->publish_state((cur_raw & 0x8000) ? -int(cur_raw & 0x7FFF) : int(cur_raw));
  }

#ifdef USE_SWITCH
  std::vector<uint8_t> torque;
  if (this->torque_switch_ != nullptr && this->read_registers_(SCS_REG_TORQUE_ENABLE, 1, &torque)) {
    bool on = torque[0] != 0;
    if (on != this->torque_switch_->state)
      this->torque_switch_->publish_state(on);
  }
#endif
}

bool SCSServo::ping() {
  std::vector<uint8_t> reply;
  return this->transact_(SCS_INST_PING, {}, &reply, 0);
}

bool SCSServo::set_goal_position(uint16_t position) {
  if (position > 1023)
    position = 1023;
  ESP_LOGD(TAG, "Goal position -> %u", position);
  return this->write_registers_(SCS_REG_GOAL_POSITION, {uint8_t(position >> 8), uint8_t(position & 0xFF)});
}

bool SCSServo::set_torque(bool enable) {
  ESP_LOGD(TAG, "Torque %s", enable ? "ON" : "OFF");
  return this->write_registers_(SCS_REG_TORQUE_ENABLE, {uint8_t(enable ? 1 : 0)});
}

bool SCSServo::set_motor_mode(bool motor) {
  ESP_LOGI(TAG, "Setting servo ID %u to %s mode", this->servo_id_, motor ? "motor (continuous)" : "position");
  // EEPROM writes can take longer to acknowledge than normal ones: wait longer and retry each step.
  const uint32_t saved_timeout = this->response_timeout_ms_;
  this->response_timeout_ms_ = 100;
  auto write = [this](uint8_t reg, const std::vector<uint8_t> &data, const char *what) {
    for (int attempt = 1; attempt <= 3; attempt++) {
      if (this->write_registers_(reg, data))
        return true;
      ESP_LOGW(TAG, "  %s: no acknowledgement (attempt %d)", what, attempt);
      delay(20);
    }
    return false;
  };
  std::vector<uint8_t> limits = motor ? std::vector<uint8_t>{0, 0, 0, 0} : std::vector<uint8_t>{0, 0, 0x03, 0xFF};
  bool ok = write(SCS_REG_LOCK, {0}, "unlock");
  ok = write(SCS_REG_MIN_ANGLE_LIMIT, limits, "angle limits") && ok;
  ok = write(SCS_REG_LOCK, {1}, "lock") && ok;
  // Read back whatever happened, so the log always says which mode the servo is in.
  std::vector<uint8_t> check;
  bool read_ok = false;
  for (int attempt = 0; attempt < 3 && !read_ok; attempt++)
    read_ok = this->read_registers_(SCS_REG_MIN_ANGLE_LIMIT, 4, &check);
  this->response_timeout_ms_ = saved_timeout;
  if (read_ok) {
    uint16_t lo = (check[0] << 8) | check[1], hi = (check[2] << 8) | check[3];
    ESP_LOGI(TAG, "Angle limits now %u / %u: %s mode", lo, hi, (lo == 0 && hi == 0) ? "motor" : "position");
    return lo == 0 && hi == 0 ? motor : !motor;
  }
  ESP_LOGW(TAG, "Couldn't read the angle limits back");
  return false;
}

bool SCSServo::set_pwm(int duty) {
  duty = std::max(-1000, std::min(1000, duty));
  uint16_t v = duty < 0 ? uint16_t(-duty) | 0x0400 : uint16_t(duty);
  return this->write_registers_(SCS_REG_GOAL_TIME, {uint8_t(v >> 8), uint8_t(v & 0xFF)});
}

bool SCSServo::start_run(int duty, uint32_t max_ms, int stall_current) {
  if (this->running_)
    this->stop_run();
  if (!this->write_registers_(SCS_REG_TORQUE_ENABLE, {1}) || !this->set_pwm(duty)) {
    ESP_LOGW(TAG, "Run start failed");
    this->last_run_ = RunResult{};
    this->last_run_.failed = true;
    return false;
  }
  ESP_LOGD(TAG, "Run: duty %d for up to %u ms, stall current %d", duty, (unsigned) max_ms, stall_current);
  this->running_ = true;
  this->run_duty_ = duty;
  this->run_start_ = this->last_poll_ = millis();
  this->run_max_ms_ = max_ms;
  this->run_stall_current_ = stall_current;
  this->stall_count_ = this->poll_failures_ = 0;
  this->run_ = RunResult{};
  return true;
}

void SCSServo::stop_run() {
  if (this->running_)
    this->finish_run_(false, false);
}

void SCSServo::finish_run_(bool stalled, bool failed) {
  // Stop even if the servo is answering badly: try twice.
  if (!this->set_pwm(0))
    this->set_pwm(0);
  this->run_.ms = millis() - this->run_start_;
  this->run_.stalled = stalled;
  this->run_.failed = failed;
  this->last_run_ = this->run_;
  this->running_ = false;
  ESP_LOGD(TAG, "Run done: %u ms%s%s, peak current %d", (unsigned) this->run_.ms, stalled ? ", stalled" : "",
           failed ? ", FAILED" : "", this->run_.peak_current);
}

void SCSServo::loop() {
  if (!this->running_)
    return;
  const uint32_t now = millis();
  if (now - this->run_start_ >= this->run_max_ms_) {
    this->finish_run_(false, false);
    return;
  }
  if (now - this->last_poll_ < 100)
    return;
  this->last_poll_ = now;
  std::vector<uint8_t> d;
  if (!this->read_registers_(SCS_REG_PRESENT_POSITION, 15, &d)) {
    if (++this->poll_failures_ >= 3)
      this->finish_run_(false, true);
    return;
  }
  this->poll_failures_ = 0;
  uint16_t pos = (uint16_t(d[0]) << 8) | d[1];
  uint16_t spd_raw = (uint16_t(d[2]) << 8) | d[3];
  uint16_t cur_raw = (uint16_t(d[13]) << 8) | d[14];
  int speed = (spd_raw & 0x8000) ? -int(spd_raw & 0x7FFF) : int(spd_raw);
  int current = (cur_raw & 0x8000) ? -int(cur_raw & 0x7FFF) : int(cur_raw);
  this->last_position_ = pos;
  this->run_.peak_current = std::max(this->run_.peak_current, std::abs(current));
  uint16_t load_raw = (uint16_t(d[4]) << 8) | d[5];
  int load = (load_raw & 0x0400) ? -int(load_raw & 0x03FF) : int(load_raw & 0x03FF);
  ESP_LOGD(TAG, "  t=%4u ms pos=%4u speed=%5d load=%5d current=%4d moving=%u volt=%.1f", (unsigned) (now - this->run_start_),
           pos, speed, load, current, d[10], d[6] / 10.0f);
  if (this->end_check_ && this->end_check_(this->run_duty_ > 0 ? 1 : -1)) {
    ESP_LOGD(TAG, "  end of travel signalled");
    this->finish_run_(true, false);
    return;
  }
  // Ignore the start-up surge, then need two polls in a row over the threshold.
  if (this->run_stall_current_ > 0 && now - this->run_start_ > 400 && std::abs(current) >= this->run_stall_current_) {
    if (++this->stall_count_ >= 2)
      this->finish_run_(true, false);
  } else {
    this->stall_count_ = 0;
  }
}

bool SCSServo::read_registers_(uint8_t address, uint8_t length, std::vector<uint8_t> *data) {
  return this->transact_(SCS_INST_READ, {address, length}, data, length);
}

bool SCSServo::write_registers_(uint8_t address, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> params;
  params.reserve(data.size() + 1);
  params.push_back(address);
  params.insert(params.end(), data.begin(), data.end());
  std::vector<uint8_t> reply;
  return this->transact_(SCS_INST_WRITE, params, &reply, 0);
}

// Sends one instruction packet and waits for the status packet.
// Packet: FF FF ID LEN INSTR PARAMS... CHK, LEN = params + 2, CHK = ~(ID + LEN + INSTR + params).
// Half-duplex adapters may echo our own transmission back on RX; that echo is skipped.
bool SCSServo::transact_(uint8_t instruction, const std::vector<uint8_t> &params, std::vector<uint8_t> *reply,
                         size_t reply_params) {
  // Drop any stale bytes.
  while (this->available())
    this->read();

  std::vector<uint8_t> tx;
  tx.reserve(params.size() + 6);
  tx.push_back(0xFF);
  tx.push_back(0xFF);
  tx.push_back(this->servo_id_);
  tx.push_back(uint8_t(params.size() + 2));
  tx.push_back(instruction);
  tx.insert(tx.end(), params.begin(), params.end());
  uint8_t sum = 0;
  for (size_t i = 2; i < tx.size(); i++)
    sum += tx[i];
  tx.push_back(uint8_t(~sum));

  this->write_array(tx);
  this->flush();

  const size_t reply_len = reply_params + 6;  // FF FF ID LEN ERR params CHK
  std::vector<uint8_t> rx;
  rx.reserve(tx.size() + reply_len);
  const uint32_t start = millis();
  bool echo_checked = false;
  bool echo_seen = false;
  while (millis() - start < this->response_timeout_ms_) {
    while (this->available()) {
      uint8_t b;
      this->read_byte(&b);
      rx.push_back(b);
    }
    // Strip an exact echo of our transmission, if present.
    if (!echo_checked && rx.size() >= tx.size()) {
      echo_checked = true;
      if (std::equal(tx.begin(), tx.end(), rx.begin())) {
        rx.erase(rx.begin(), rx.begin() + tx.size());
        echo_seen = true;
      }
    }
    if (echo_checked && rx.size() >= reply_len)
      break;
    delay(1);
  }

  // Find the status packet header.
  for (size_t i = 0; i + reply_len <= rx.size(); i++) {
    if (rx[i] != 0xFF || rx[i + 1] != 0xFF || rx[i + 2] != this->servo_id_)
      continue;
    if (rx[i + 3] != reply_params + 2)
      continue;
    uint8_t s = 0;
    for (size_t j = i + 2; j < i + reply_len - 1; j++)
      s += rx[j];
    if (uint8_t(~s) != rx[i + reply_len - 1]) {
      ESP_LOGW(TAG, "Checksum error in reply");
      return false;
    }
    uint8_t err = rx[i + 4];
    if (err != 0)
      ESP_LOGW(TAG, "Servo reports error flags 0x%02X", err);
    this->last_failure_.clear();
    reply->assign(rx.begin() + i + 5, rx.begin() + i + 5 + reply_params);
    return true;
  }
  // Say what did arrive, which tells wiring faults apart: nothing at all (RX path or adapter), only
  // our own echo (TX path works, servo silent), or garbage (baud rate or level shifter). Only logged
  // when it changes, since this repeats every poll while the servo is offline.
  std::string summary = echo_seen ? "echo of our packet, then " : "";
  summary += rx.empty() ? "nothing" : format_hex_pretty(rx);
  if (summary != this->last_failure_) {
    ESP_LOGD(TAG, "No valid reply from ID %u: received %s", this->servo_id_, summary.c_str());
    this->last_failure_ = summary;
  }
  return false;
}

#ifdef USE_NUMBER
void SCSGoalNumber::control(float value) {
  if (this->parent_->set_goal_position(uint16_t(value)))
    this->publish_state(value);
}
#endif

#ifdef USE_SWITCH
void SCSTorqueSwitch::write_state(bool state) {
  if (this->parent_->set_torque(state))
    this->publish_state(state);
}
#endif

}  // namespace scs_servo
}  // namespace esphome
