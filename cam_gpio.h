// Raw levels of the mixer gearbox cam switches on DI5-DI8 (GPIO8-11, active low), read directly so
// switch changes can be placed against the servo position without the binary sensors' filtering.
#pragma once

#include <cstdint>
#include <driver/gpio.h>

namespace cam_gpio {

// Bit n set = DI(5+n) active.
inline uint8_t read() {
  uint8_t v = 0;
  const gpio_num_t pins[4] = {GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11};
  for (int i = 0; i < 4; i++)
    if (gpio_get_level(pins[i]) == 0) v |= 1 << i;
  return v;
}

}  // namespace cam_gpio
