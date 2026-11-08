// Re-initialise the LCD without restarting the board. With CS tied low the ILI9341 can't resync after
// an electrical glitch (seen 8 Nov when a zone relay switched) and goes white. mipi_spi's setup()
// resets the panel and sends the init sequence, but clears the sequence length when done, so it is
// captured at boot (before the display's own setup) and restored before each re-run.
#pragma once

#include <cstddef>
#include <cstdint>

namespace lcd_reset {

// Reads the display's protected init-sequence fields.
template<typename L> struct Peek : L {
  static const uint8_t *seq(L *l) { return static_cast<Peek *>(l)->init_sequence_; }
  static size_t len(L *l) { return static_cast<Peek *>(l)->init_sequence_len_; }
};

inline const uint8_t *&saved_seq() {
  static const uint8_t *s = nullptr;
  return s;
}
inline size_t &saved_len() {
  static size_t n = 0;
  return n;
}

// Call before the display's setup (on_boot priority 800).
template<typename L> void capture(L *lcd) {
  saved_seq() = Peek<L>::seq(lcd);
  saved_len() = Peek<L>::len(lcd);
}

// Reset the panel and send the init sequence again; the next redraw repaints the whole screen.
template<typename L> bool reinit(L *lcd) {
  if (saved_len() == 0 || saved_seq() == nullptr) return false;
  lcd->set_init_sequence(saved_seq(), saved_len());
  lcd->setup();
  lcd->update();
  return true;
}

}  // namespace lcd_reset
