// Heat-flow screen for the 320x240 LCD: boiler loop on the left, floor loop on the right, the mixer
// between them. Pipes take their colour from the water temperature; the flame, pumps and zone dots
// follow the relays; water "bubbles" move clockwise round a loop while its pump runs.
//
// Plain C++ so tools/screen_preview can render it on a PC. On the device it is called from the
// display lambda in ufh-controller.yaml.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>

#ifndef UFH_PREVIEW
#include "esphome/components/display/display.h"
#endif

namespace ufh_screen {

#ifndef UFH_PREVIEW
using esphome::Color;
using esphome::display::TextAlign;
#endif

struct State {
  float boiler = NAN, mixed = NAN, ret = NAN;  // °C, NAN when a probe has no reading
  float mixer_pct = NAN;                        // NAN when the servo is offline
  bool boiler_on = false;                       // CFH + boiler pump (one relay)
  bool floor_pump_on = false;
  bool zone[6] = {};  // Playroom, Kitchen x2, Booth x2, Lounge
  uint32_t ms = 0;    // millis(), drives the animation
};

// Temperature colour scale: blue at 10 °C, orange at the 35 °C set point, red at 60 °C.
constexpr float T_COLD = 10, T_SET = 35, T_HOT = 60;
inline Color mix(Color a, Color b, float f) {
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return Color(a.r + (b.r - a.r) * f, a.g + (b.g - a.g) * f, a.b + (b.b - a.b) * f);
}
inline Color c_blue() { return Color(0x22, 0x33, 0xFF); }
inline Color c_orange() { return Color(0xF0, 0x9A, 0x38); }
inline Color c_red() { return Color(0xE8, 0x38, 0x2A); }
inline Color c_grey() { return Color(0x55, 0x55, 0x55); }
inline Color c_green() { return Color(0x4C, 0xAF, 0x50); }
inline Color temp_colour(float t) {
  if (std::isnan(t)) return c_grey();
  if (t >= T_SET) return mix(c_orange(), c_red(), (t - T_SET) / (T_HOT - T_SET));
  return mix(c_orange(), c_blue(), (T_SET - t) / (T_SET - T_COLD));
}

// Layout. Pipe centre lines: left and right verticals, top and bottom runs.
constexpr int W = 8;                     // pipe width
constexpr int L = 14, R = 238, T = 40, B = 168;
constexpr int MX0 = 100, MX1 = 172;      // mixer box, left/right
constexpr int MY0 = 10, MY1 = 82;        // mixer box, top/bottom
constexpr int MXC = (MX0 + MX1) / 2;     // mixer stem (return into the mixer)
constexpr int FLAME_Y = 105;             // flame sits in a gap in the left vertical
constexpr int ZONE_Y0 = 80, ZONE_DY = 16;

struct Pt {
  int x, y;
};

// A pipe run through a list of corner points, drawn with rounded joints.
template<typename D> void pipe(D &it, const Pt *p, int n, Color c) {
  for (int i = 0; i + 1 < n; i++) {
    int x0 = std::min(p[i].x, p[i + 1].x), x1 = std::max(p[i].x, p[i + 1].x);
    int y0 = std::min(p[i].y, p[i + 1].y), y1 = std::max(p[i].y, p[i + 1].y);
    it.filled_rectangle(x0 - W / 2, y0 - W / 2, x1 - x0 + W, y1 - y0 + W, c);
  }
}

// Bubbles every SPACING px along the run, moving in the run's direction.
template<typename D> void bubbles(D &it, const Pt *p, int n, Color c, uint32_t ms) {
  constexpr int SPACING = 16;
  const int phase = (ms / 40) % SPACING;  // 25 px/s
  Color bc = mix(c, Color(255, 255, 255), 0.6f);
  int dist = 0;  // distance travelled along the run so far
  for (int i = 0; i + 1 < n; i++) {
    int dx = (p[i + 1].x > p[i].x) - (p[i + 1].x < p[i].x);
    int dy = (p[i + 1].y > p[i].y) - (p[i + 1].y < p[i].y);
    int len = std::abs(p[i + 1].x - p[i].x) + std::abs(p[i + 1].y - p[i].y);
    int first = ((phase - dist) % SPACING + SPACING) % SPACING;
    for (int s = first; s <= len; s += SPACING)
      it.filled_rectangle(p[i].x + dx * s - 2, p[i].y + dy * s - 2, 4, 4, bc);
    dist += len;
  }
}

template<typename D> void rounded_box(D &it, int x0, int y0, int x1, int y1, int r, Color c) {
  it.filled_rectangle(x0 + r, y0, x1 - x0 - 2 * r, y1 - y0, c);
  it.filled_rectangle(x0, y0 + r, x1 - x0, y1 - y0 - 2 * r, c);
  it.filled_circle(x0 + r, y0 + r, r, c);
  it.filled_circle(x1 - r - 1, y0 + r, r, c);
  it.filled_circle(x0 + r, y1 - r - 1, r, c);
  it.filled_circle(x1 - r - 1, y1 - r - 1, r, c);
}

// Clockwise circulation arrow, like the pump symbol in the sketch.
template<typename D> void pump_arrow(D &it, int cx, int cy, int r, Color c) {
  const float a0 = -100 * M_PI / 180, a1 = 150 * M_PI / 180;
  for (float a = a0; a <= a1; a += 0.06f)
    it.filled_circle(cx + r * cosf(a), cy + r * sinf(a), 2, c);
  float px = cx + r * cosf(a1), py = cy + r * sinf(a1);
  float tx = -sinf(a1), ty = cosf(a1);  // clockwise tangent
  float nx = cosf(a1), ny = sinf(a1);
  it.filled_triangle(px + tx * 9, py + ty * 9, px + nx * 7 - tx * 2, py + ny * 7 - ty * 2, px - nx * 7 - tx * 2,
                     py - ny * 7 - ty * 2, c);
}

template<typename D> void flame(D &it, int x, int y, bool on) {
  Color outer = on ? c_red() : c_grey();
  Color inner = on ? c_orange() : Color(0x80, 0x80, 0x80);
  it.filled_circle(x, y + 7, 10, outer);
  it.filled_triangle(x - 10, y + 5, x + 10, y + 5, x + 2, y - 16, outer);
  it.filled_triangle(x - 10, y + 6, x - 4, y - 4, x - 9, y - 9, outer);
  it.filled_circle(x, y + 10, 5, inner);
  it.filled_triangle(x - 5, y + 9, x + 5, y + 9, x + 1, y - 3, inner);
}

template<typename D, typename F>
void draw(D &it, const State &s, F *f_pct, F *f_temp, F *f_label, F *f_small) {
  it.fill(Color(0, 0, 0));
  const Color hot = temp_colour(s.boiler), mixed = temp_colour(s.mixed), ret = temp_colour(s.ret);

  // Runs are listed in the direction the water flows (clockwise round each loop).
  const Pt boiler_flow[] = {{L, FLAME_Y - 18}, {L, T}, {MX0, T}};
  const Pt boiler_return[] = {{MXC, B}, {L, B}, {L, FLAME_Y + 20}};
  const Pt floor_flow[] = {{MX1, T}, {R, T}, {R, ZONE_Y0 - 10}};
  const Pt floor_return[] = {{R, ZONE_Y0 + 3 * ZONE_DY + 10}, {R, B}, {MXC, B}, {MXC, MY1}};
  pipe(it, boiler_flow, 3, hot);
  pipe(it, floor_flow, 3, mixed);
  pipe(it, floor_return, 4, ret);
  pipe(it, boiler_return, 3, ret);
  if (s.boiler_on) {
    bubbles(it, boiler_flow, 3, hot, s.ms);
    bubbles(it, boiler_return, 3, ret, s.ms);
  }
  if (s.floor_pump_on) {
    bubbles(it, floor_flow, 3, mixed, s.ms);
    bubbles(it, floor_return, 4, ret, s.ms);
  }

  // Mixer
  rounded_box(it, MX0, MY0, MX1, MY1, 12, Color(0xC8, 0xD0, 0xD8));
  rounded_box(it, MX0 + 4, MY0 + 4, MX1 - 4, MY1 - 4, 9, Color(0, 0, 0));
  if (std::isnan(s.mixer_pct))
    it.print(MXC, (MY0 + MY1) / 2, f_label, c_grey(), TextAlign::CENTER, "offline");
  else
    it.printf(MXC, (MY0 + MY1) / 2, f_pct, Color(255, 255, 255), TextAlign::CENTER, "%.0f%%", s.mixer_pct);

  // Temperatures, each in its pipe's colour
  auto temp = [&](int x, int y, float t, Color c) {
    if (std::isnan(t))
      it.print(x, y, f_temp, c_grey(), TextAlign::TOP_CENTER, "--");
    else
      it.printf(x, y, f_temp, c, TextAlign::TOP_CENTER, "%.0f°C", t);
  };
  temp((L + MX0) / 2 + 2, T + 9, s.boiler, hot);
  temp((MX1 + R) / 2, T + 9, s.mixed, mixed);
  temp((MX1 + R) / 2, B + 8, s.ret, ret);

  // Boiler (flame) and the two pumps
  flame(it, L, FLAME_Y, s.boiler_on);
  pump_arrow(it, (L + MX0) / 2 + 2, 118, 19, s.boiler_on ? c_green() : c_grey());
  pump_arrow(it, (MX1 + R) / 2, 114, 17, s.floor_pump_on ? c_green() : c_grey());

  // Manifold zones: blue closed (cold), orange open (hot)
  static const char *const NAMES[4] = {"Playroom", "Kitchen", "Booth", "Lounge"};
  static const int8_t ZONES[4][2] = {{0, -1}, {1, 2}, {3, 4}, {5, -1}};
  for (int row = 0; row < 4; row++) {
    int y = ZONE_Y0 + row * ZONE_DY;
    bool pair = ZONES[row][1] >= 0;
    for (int k = 0; k < (pair ? 2 : 1); k++) {
      int x = pair ? R - 7 + 14 * k : R;
      it.filled_circle(x, y, 6, s.zone[ZONES[row][k]] ? c_orange() : c_blue());
    }
    it.print(R + 17, y, f_label, Color(0xE0, 0xE0, 0xE0), TextAlign::CENTER_LEFT, NAMES[row]);
  }

  // Colour scale
  constexpr int SX0 = 40, SX1 = 280, SY = 212, SH = 14;
  for (int x = SX0; x < SX1; x++) {
    float t = T_HOT - (T_HOT - T_COLD) * (x - SX0) / float(SX1 - SX0);
    it.filled_rectangle(x, SY, 1, SH, temp_colour(t));
  }
  it.print(SX0 - 3, SY + SH / 2, f_small, c_red(), TextAlign::CENTER_RIGHT, "60°C");
  it.print(SX1 + 3, SY + SH / 2, f_small, c_blue(), TextAlign::CENTER_LEFT, "10°C");
  it.print((SX0 + SX1) / 2, SY + SH / 2, f_small, Color(255, 255, 255), TextAlign::CENTER, "35°C");
}

}  // namespace ufh_screen
