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
  bool zone[6] = {};           // manifold valves: Playroom, Kitchen x2, Booth x2, Lounge
  bool room_calling[4] = {};   // thermostat calling for heat (room cold): Playroom, Kitchen, Booth, Lounge
  const char *mode = "";       // e.g. AUTO, OVERRIDE, MANUAL
  const char *status = "";     // e.g. Warming, Holding, Waiting
  uint32_t ms = 0;             // millis(), drives the animation
};

// Temperature colour scale: blue at 10 °C, orange at the 35 °C set point, red at 60 °C. Below the set
// point it runs through sky blue and amber rather than straight to blue, which goes purple midway.
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
inline Color c_sky() { return Color(0x30, 0xB0, 0xF0); }
inline Color temp_colour(float t) {
  if (std::isnan(t)) return c_grey();
  struct Stop {
    float t;
    Color c;
  };
  const Stop stops[] = {{10, c_blue()}, {20, c_sky()}, {28, Color(0xF5, 0xD0, 0x6E)}, {35, c_orange()}, {60, c_red()}};
  if (t <= stops[0].t) return stops[0].c;
  for (int i = 1; i < 5; i++)
    if (t <= stops[i].t) return mix(stops[i - 1].c, stops[i].c, (t - stops[i - 1].t) / (stops[i].t - stops[i - 1].t));
  return stops[4].c;
}

// Layout. Pipe centre lines: left and right verticals, top and bottom runs.
constexpr int W = 8;                     // pipe width
constexpr int L = 14, R = 238, T = 40, B = 168;
constexpr int MX0 = 96, MX1 = 176;       // mixer box, left/right
constexpr int MY0 = 4, MY1 = 100;        // mixer box, top/bottom
constexpr int TEMP_Y = 76;               // all three temperatures sit on this line
constexpr int MXC = (MX0 + MX1) / 2;     // mixer stem (return into the mixer)
constexpr int FLAME_Y = 105;             // flame sits in a gap in the left vertical
constexpr int ZONE_Y0 = 92, ZONE_DY = 16;

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
    it.print(MXC, 34, f_label, c_grey(), TextAlign::CENTER, "offline");
  else
    it.printf(MXC, 34, f_pct, Color(255, 255, 255), TextAlign::CENTER, "%.0f%%", s.mixer_pct);

  // Temperatures in line, each in its pipe's colour: boiler flow, return (in the mixer, above its
  // inlet), mixed flow.
  auto temp = [&](int x, float t, Color c) {
    if (std::isnan(t))
      it.print(x, TEMP_Y, f_temp, c_grey(), TextAlign::CENTER, "--");
    else
      it.printf(x, TEMP_Y, f_temp, c, TextAlign::CENTER, "%.0f°C", t);
  };
  temp((L + MX0) / 2 + 2, s.boiler, hot);
  temp(MXC, s.ret, ret);
  temp((MX1 + R) / 2 - 3, s.mixed, mixed);

  // Boiler (flame) and the two pumps
  flame(it, L, FLAME_Y, s.boiler_on);
  pump_arrow(it, (L + MX0) / 2 + 2, 130, 18, s.boiler_on ? c_green() : c_grey());
  pump_arrow(it, (MX1 + R) / 2, 130, 18, s.floor_pump_on ? c_green() : c_grey());

  // Manifold zones: blue closed (cold), orange open (hot). Room names show the thermostats: blue
  // while one calls for heat (room cold), orange when satisfied (room warm).
  static const char *const NAMES[4] = {"Playroom", "Kitchen", "Booth", "Lounge"};
  static const int8_t ZONES[4][2] = {{0, -1}, {1, 2}, {3, 4}, {5, -1}};
  for (int row = 0; row < 4; row++) {
    int y = ZONE_Y0 + row * ZONE_DY;
    bool pair = ZONES[row][1] >= 0;
    for (int k = 0; k < (pair ? 2 : 1); k++) {
      int x = pair ? R - 7 + 14 * k : R;
      it.filled_circle(x, y, 6, s.zone[ZONES[row][k]] ? c_orange() : c_blue());
    }
    it.print(R + 17, y, f_label, s.room_calling[row] ? c_sky() : c_orange(), TextAlign::CENTER_LEFT, NAMES[row]);
  }

  // Status line: mode in a pill, then what the controller is doing
  constexpr int SY = 206;
  if (*s.mode) {
    rounded_box(it, 6, SY - 14, 96, SY + 14, 8, Color(0xC8, 0xD0, 0xD8));
    rounded_box(it, 8, SY - 12, 94, SY + 12, 6, Color(0, 0, 0));
    it.print(51, SY, f_label, Color(255, 255, 255), TextAlign::CENTER, s.mode);
  }
  it.print(110, SY, f_temp, Color(0xE0, 0xE0, 0xE0), TextAlign::CENTER_LEFT, s.status);
}

}  // namespace ufh_screen
