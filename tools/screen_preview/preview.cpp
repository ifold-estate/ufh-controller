// Renders ufh_screen.h on a PC: shapes into a PPM, text as a list that render.py draws on top.
// Build and run with render.py, which also takes care of the fonts.
#define UFH_PREVIEW
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <string>

namespace ufh_screen {
struct Color {
  uint8_t r, g, b;
  Color(int r = 0, int g = 0, int b = 0) : r(r), g(g), b(b) {}
};
enum class TextAlign { TOP_LEFT, TOP_CENTER, CENTER, CENTER_LEFT, CENTER_RIGHT };
}  // namespace ufh_screen
#include "../../ufh_screen.h"

using namespace ufh_screen;

struct Font {
  const char *name;
};

struct Canvas {
  static const int WD = 320, HT = 240;
  Color px[HT][WD];
  FILE *text;
  void set(int x, int y, Color c) {
    if (x >= 0 && x < WD && y >= 0 && y < HT) px[y][x] = c;
  }
  void fill(Color c) {
    for (auto &row : px)
      for (auto &p : row) p = c;
  }
  void filled_rectangle(int x, int y, int w, int h, Color c) {
    for (int j = y; j < y + h; j++)
      for (int i = x; i < x + w; i++) set(i, j, c);
  }
  // Same rasterisation as ESPHome: dx^2 + dy^2 <= r^2 around the centre pixel.
  void filled_circle(int cx, int cy, int r, Color c) {
    for (int j = -r; j <= r; j++)
      for (int i = -r; i <= r; i++)
        if (i * i + j * j <= r * r) set(cx + i, cy + j, c);
  }
  void filled_triangle(int x1, int y1, int x2, int y2, int x3, int y3, Color c) {
    auto e = [](int ax, int ay, int bx, int by, int px, int py) { return (bx - ax) * (py - ay) - (by - ay) * (px - ax); };
    for (int y = std::min({y1, y2, y3}); y <= std::max({y1, y2, y3}); y++)
      for (int x = std::min({x1, x2, x3}); x <= std::max({x1, x2, x3}); x++) {
        int a = e(x1, y1, x2, y2, x, y), b = e(x2, y2, x3, y3, x, y), d = e(x3, y3, x1, y1, x, y);
        if ((a >= 0 && b >= 0 && d >= 0) || (a <= 0 && b <= 0 && d <= 0)) set(x, y, c);
      }
  }
  void print(int x, int y, Font *f, Color c, TextAlign a, const char *s) {
    fprintf(text, "%d\t%d\t%s\t%d\t%d\t%d\t%d\t%s\n", x, y, f->name, c.r, c.g, c.b, int(a), s);
  }
  void printf(int x, int y, Font *f, Color c, TextAlign a, const char *fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    print(x, y, f, c, a, buf);
  }
};

// Usage: preview OUT_PREFIX boiler mixed return mixer_pct boiler_on floor_pump_on zones(6 chars of 0/1)
//                rooms_calling(4 chars of 0/1) mode status ms
int main(int argc, char **argv) {
  if (argc != 13) return 1;
  static Canvas cv;
  std::string prefix = argv[1];
  cv.text = fopen((prefix + ".txt").c_str(), "w");
  auto num = [](const char *s) { return strcmp(s, "-") == 0 ? NAN : float(atof(s)); };
  State s;
  s.boiler = num(argv[2]);
  s.mixed = num(argv[3]);
  s.ret = num(argv[4]);
  s.mixer_pct = num(argv[5]);
  s.boiler_on = argv[6][0] == '1';
  s.floor_pump_on = argv[7][0] == '1';
  for (int i = 0; i < 6; i++) s.zone[i] = argv[8][i] == '1';
  for (int i = 0; i < 4; i++) s.room_calling[i] = argv[9][i] == '1';
  s.mode = argv[10];
  s.status = argv[11];
  s.ms = atoi(argv[12]);
  Font pct{"pct"}, temp{"temp"}, label{"label"}, small{"small"};
  draw(cv, s, &pct, &temp, &label, &small);
  fclose(cv.text);
  FILE *f = fopen((prefix + ".ppm").c_str(), "wb");
  fprintf(f, "P6 %d %d 255\n", Canvas::WD, Canvas::HT);
  for (auto &row : cv.px)
    for (auto &p : row) fwrite(&p, 1, 3, f);
  fclose(f);
}
