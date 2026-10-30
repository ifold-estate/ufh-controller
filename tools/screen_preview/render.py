"""Preview the LCD heat-flow screen (ufh_screen.h) as PNGs, without the board.

Usage: render.py OUT_DIR
Writes a few example states, plus an animated GIF of the pumps running.
Fonts and sizes must match the font: section of ufh-controller.yaml.
"""
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONTS = os.path.join(HERE, "..", "..", "fonts")
FONT_FILES = {  # id -> (file, size), as in ufh-controller.yaml
    "pct": ("DejaVuSans-Bold.ttf", 22),
    "temp": ("DejaVuSans-Bold.ttf", 20),
    "label": ("DejaVuSans.ttf", 13),
    "small": ("DejaVuSans.ttf", 12),
}
ANCHORS = ["la", "ma", "mm", "lm", "rm"]  # TextAlign order in preview.cpp
SCALE = 2

STATES = {  # name: boiler mixed return mixer% boiler_on floor_pump_on zones rooms_calling mode status
    "heating": ("58", "37", "24", "67", "1", "1", "110101", "1101", "AUTO", "Holding"),
    "idle": ("21", "20", "19", "0", "0", "0", "000000", "0000", "AUTO", "Idle"),
    "warming": ("33", "29", "21", "100", "1", "1", "011000", "0100", "AUTO", "Warming"),
    "faults": ("-", "36", "-", "-", "0", "1", "100001", "1001", "MANUAL", "Mixer offline"),
}


def frame(exe, out_dir, name, args, ms):
    prefix = os.path.join(out_dir, name)
    subprocess.run([exe, prefix, *args, str(ms)], check=True)
    img = Image.open(prefix + ".ppm").convert("RGB")
    img = img.resize((img.width * SCALE, img.height * SCALE), Image.NEAREST)
    d = ImageDraw.Draw(img)
    fonts = {k: ImageFont.truetype(os.path.join(FONTS, f), s * SCALE) for k, (f, s) in FONT_FILES.items()}
    for line in open(prefix + ".txt", encoding="utf-8"):
        x, y, font, r, g, b, align, text = line.rstrip("\n").split("\t", 7)
        d.text((int(x) * SCALE, int(y) * SCALE), text, fill=(int(r), int(g), int(b)),
               font=fonts[font], anchor=ANCHORS[int(align)])
    os.remove(prefix + ".ppm")
    os.remove(prefix + ".txt")
    return img


def main():
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    exe = os.path.join(out_dir, "preview")
    subprocess.run(["c++", "-std=c++17", "-O1", "-o", exe, os.path.join(HERE, "preview.cpp")], check=True)
    for name, args in STATES.items():
        frame(exe, out_dir, name, args, 0).save(os.path.join(out_dir, name + ".png"))
    frames = [frame(exe, out_dir, "anim", STATES["heating"], ms) for ms in range(0, 640, 80)]
    frames[0].save(os.path.join(out_dir, "heating.gif"), save_all=True, append_images=frames[1:],
                   duration=80, loop=0)
    print("wrote", out_dir)


main()
