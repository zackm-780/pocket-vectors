#!/usr/bin/env python3
"""Box art for the handheld's Ports menu, lettered in the game's own HUD stroke font.

Takes an in-game render (a 4:3 screenshot, any size) and writes:
  * screenshot.png       640 x 480, what gameinfo.xml and PortMaster show
  * PocketVectors.png    300 x 225, the stock Anbernic OS box art (Roms/PORTS/Imgs/)

The title glyphs are read from src/hud/StrokeFont.cpp, so the lettering matches the HUD: strokes
1/8 of the cap height wide with square caps, HUD green over a dark outline.

How the 1.2.2 art was made (pdx_oth data pack, HIGH, chase view, MIN declutter so the HUD only
leaves its two value boxes at the edges, which the crop removes):

    xvfb-run -a -s "-screen 0 1280x960x24" ./build/flightsim --window 1280x960 \
        --data-dir <pack>/data --conf-dir <conf with external_view = true,
        hud_declutter_default = min, graphics_quality = high> \
        --scenario <737, KPDX, start = cruise, 5,500 ft, 2026-06-21T20:00:00> \
        --frames 90 --screenshot base.png        # with FSIM_LOOK_DEG=150,-2
    python3 tools/art/make_tile.py --base base.png --crop 200,80,880,660 --out-dir port

Needs Pillow (pip install pillow). Not part of the build.
"""

import argparse
import os
import re
import sys

from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
FONT_SRC = os.path.join(HERE, "..", "..", "src", "hud", "StrokeFont.cpp")
HUD_GREEN = (51, 255, 77, 255)
OUTLINE = (0, 0, 0, 153)  # black at 0.6 alpha, as Renderer2D draws it
SS = 4  # supersampling for smooth strokes


def load_glyphs(path):
    """Character -> list of (points, closed) strokes, in grid units (x 0..4, y 0..8 down)."""
    text = open(path, encoding="utf-8").read()
    glyphs = {}
    for ch, spec in re.findall(r"\{'(.|\\')', \"([^\"]*)\"\}", text):
        ch = "'" if ch == "\\'" else ch
        strokes = []
        for part in spec.split(";"):
            toks = part.split()
            if not toks:
                continue
            closed = toks[-1] in ("z", "f")
            if closed:
                toks = toks[:-1]
            pts = [tuple(float(v) for v in t.split(",")) for t in toks]
            strokes.append((pts, closed))
        glyphs[ch] = strokes
    if "A" not in glyphs:
        sys.exit("make_tile: no glyphs found in " + path)
    return glyphs


def stroke_polys(pts, closed, ox, oy, unit, width):
    """Quads for each segment, square caps on open ends (as Renderer2D::emitStroke)."""
    hw = width / 2
    segs = list(zip(pts, pts[1:] + ([pts[0]] if closed else [])))
    polys = []
    for i, (a, b) in enumerate(segs):
        ax, ay = ox + a[0] * unit, oy + a[1] * unit
        bx, by = ox + b[0] * unit, oy + b[1] * unit
        dx, dy = bx - ax, by - ay
        ln = (dx * dx + dy * dy) ** 0.5 or 1.0
        dx, dy = dx / ln, dy / ln
        if not closed and i == 0:
            ax, ay = ax - dx * hw, ay - dy * hw
        if not closed and i == len(segs) - 1:
            bx, by = bx + dx * hw, by + dy * hw
        nx, ny = -dy * hw, dx * hw
        polys.append([(ax + nx, ay + ny), (bx + nx, by + ny), (bx - nx, by - ny), (ax - nx, ay - ny)])
    # Round-ish joins: a disc at every interior point.
    joins = pts if closed else pts[1:-1]
    discs = [(ox + p[0] * unit, oy + p[1] * unit, hw) for p in joins]
    return polys, discs


def draw_text(img, glyphs, text, center_x, cap_top, cap):
    """Draws `text` centred on center_x with its cap line at cap_top (pixel units)."""
    unit = cap / 8.0
    adv = cap * 0.75
    width = cap / 8.0
    outline_w = max(1.0, cap / 16.0)
    total = adv * len(text)
    x0 = center_x - total / 2
    pad = (adv - 4 * unit) / 2
    w, h = img.size
    layer = Image.new("RGBA", (w * SS, h * SS), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    for pass_width, colour in ((width + 2 * outline_w, OUTLINE), (width, HUD_GREEN)):
        for i, ch in enumerate(text):
            for pts, closed in glyphs.get(ch.upper(), []):
                polys, discs = stroke_polys(pts, closed, (x0 + i * adv + pad) * SS, cap_top * SS,
                                            unit * SS, pass_width * SS)
                for poly in polys:
                    d.polygon(poly, fill=colour)
                for cx, cy, r in discs:
                    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=colour)
        if colour == OUTLINE:
            # Merge the outline pass into one alpha so overlaps don't darken twice.
            alpha = layer.split()[3].point(lambda a: min(a, OUTLINE[3]))
            layer = Image.new("RGBA", layer.size, (0, 0, 0, 0))
            layer.putalpha(alpha)
            d = ImageDraw.Draw(layer)
    layer = layer.resize((w, h), Image.LANCZOS)
    img.alpha_composite(layer)


def top_shade(img, height_frac, alpha):
    """A soft dark band at the top so the title reads against a bright sky."""
    w, h = img.size
    band = Image.new("L", (1, h), 0)
    hb = int(h * height_frac)
    for y in range(hb):
        band.putpixel((0, y), int(alpha * (1 - y / hb) ** 1.5))
    shade = Image.new("RGBA", (w, h), (0, 10, 20, 0))
    shade.putalpha(band.resize((w, h)))
    img.alpha_composite(shade)


def make(base, size, glyphs, title, fill):
    """`fill`: the share of the width the title spans (bigger on the small tile)."""
    w, h = size
    img = base.resize((w, h), Image.LANCZOS).convert("RGBA")
    top_shade(img, 0.42, 150)
    cap = w * fill / (0.75 * len(title))
    draw_text(img, glyphs, title, w / 2, h * 0.075, cap)
    return img.convert("RGB")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--base", required=True, help="in-game render (PNG)")
    ap.add_argument("--crop", help="x0,y0,x1,y1 in base pixels (4:3), default: the whole image")
    ap.add_argument("--title", default="POCKET VECTORS")
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()
    base = Image.open(args.base).convert("RGB")
    if args.crop:
        x0, y0, x1, y1 = (int(v) for v in args.crop.split(","))
        base = base.crop((x0, y0, x1, y1))
    bw, bh = base.size
    if abs(bw * 3 - bh * 4) > 4:
        sys.exit("make_tile: the base (after the crop) must be 4:3, got %dx%d" % (bw, bh))
    glyphs = load_glyphs(FONT_SRC)
    os.makedirs(args.out_dir, exist_ok=True)
    for name, size, fill in (("screenshot.png", (640, 480), 0.68),
                             ("PocketVectors.png", (300, 225), 0.9)):
        out = os.path.join(args.out_dir, name)
        img = make(base, size, glyphs, args.title, fill)
        if size[0] <= 300:
            img = img.filter(ImageFilter.UnsharpMask(radius=1, percent=60, threshold=2))
        img.save(out, optimize=True)
        print("wrote", out, img.size)


if __name__ == "__main__":
    main()
