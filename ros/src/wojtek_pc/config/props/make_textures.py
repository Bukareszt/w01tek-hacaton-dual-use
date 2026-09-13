#!/usr/bin/env python3
"""Draw the picture textures the simulation's props and surfaces wear.

Most props in scene_sim.xml are plain coloured shapes, because a plain coloured
shape is all the detector needs to name a fire hydrant or a traffic light. Some
of them are not. A stop sign is a stop sign because it says STOP, and a clock is
a clock because it has numbers and hands; those two need a picture, and this is
where the pictures come from.

The harbour dock needs pictures for a second reason: its surfaces are large, the
camera sits 0.2 m above the quay, and a flat fill at that distance looks like
fog. So the dock adds four tiling materials -- the concrete of the quay, the
corrugated side of a shipping container, the water in the basin, and the boards
of a crate. Each is 512x512 and seamless: every pattern in them is built from
noise that wraps on a torus or from shapes cut on the tile edge, so MuJoCo can
repeat them across a whole quay without a seam showing. The container wall is
deliberately grey and nearly colourless, because the material's rgba tints it
red, blue or green; only its light and shade belong to the texture.

The results are committed next to this file, so nobody has to run it to use the
simulation. Run it when you want to change how a surface looks:

    python3 ros/src/wojtek_pc/config/props/make_textures.py

Pillow is the only requirement, and any font will do -- the letters are large
and the detector reads shapes, not typefaces. The random numbers run off fixed
seeds, so a run that changes nothing rewrites the same pixels.
"""

import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).parent
SIZE = 256
TILE = 512

# Whatever bold face the machine happens to have; the last resort is the one
# Pillow carries itself, which is why this list may end without a match.
FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
    "/Library/Fonts/Arial Bold.ttf",
)


def font(points):
    for path in FONT_CANDIDATES:
        if Path(path).exists():
            return ImageFont.truetype(path, points)
    return ImageFont.load_default(size=points)


def octagon(centre, radius, start_deg=22.5):
    return [
        (
            centre + radius * math.cos(math.radians(start_deg + i * 45)),
            centre + radius * math.sin(math.radians(start_deg + i * 45)),
        )
        for i in range(8)
    ]


def stop_sign(size=SIZE):
    """A road stop sign, drawn to the edges of a square.

    The prop is a flat square plate, so the corners around the octagon show.
    They are painted near-black, which is what makes the sign work at three
    metres: red corners turned the whole plate into one red square as the
    picture got small, and the detector stopped seeing an octagon.
    """
    img = Image.new("RGB", (size, size), (28, 28, 32))
    d = ImageDraw.Draw(img)
    d.polygon(octagon(size / 2, size / 2), fill=(196, 26, 32))
    ring = octagon(size / 2, size / 2 * 0.86)
    d.line(ring + [ring[0]], fill=(245, 245, 245), width=max(3, size // 40))
    d.text(
        (size / 2, size / 2), "STOP",
        font=font(int(size * 0.30)), fill=(250, 250, 250), anchor="mm",
    )
    img.save(HERE / "stop_sign.png")


def clock(size=SIZE):
    """A round wall clock: rim, hours 1 to 12, and hands at about ten past two.

    Like the stop sign, the square corners left over around the dial are
    painted near-black so what carries to the camera is a round face and not
    a white square.
    """
    img = Image.new("RGB", (size, size), (28, 28, 32))
    d = ImageDraw.Draw(img)
    c = size / 2
    d.ellipse([2, 2, size - 3, size - 3], fill=(235, 233, 226),
              outline=(40, 40, 45), width=max(4, size // 30))
    numerals = font(int(size * 0.11))
    for hour in range(1, 13):
        a = math.radians(hour * 30 - 90)
        d.text(
            (c + 0.78 * c * math.cos(a), c + 0.78 * c * math.sin(a)),
            str(hour), font=numerals, fill=(30, 30, 35), anchor="mm",
        )
    for length, width, degrees in ((0.42, 7, -30), (0.62, 5, 60)):
        a = math.radians(degrees)
        d.line(
            [c, c, c + length * c * math.cos(a), c + length * c * math.sin(a)],
            fill=(20, 20, 25), width=width,
        )
    d.ellipse([c - 6, c - 6, c + 6, c + 6], fill=(20, 20, 25))
    img.save(HERE / "clock.png")


# --- the tiling dock surfaces -------------------------------------------------
#
# Everything below builds a 512x512 field of numbers first and turns it into
# pixels at the end. That is the cheapest way to stay seamless: a value-noise
# lattice that wraps at its own edge cannot produce a seam, and neither can a
# sine whose period divides the tile.


def smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


def noise_field(size, cells_x, cells_y, rng, octaves=1):
    """Value noise on a torus, returned as rows of numbers in 0..1.

    The lattice indices are taken modulo the lattice, so the right edge
    interpolates back into the left one and the bottom into the top. Cells are
    counted per axis, which is how a field gets stretched: few cells across and
    many down gives horizontal streaks, and the other way round gives vertical
    ones.
    """
    field = [[0.0] * size for _ in range(size)]
    amplitude, total = 1.0, 0.0
    for octave in range(octaves):
        cx, cy = cells_x * 2 ** octave, cells_y * 2 ** octave
        lattice = [[rng.random() for _ in range(cx)] for _ in range(cy)]
        columns = []
        for x in range(size):
            fx = x * cx / size
            x0 = int(fx)
            columns.append((x0 % cx, (x0 + 1) % cx, smoothstep(fx - x0)))
        for y in range(size):
            fy = y * cy / size
            y0 = int(fy)
            ty = smoothstep(fy - y0)
            row0, row1 = lattice[y0 % cy], lattice[(y0 + 1) % cy]
            out = field[y]
            for x, (x0, x1, tx) in enumerate(columns):
                a = row0[x0] + (row0[x1] - row0[x0]) * tx
                b = row1[x0] + (row1[x1] - row1[x0]) * tx
                out[x] += amplitude * (a + (b - a) * ty)
        total += amplitude
        amplitude *= 0.5
    return [[v / total for v in row] for row in field]


def clamp(value):
    return 0 if value < 0 else 255 if value > 255 else int(value)


def wrapped(delta, size):
    """Distance along one axis on a tile that wraps, so |a - b| never lies."""
    delta = abs(delta) % size
    return min(delta, size - delta)


def save(pixels, name):
    img = Image.new("RGB", (TILE, TILE))
    img.putdata(pixels)
    img.save(HERE / name, optimize=True)


def quay_concrete(size=TILE):
    """The concrete of the quay: warm grey, stained, cut by expansion joints.

    The joints sit on the tile edges, one across and one down, so repeating the
    texture lays a grid of them over the whole quay. They are kept shallow and
    the grey is kept busy on purpose: a strong joint or a strong low-frequency
    patch turns into a checkerboard once the camera drops to 0.2 m and sees
    dozens of tiles at once, which is the failure this texture is written to
    avoid.
    """
    rng = random.Random(4711)
    coarse = noise_field(size, 3, 3, rng, octaves=2)
    grit = noise_field(size, 24, 24, rng, octaves=3)
    aggregate = noise_field(size, 96, 96, rng, octaves=2)
    blotch = noise_field(size, 5, 5, rng, octaves=2)
    stains = [
        (
            rng.randrange(size), rng.randrange(size),
            rng.uniform(0.10, 0.21) * size, rng.uniform(9.0, 21.0),
        )
        for _ in range(7)
    ]

    pixels = []
    for y in range(size):
        edge_y = min(y, size - y)
        for x in range(size):
            level = 166.0
            level += 9.0 * (coarse[y][x] - 0.5) * 2.0
            level += 13.0 * (grit[y][x] - 0.5) * 2.0
            level += 11.0 * (aggregate[y][x] - 0.5) * 2.0
            level += rng.uniform(-3.5, 3.5)

            for sx, sy, radius, depth in stains:
                dx = wrapped(x - sx, size)
                dy = wrapped(y - sy, size)
                d2 = dx * dx + dy * dy
                if d2 < radius * radius:
                    fade = (1.0 - d2 / (radius * radius)) ** 1.6
                    level -= depth * fade * (0.35 + blotch[y][x])

            edge = min(edge_y, min(x, size - x))
            width = 4.0 + 1.6 * grit[y][x]
            if edge < width:
                level -= 26.0 * (1.0 - edge / width) ** 0.7
            elif edge < width + 2.5:
                level += 4.0

            pixels.append((clamp(level * 1.02), clamp(level), clamp(level * 0.94)))
    save(pixels, "quay_concrete.png")


def container_wall(size=TILE):
    """The corrugated side of a shipping container, in grey so it can be tinted.

    Eight ribs fit across the tile, which makes their period divide the edge and
    keeps the sides seamless. The rust is a band that fades out before it
    reaches either the top or the bottom edge -- a streak running off the bottom
    would meet a clean top on the next tile and draw a line across the wall. The
    stencil is white and sits well inside the tile for the same reason.
    """
    rng = random.Random(1372)
    dirt = noise_field(size, 20, 3, rng, octaves=2)
    drips = noise_field(size, 40, 5, rng, octaves=2)
    grime = noise_field(size, 6, 6, rng, octaves=2)

    rib = size / 8.0
    ribs = []
    for x in range(size):
        u = (x % rib) / rib
        shade = 38.0 * math.cos(2.0 * math.pi * u)
        fold = min(abs(u - 0.5), 1.0 - abs(u - 0.5))
        shade -= 13.0 * math.exp(-((fold / 0.05) ** 2))
        crest = min(abs(u - 0.02), 1.0 - abs(u - 0.02))
        shade += 14.0 * math.exp(-((crest / 0.045) ** 2))
        ribs.append(shade)

    pixels = []
    for y in range(size):
        t = y / size
        band = 0.0
        if 0.50 < t < 0.97:
            band = math.sin(math.pi * (t - 0.50) / 0.47) ** 1.7
        for x in range(size):
            level = 131.0 + ribs[x]
            level += 10.0 * (grime[y][x] - 0.5) * 2.0
            level -= 9.0 * dirt[y][x]
            red = green = blue = level

            rust = band * (max(0.0, drips[y][x] - 0.40) / 0.60) ** 1.4
            if rust > 0.0:
                red = level - 10.0 * rust
                green = level - 34.0 * rust
                blue = level - 52.0 * rust
            pixels.append((clamp(red), clamp(green), clamp(blue)))

    img = Image.new("RGB", (size, size))
    img.putdata(pixels)
    d = ImageDraw.Draw(img)
    stencil = font(int(size * 0.055))
    d.text((size * 0.5, size * 0.20), "WJT 4210", font=stencil,
           fill=(224, 224, 222), anchor="mm")
    d.text((size * 0.5, size * 0.26), "22 G1", font=stencil,
           fill=(224, 224, 222), anchor="mm")
    d.rectangle([size * 0.5 - 78, size * 0.30, size * 0.5 + 78, size * 0.315],
                fill=(216, 216, 214))
    img.save(HERE / "container_wall.png", optimize=True)


def water(size=TILE):
    """The water in the basin: dark blue-green, with ripples and a few glints.

    The swell is a handful of sines whose frequencies are whole numbers of
    cycles per tile, so they close on themselves at the edge, and the chop on
    top of them is wrapping noise. The glints are cut from the crests with a
    threshold, which keeps them sparse instead of turning the whole surface
    silver.
    """
    rng = random.Random(9021)
    chop = noise_field(size, 10, 6, rng, octaves=3)
    ripple = noise_field(size, 34, 20, rng, octaves=2)
    sparkle = noise_field(size, 30, 26, rng, octaves=2)
    waves = [
        (rng.randint(1, 3), rng.randint(2, 5), rng.uniform(0, math.tau), rng.uniform(0.6, 1.0))
        for _ in range(5)
    ]

    deep = (11, 42, 49)
    crest_colour = (104, 168, 172)
    pixels = []
    for y in range(size):
        for x in range(size):
            swell, weight = 0.0, 0.0
            for fx, fy, phase, amp in waves:
                swell += amp * math.sin(
                    2.0 * math.pi * (fx * x + fy * y) / size + phase
                )
                weight += amp
            level = 0.5 + 0.5 * (swell / weight)
            level = 0.58 * level + 0.27 * chop[y][x] + 0.15 * ripple[y][x]
            shade = min(1.0, max(0.0, (level - 0.18) / 0.64)) ** 1.5

            red = deep[0] + (crest_colour[0] - deep[0]) * shade
            green = deep[1] + (crest_colour[1] - deep[1]) * shade
            blue = deep[2] + (crest_colour[2] - deep[2]) * shade

            if level > 0.68 and sparkle[y][x] > 0.55:
                glint = ((level - 0.68) / 0.32) ** 1.5 * (sparkle[y][x] - 0.55) / 0.45
                red += 165.0 * glint
                green += 160.0 * glint
                blue += 150.0 * glint
            pixels.append((clamp(red), clamp(green), clamp(blue)))
    save(pixels, "water.png")


def crate(size=TILE):
    """The face of a wooden crate: four boards, dark gaps, one diagonal brace.

    The boards are 128 pixels tall and the gaps fall on multiples of that, so
    one gap lands on the tile edge and the stack continues into the next tile.
    The brace is cut from (x + y) taken modulo the tile, which is a 45 degree
    band that leaves one edge exactly where it re-enters the opposite one: over
    a repeated wall the braces line up into long diagonals.
    """
    rng = random.Random(2856)
    grain = noise_field(size, 4, 48, rng, octaves=3)
    rough = noise_field(size, 30, 30, rng, octaves=2)
    boards = size // 4
    shifts = [rng.randrange(size) for _ in range(4)]
    tones = [rng.uniform(-11.0, 11.0) for _ in range(4)]

    base = (151, 113, 70)
    brace_base = (171, 129, 82)
    pixels = []
    for y in range(size):
        board = (y // boards) % 4
        shift = shifts[board]
        gap = min(y % boards, boards - (y % boards))
        for x in range(size):
            g = grain[y][(x + shift) % size]
            streak = -16.0 + 32.0 * g
            streak -= 14.0 * math.exp(-(((g - 0.5) / 0.045) ** 2))

            diagonal = (x + y) % size
            on_brace = abs(diagonal - size * 0.5) < 26.0
            colour = brace_base if on_brace else base
            level = tones[board] + streak + 7.0 * (rough[y][x] - 0.5) * 2.0
            if on_brace:
                level += 6.0
                if abs(diagonal - size * 0.5) > 22.0:
                    level -= 34.0
            elif gap < 3.0:
                level -= 62.0 * (1.0 - gap / 3.0) ** 0.6

            pixels.append((
                clamp(colour[0] + level),
                clamp(colour[1] + level * 0.85),
                clamp(colour[2] + level * 0.62),
            ))
    save(pixels, "crate.png")


if __name__ == "__main__":
    stop_sign()
    clock()
    quay_concrete()
    container_wall()
    water()
    crate()
    print(
        "wrote stop_sign.png, clock.png, quay_concrete.png, container_wall.png, "
        f"water.png and crate.png in {HERE}"
    )
