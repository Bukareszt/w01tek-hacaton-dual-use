// Turning COCO labels into something a dock cares about.
//
// YOLOX-nano knows the 80 COCO classes, and the dock is not in them. It has
// no helmet, no hi-vis vest and no drone, so on its own the detector says
// "person" to a dock worker and to a stranger in a hoodie alike, and says
// nothing at all about the quadcopter over the crane. This file is the layer
// that reads a little more out of the same pixels:
//
//   - a "person" is a worker if there is hi-vis orange across the chest,
//     and unknown if there is not;
//   - the classes the network reaches for when it sees a small flying thing
//     -- airplane, bird, kite -- are called a drone;
//   - and drones it misses entirely are found by looking for small dark
//     blobs surrounded by sky.
//
// Like yolox.js this is arithmetic and nothing else: it is handed the pixel
// array det_worker.js already drew for the network, so the extra work is a
// few thousand array reads and no second copy of the frame. Same coordinate
// habits too -- boxes are in frame pixels, the pixel array is the
// letterboxed input x input square with the frame top-left at scale r and
// grey padding around it, and `* r` walks one into the other.
//
// None of this is a detector with a training set behind it. It is a colour
// rule and a blob finder, tuned to the simulated harbour: the vest material
// is orange, the sky is a clean blue gradient. Read it as triage that is
// generous on purpose -- it would rather call a shadow a drone than miss
// one -- not as a classifier.

// The kinds this layer invents, and whether each one is fine to see on a
// dock. Anything else keeps its COCO label as its kind and is fine: a boat
// is a boat and a boat belongs here.
export const KIND_OK = { worker: true, unknown: false, drone: false };

// Where on a person to look for the vest: the chest band, rows 22% to 55%
// down the box and the middle 60% across. Not the whole box, because a
// standing worker's box is mostly trousers and tarmac, and not the top
// either, because that is a helmet and a face.
const CHEST_TOP = 0.22, CHEST_BOTTOM = 0.55, CHEST_INSET = 0.20;

// How much of the band has to be hi-vis. A fifth is low, and deliberately:
// the band catches arms, background and a slice of shadow, and a vest seen
// from the side is a stripe in it.
const HI_VIS = 0.20;

// What the network calls a quadcopter. It has never seen one, so it reaches
// for the nearest thing it knows that hangs in the air.
const DRONE_LABELS = new Set(["airplane", "bird", "kite"]);

// Is this pixel hi-vis?
//
// Two rules, either will do. The first is orange -- the sim's vest material
// is 1.0 0.45 0.05 and the shading walks it up and down that line, so the
// test is the shape of the colour rather than the colour: bright red
// channel, green somewhere between a third and four fifths of it, almost no
// blue. The second is the yellow-green a real vest is just as likely to be.
function isHiVis(r, g, b) {
  if (r >= 140 && g >= 0.30 * r && g <= 0.80 * r && b <= 0.55 * g) return true;
  return g >= 150 && r >= 0.6 * g && b <= 0.5 * g;
}

const luma = (r, g, b) => 0.299 * r + 0.587 * g + 0.114 * b;

// The sim's sky: a blue gradient, so blue clearly ahead of red and above
// green, and neither black nor blown out. A grey container or a white hull
// fails the first test, a shadow and the sun fail the last.
const defaultIsSky = (r, g, b) => {
  const l = luma(r, g, b);
  return b > r + 15 && b > g && l > 40 && l < 200;
};

// What fraction of a rectangle of the input canvas is hi-vis.
//
// The bounds are canvas pixels and may be fractional and may hang off the
// square -- a box at the edge of the frame does -- so they are clamped
// here rather than at every call site. An empty rectangle is 0, not NaN.
export function hiVisFraction(px, input, x0, y0, x1, y1) {
  const ax = Math.max(0, Math.min(input, Math.floor(x0)));
  const ay = Math.max(0, Math.min(input, Math.floor(y0)));
  const bx = Math.max(0, Math.min(input, Math.ceil(x1)));
  const by = Math.max(0, Math.min(input, Math.ceil(y1)));
  if (bx <= ax || by <= ay) return 0;
  let hit = 0;
  for (let y = ay; y < by; y++) {
    let o = (y * input + ax) * 4;
    for (let x = ax; x < bx; x++, o += 4) {
      if (isHiVis(px[o], px[o + 1], px[o + 2])) hit++;
    }
  }
  return hit / ((bx - ax) * (by - ay));
}

// Sort the network's boxes into kinds.
//
// Boxes come in and go out in frame pixels; `r` is only needed because the
// pixels being sampled are the letterboxed square. The label is left alone
// -- the overlay still wants to say what the network actually thought --
// and `kind` and `ok` are added beside it.
export function triage(boxes, px, input, r) {
  return boxes.map(b => {
    let kind = b.label;
    if (b.label === "person") {
      const f = hiVisFraction(px, input,
        (b.x + CHEST_INSET * b.w) * r,
        (b.y + CHEST_TOP * b.h) * r,
        (b.x + (1 - CHEST_INSET) * b.w) * r,
        (b.y + CHEST_BOTTOM * b.h) * r);
      kind = f >= HI_VIS ? "worker" : "unknown";
    } else if (DRONE_LABELS.has(b.label)) {
      kind = "drone";
    }
    const ok = Object.hasOwn(KIND_OK, kind) ? KIND_OK[kind] : true;
    return { ...b, kind, ok };
  });
}

// Do two boxes in the same space share any area at all?
function overlaps(a, b) {
  return Math.max(a.x, b.x) < Math.min(a.x + a.w, b.x + b.w) &&
         Math.max(a.y, b.y) < Math.min(a.y + a.h, b.y + b.h);
}

// Is the blob hanging in clear sky? Sampled on the same grid, one ring of
// points `margin` canvas pixels outside the box. Points off the canvas are
// not counted -- and note the grey letterbox padding is not sky, so a blob
// against the edge of the frame fails this, which is the right answer for a
// blob that is half off the picture anyway.
function ringIsSky(px, input, x0, y0, w, h, cell, margin, isSky) {
  const X0 = x0 - margin, Y0 = y0 - margin;
  const X1 = x0 + w + margin, Y1 = y0 + h + margin;
  let sky = 0, n = 0;
  const look = (x, y) => {
    if (x < 0 || y < 0 || x >= input || y >= input) return;
    const o = ((y | 0) * input + (x | 0)) * 4;
    n++;
    if (isSky(px[o], px[o + 1], px[o + 2])) sky++;
  };
  for (let x = X0; x <= X1; x += cell) { look(x, Y0); look(x, Y1); }
  for (let y = Y0; y <= Y1; y += cell) { look(X0, y); look(X1, y); }
  return n > 0 && sky / n >= 0.85;
}

// The drones the network does not see.
//
// A quadcopter a hundred metres off is a handful of dark pixels, too small
// and too unlike anything in COCO for YOLOX-nano to fire on, but against a
// blue sky it is the one dark thing up there. So: scan the top of the frame
// on a coarse grid, join the dark cells into blobs, and keep a blob that is
// drone-sized and ringed by sky.
//
// It runs on every frame in a worker on a Steam Deck, so it stays cheap: one
// pixel read per `cell` x `cell` square of the band and a flood fill over
// that small grid, no per-pixel pass over the frame. `cell` and `skyMargin`
// are canvas pixels (this is a scan of the canvas); `minSize` and `maxSize`
// are frame pixels, because that is the size a thing in the world has.
//
// Boxes the network already found are passed in as `taken` so a dark hoodie
// inside a person box is not also reported as a drone.
export function skyDrones(px, input, r, frameW, frameH, taken = [], opts = {}) {
  const band = opts.band ?? 0.40;
  const cell = opts.cell ?? 4;
  const dark = opts.dark ?? 60;
  const minSize = opts.minSize ?? 6;
  const maxSize = opts.maxSize ?? 90;
  const skyMargin = opts.skyMargin ?? 6;
  const isSky = opts.isSky ?? defaultIsSky;

  // The part of the canvas the frame's top band landed on.
  const w = Math.min(input, Math.round(frameW * r));
  const h = Math.min(input, Math.round(frameH * band * r));
  const cols = Math.floor(w / cell), rows = Math.floor(h / cell);
  if (cols < 1 || rows < 1) return [];

  // 0 not dark, 1 dark and not yet reached, 2 already in a blob.
  const grid = new Uint8Array(cols * rows);
  const half = cell >> 1;
  for (let gy = 0; gy < rows; gy++) {
    const y = gy * cell + half;
    for (let gx = 0; gx < cols; gx++) {
      const o = (y * input + gx * cell + half) * 4;
      if (luma(px[o], px[o + 1], px[o + 2]) < dark) grid[gy * cols + gx] = 1;
    }
  }

  const out = [], stack = [];
  for (let seed = 0; seed < grid.length; seed++) {
    if (grid[seed] !== 1) continue;
    // Flood fill 4-connected, keeping only the extent -- the shape of a
    // blob this coarse says nothing, its bounding box is the whole answer.
    let minX = cols, maxX = -1, minY = rows, maxY = -1;
    grid[seed] = 2;
    stack.push(seed);
    while (stack.length) {
      const i = stack.pop();
      const gx = i % cols, gy = (i - gx) / cols;
      if (gx < minX) minX = gx;
      if (gx > maxX) maxX = gx;
      if (gy < minY) minY = gy;
      if (gy > maxY) maxY = gy;
      if (gx > 0 && grid[i - 1] === 1) { grid[i - 1] = 2; stack.push(i - 1); }
      if (gx < cols - 1 && grid[i + 1] === 1) { grid[i + 1] = 2; stack.push(i + 1); }
      if (gy > 0 && grid[i - cols] === 1) { grid[i - cols] = 2; stack.push(i - cols); }
      if (gy < rows - 1 && grid[i + cols] === 1) { grid[i + cols] = 2; stack.push(i + cols); }
    }

    const x0 = minX * cell, y0 = minY * cell;
    const bw = (maxX - minX + 1) * cell, bh = (maxY - minY + 1) * cell;
    // Too small to be anything but noise, or far too big to be flying: the
    // top of a container against the sky is a dark blob as well.
    if (bw < minSize * r || bh < minSize * r) continue;
    if (bw > maxSize * r || bh > maxSize * r) continue;
    if (!ringIsSky(px, input, x0, y0, bw, bh, cell, skyMargin, isSky)) continue;

    const box = { x: x0 / r, y: y0 / r, w: bw / r, h: bh / r };
    if (taken.some(t => overlaps(box, t))) continue;
    // p is a flat 0.5: there is no score behind this, only the rules above,
    // and pretending otherwise would put a made-up number on the overlay.
    out.push({ ...box, label: "drone", p: 0.5, kind: "drone", ok: false, via: "sky" });
  }
  return out;
}
