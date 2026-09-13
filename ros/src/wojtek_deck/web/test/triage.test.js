// node --test web/test   (no browser needed; triage.js is plain arithmetic)
//
// triage.js reads pixels, so the tests paint them. Each one builds the same
// thing det_worker.js hands the module -- a letterboxed input x input RGBA
// canvas -- out of flat rectangles, with the colours the simulated dock
// actually uses: the vest material 1.0 0.45 0.05, the blue sky, the 114 grey
// of the letterbox padding.
//
// The numbers in these tests are worked out on paper, because the failures
// that matter here are quiet ones. A chest band measured over the whole box
// instead of the chest still calls most workers workers, it just also calls
// a man in a hoodie standing on an orange pallet one. A blob finder that
// forgets the letterbox scale still finds drones, a few metres from where
// they are.
import { test } from "node:test";
import assert from "node:assert/strict";
import { KIND_OK, hiVisFraction, skyDrones, triage } from "../triage.js";

const ORANGE = [255, 115, 13];        // the vest, lit: 1.0 0.45 0.05
const SHADED = [180, 81, 9];          // the same vest on its dark side
const LIME = [190, 220, 40];          // the yellow-green a real vest can be
const NAVY = [20, 24, 48];            // a dark hoodie
const GREY = [114, 114, 114];         // cfg.pad, the letterbox padding
const SKY = [120, 160, 210];          // the sim's blue sky
const DRONE = [20, 20, 25];           // a quadcopter against it

// A flat input x input RGBA canvas, and a rectangle painted onto one.
function canvas(input, [r, g, b]) {
  const px = new Uint8ClampedArray(input * input * 4);
  for (let i = 0; i < input * input; i++) {
    px[i * 4] = r; px[i * 4 + 1] = g; px[i * 4 + 2] = b; px[i * 4 + 3] = 255;
  }
  return px;
}
function rect(px, input, x, y, w, h, [r, g, b]) {
  for (let j = y; j < y + h; j++) {
    for (let i = x; i < x + w; i++) {
      const o = (j * input + i) * 4;
      px[o] = r; px[o + 1] = g; px[o + 2] = b; px[o + 3] = 255;
    }
  }
  return px;
}

test("hi-vis is the fraction of the rectangle, and grey is not hi-vis", () => {
  const px = rect(canvas(64, GREY), 64, 10, 10, 20, 20, ORANGE);
  assert.equal(hiVisFraction(px, 64, 10, 10, 30, 30), 1);   // the block
  assert.equal(hiVisFraction(px, 64, 40, 40, 60, 60), 0);   // padding only
  // Half in, half out: 20 of the 40 columns are orange.
  assert.equal(hiVisFraction(px, 64, 10, 10, 50, 30), 0.5);
});

test("hi-vis takes shaded orange and yellow-green too", () => {
  for (const c of [ORANGE, SHADED, LIME]) {
    const px = rect(canvas(32, GREY), 32, 4, 4, 8, 8, c);
    assert.equal(hiVisFraction(px, 32, 4, 4, 12, 12), 1, `${c}`);
  }
  // Navy is not, and neither is the sky -- blue fails both rules.
  for (const c of [NAVY, SKY]) {
    const px = rect(canvas(32, GREY), 32, 4, 4, 8, 8, c);
    assert.equal(hiVisFraction(px, 32, 4, 4, 12, 12), 0, `${c}`);
  }
});

test("a rectangle off the edge of the canvas is clamped, not read", () => {
  const px = rect(canvas(32, GREY), 32, 0, 0, 8, 8, ORANGE);
  // Asking past the corner still measures the 8x8 that is there.
  assert.equal(hiVisFraction(px, 32, -20, -20, 8, 8), 1);
  assert.equal(hiVisFraction(px, 32, 40, 40, 60, 60), 0);   // nothing at all
  assert.equal(hiVisFraction(px, 32, 10, 10, 10, 10), 0);   // empty
});

test("a person with an orange chest is a worker, and is fine to see", () => {
  // The box is 60x200 at (100, 100), r = 1, so the chest band is
  // x 112..148 (the middle 60%) and y 144..210 (rows 22%..55%).
  const px = rect(canvas(416, GREY), 416, 105, 140, 50, 75, ORANGE);
  const [b] = triage([{ x: 100, y: 100, w: 60, h: 200, label: "person", p: .8 }],
    px, 416, 1);
  assert.equal(b.kind, "worker");
  assert.equal(b.ok, true);
});

test("a person in a dark hoodie is unknown, and is not fine to see", () => {
  const px = rect(canvas(416, GREY), 416, 105, 140, 50, 75, NAVY);
  const [b] = triage([{ x: 100, y: 100, w: 60, h: 200, label: "person", p: .8 }],
    px, 416, 1);
  assert.equal(b.kind, "unknown");
  assert.equal(b.ok, false);
});

test("only the chest counts: an orange hat over a dark chest is unknown", () => {
  // Orange across the top 15% of the box, above the band, and nothing in
  // the band itself. A rule that measured the whole box would say worker.
  const px = rect(canvas(416, GREY), 416, 100, 100, 60, 30, ORANGE);
  const [b] = triage([{ x: 100, y: 100, w: 60, h: 200, label: "person", p: .8 }],
    px, 416, 1);
  assert.equal(b.kind, "unknown");
});

test("a stripe of vest across the band is enough", () => {
  // 20 of the 66 rows of the band, so 0.30 of it: over the 0.20 the rule
  // asks for, which is the point -- a vest seen from the side is a stripe.
  const px = rect(canvas(416, GREY), 416, 112, 160, 36, 20, ORANGE);
  const [b] = triage([{ x: 100, y: 100, w: 60, h: 200, label: "person", p: .8 }],
    px, 416, 1);
  assert.equal(b.kind, "worker");
});

test("the flying classes become drones and the rest keep their label", () => {
  const boxes = [
    { x: 10, y: 10, w: 20, h: 20, label: "kite", p: .4 },
    { x: 40, y: 10, w: 20, h: 20, label: "airplane", p: .5 },
    { x: 70, y: 10, w: 20, h: 20, label: "bird", p: .6 },
    { x: 10, y: 60, w: 90, h: 40, label: "boat", p: .7 },
    { x: 10, y: 60, w: 90, h: 40, label: "truck", p: .7 },
  ];
  const out = triage(boxes, canvas(416, SKY), 416, 1);
  assert.deepEqual(out.map(b => b.kind),
    ["drone", "drone", "drone", "boat", "truck"]);
  assert.deepEqual(out.map(b => b.ok), [false, false, false, true, true]);
});

test("triage leaves the box it was given alone", () => {
  const box = { x: 100, y: 100, w: 60, h: 200, label: "person", p: .8 };
  const before = { ...box };
  const [b] = triage([box], canvas(416, GREY), 416, 1);
  assert.deepEqual(box, before);                 // not written through
  assert.notEqual(b, box);                       // a new object
  assert.equal(b.label, "person");               // the label survives
  assert.equal(b.p, .8);
  assert.deepEqual([b.x, b.y, b.w, b.h], [100, 100, 60, 200]);
});

test("the chest band follows the letterbox scale", () => {
  // A 640x360 frame in a 416 square is r = 0.65, so the band of the same
  // box lands at canvas x 72.8..96.2, y 93.6..136.5. Painting the vest
  // there and nowhere else: a triage that forgot r would look at
  // x 112..148, y 144..210 instead and find grey.
  const px = rect(canvas(416, GREY), 416, 70, 90, 30, 50, ORANGE);
  const box = { x: 100, y: 100, w: 60, h: 200, label: "person", p: .8 };
  assert.equal(triage([box], px, 416, .65)[0].kind, "worker");
  assert.equal(triage([box], px, 416, 1)[0].kind, "unknown");
});

test("KIND_OK says which invented kinds belong on a dock", () => {
  assert.deepEqual(KIND_OK, { worker: true, unknown: false, drone: false });
});

// One quadcopter in the sky: a 20x20 cross at (300, 40), arms 6 px thick,
// the kind of thing YOLOX-nano does not fire on at all.
function withCross(input = 416) {
  const px = canvas(input, SKY);
  rect(px, input, 300, 47, 20, 6, DRONE);        // the horizontal arm
  rect(px, input, 307, 40, 6, 20, DRONE);        // the vertical one
  return px;
}

test("a dark cross in clear sky is one drone box around it", () => {
  const out = skyDrones(withCross(), 416, 1, 416, 416);
  assert.equal(out.length, 1);
  const [d] = out;
  // Within one cell of the cross, which is all a 4 px grid can promise.
  for (const [got, want] of [[d.x, 300], [d.y, 40], [d.w, 20], [d.h, 20]]) {
    assert.ok(Math.abs(got - want) <= 4, `${got} is not within 4 of ${want}`);
  }
  assert.equal(d.label, "drone");
  assert.equal(d.kind, "drone");
  assert.equal(d.ok, false);
  assert.equal(d.p, 0.5);
  assert.equal(d.via, "sky");
});

test("a dark blob against something that is not sky is not a drone", () => {
  // The same size of blob, but with a grey wall just above it -- a gantry,
  // a container stack, the hull of a ship. The ring around it is no longer
  // sky, so the blob is a dark thing on a dark thing, not a drone.
  const px = rect(canvas(416, SKY), 416, 80, 30, 60, 28, GREY);
  rect(px, 416, 100, 60, 20, 20, DRONE);
  assert.deepEqual(skyDrones(px, 416, 1, 416, 416), []);
  // and the same blob with the sky put back is found
  assert.equal(skyDrones(rect(canvas(416, SKY), 416, 100, 60, 20, 20, DRONE),
    416, 1, 416, 416).length, 1);
});

test("the dark top of a container is too big to be a drone", () => {
  const px = rect(canvas(416, SKY), 416, 50, 20, 200, 100, DRONE);
  assert.deepEqual(skyDrones(px, 416, 1, 416, 416), []);
});

test("a blob the network already has a box on is skipped", () => {
  const taken = [{ x: 295, y: 35, w: 40, h: 40, label: "bird", p: .4 }];
  assert.deepEqual(skyDrones(withCross(), 416, 1, 416, 416, taken), []);
  // a box somewhere else does not hide it
  const elsewhere = [{ x: 10, y: 10, w: 40, h: 40, label: "person", p: .9 }];
  assert.equal(skyDrones(withCross(), 416, 1, 416, 416, elsewhere).length, 1);
});

test("only the top of the frame is scanned", () => {
  // The same cross, 300 px down a 416 px frame: below the 0.40 band, so it
  // is a dark thing at ground level and none of this layer's business.
  const px = canvas(416, SKY);
  rect(px, 416, 300, 307, 20, 6, DRONE);
  rect(px, 416, 307, 300, 6, 20, DRONE);
  assert.deepEqual(skyDrones(px, 416, 1, 416, 416), []);
  // raising the band to the whole frame finds it
  assert.equal(skyDrones(px, 416, 1, 416, 416, [], { band: 1 }).length, 1);
});

test("a speck smaller than minSize is noise", () => {
  const px = rect(canvas(416, SKY), 416, 200, 50, 4, 4, DRONE);
  assert.deepEqual(skyDrones(px, 416, 1, 416, 416), []);
  // ...unless you ask for specks
  assert.equal(skyDrones(px, 416, 1, 416, 416, [], { minSize: 2 }).length, 1);
});

test("drone boxes come back in frame pixels", () => {
  // The cross drawn where a 0.65 letterbox puts frame (400, 20): the canvas
  // blob is at 260, 13 and the answer has to be the frame's own numbers.
  const px = canvas(416, SKY);
  rect(px, 416, 260, 20, 20, 6, DRONE);
  rect(px, 416, 267, 13, 6, 20, DRONE);
  const [d] = skyDrones(px, 416, .65, 640, 360, [], { minSize: 4 });
  assert.ok(Math.abs(d.x - 400) <= 8, `${d.x} is not within 8 of 400`);
  assert.ok(Math.abs(d.y - 20) <= 8, `${d.y} is not within 8 of 20`);
  assert.ok(d.w > 20 && d.w < 40, `${d.w} is not a frame-sized width`);
});
