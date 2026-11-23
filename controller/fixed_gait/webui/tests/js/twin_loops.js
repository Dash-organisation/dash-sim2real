/* Harness for the digital twin's loop solve (static/twin3d.js) on the published homing MJCF
 * (static/twin/dash01_homing_v8.xml). Run by tests/test_twin_loops_js.py, or directly:
 *
 *     node controller/fixed_gait/webui/tests/js/twin_loops.js
 *
 * Node has no DOMParser, so a minimal one is shimmed below: enough XML for an MJCF (elements,
 * attributes, comments, the prolog) and the four selectors twin3d.js uses. */
"use strict";
const fs = require("fs");
const path = require("path");

const WEBUI = path.join(__dirname, "..", "..");
const XML = path.join(WEBUI, "static", "twin", "dash01_homing_v8.xml");

function parseXML(text) {
  const root = { tagName: "#document", children: [], attrs: {}, parent: null };
  let cur = root;
  const re = /<!--[\s\S]*?-->|<\?[\s\S]*?\?>|<(\/?)([A-Za-z_][\w.-]*)((?:\s+[\w:.-]+\s*=\s*"[^"]*")*)\s*(\/?)>/g;
  let m;
  while ((m = re.exec(text))) {
    if (!m[2]) continue;                                   // comment / prolog
    if (m[1]) { cur = cur.parent; continue; }
    const attrs = {};
    for (const a of m[3].matchAll(/([\w:.-]+)\s*=\s*"([^"]*)"/g)) attrs[a[1]] = a[2];
    const el = { tagName: m[2], attrs, children: [], parent: cur };
    cur.children.push(el);
    if (!m[4]) cur = el;
  }
  const all = (n, out = []) => { for (const c of n.children) { out.push(c); all(c, out); } return out; };
  const match = (el, sel) => {                             // "tag" or "parent > tag"
    const p = sel.split(">").map((x) => x.trim());
    return p.length === 1 ? el.tagName === p[0] : el.tagName === p[1] && el.parent.tagName === p[0];
  };
  const wrap = (n) => {
    n.getAttribute = (k) => (k in n.attrs ? n.attrs[k] : null);
    n.querySelectorAll = (sel) => all(n).filter((e) => match(e, sel));
    n.querySelector = (sel) => n.querySelectorAll(sel)[0] || null;
    n.children.forEach(wrap);
    return n;
  };
  return wrap(root);
}

global.window = {};
global.DOMParser = class { parseFromString(t) { return parseXML(t); } };
// twin3d.js only needs these at class-instantiation time, which this harness never does
global.requestAnimationFrame = () => {};
require(path.join(WEBUI, "static", "twin3d.js"));
const K = window.Twin3DKin;

let fails = 0;
const ok = (cond, msg) => { if (!cond) { fails++; console.log("FAIL " + msg); } else console.log("ok   " + msg); };

const model = K.parseMJCF(fs.readFileSync(XML, "utf8"));
const MOTOR = [[/hip_left/i, "left.abd"], [/cam_left/i, "left.cam"], [/thigh_left/i, "left.thigh"],
  [/hip_right/i, "right.abd"], [/cam_right/i, "right.cam"], [/thigh_right/i, "right.thigh"]];
for (const b of model.all) {                               // the same tagging Twin3D.load does
  const hit = MOTOR.find(([re]) => re.test(b.name));
  b.motor = hit && b.joint ? hit[1] : null;
  b.side = (/left/i.test(b.name) ? "left" : /right/i.test(b.name) ? "right" : null) ||
    (b.parent && b.parent.side) || null;
}
model.baseR = [1, 0, 0, 0, 1, 0, 0, 0, 1];
K.fk(model.roots);
model.loops = K.findLoops(model);
model.byMotor = {};
for (const b of model.all) if (b.motor) model.byMotor[b.motor] = b;

ok(model.loops.length === 4, "four loops found: " + model.loops.map((L) => L.name));
ok(Object.keys(model.byMotor).length === 6, "six motors mapped: " + Object.keys(model.byMotor));
for (const side of ["left", "right"]) {
  const mine = model.loops.filter((L) => L.side === side).map((L) => L.name);
  ok(mine.length === 2 && /knee/.test(mine[0]) && /achilles/.test(mine[1]),
    `${side}: knee solved before achilles (${mine})`);
}

const pose = (q) => {
  for (const [n, b] of Object.entries(model.byMotor)) b.q = q[n] || 0;
  K.solveLoops(model);
  return model.loops.map((L) => L.resid);
};
const r0 = pose({});
ok(Math.max(...r0) < 1e-6, `homing pose (all 0) closes every loop: max ${(Math.max(...r0) * 1e3).toFixed(4)} mm`);
ok(model.loops.every((L) => Math.abs(L.rod.q) < 1e-6 && Math.abs(L.other.q) < 1e-6),
  "homing pose leaves every passive joint at 0");

const d = Math.PI / 180;
// The homing pose is near the knee four-bar's full stretch: the thigh alone swung the other way
// (q -8 deg left) runs out of linkage after ~4 deg. Then the twin draws the closest pose and the
// residual reports the gap -- which must be the KNEE loop, the achilles loop still closing.
pose({ "left.thigh": -8 * d });
const kneeL = model.loops.find((L) => L.side === "left" && /knee/.test(L.name));
const achL = model.loops.find((L) => L.side === "left" && /achilles/.test(L.name));
ok(kneeL.resid > 1e-3 && achL.resid < 1e-6,
  `past the knee's stretch the knee gap is reported (${(kneeL.resid * 1e3).toFixed(2)} mm), achilles still closes`);
for (const q of [{ "left.cam": 10 * d, "right.cam": -10 * d }, { "left.thigh": 8 * d, "right.thigh": -8 * d },
  { "left.cam": 20 * d, "left.thigh": -5 * d, "left.abd": 5 * d, "right.cam": -20 * d, "right.thigh": 5 * d }]) {
  const r = pose(q);
  ok(Math.max(...r) < 1e-6, `${JSON.stringify(Object.fromEntries(Object.entries(q).map(([k, v]) =>
    [k, +(v / d).toFixed(1)])))} closes: max ${(Math.max(...r) * 1e3).toFixed(4)} mm`);
  ok(model.loops.every((L) => Math.abs(L.rod.q) > 1e-4), "  ... and the passive joints moved");
}

if (fails) { console.log(fails + " twin loop check(s) failed"); process.exit(1); }
console.log("all twin loop checks passed");
