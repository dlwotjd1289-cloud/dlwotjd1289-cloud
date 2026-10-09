// Run the simulator core (no DOM) under node:
//   node viewer/donghan/check_core.js [order] [seed] [repeat] [touchingP] [intervalS]
// Runs the whole line twice (inlet camera on / off) and prints verdict counts and flow metrics.
const fs = require("fs");
const path = require("path");

const html = fs.readFileSync(path.join(__dirname, "perception_sim.html"), "utf8");
const src = html.match(/<script id="core">([\s\S]*?)<\/script>/)[1];
const mod = { exports: {} };
new Function("module", src)(mod);
const CORE = mod.exports;

const [order = "demo_original10", seed = "20261009", repeat = "3", touchingP, intervalS] = process.argv.slice(2);
const extra = { ...(touchingP ? { touchingP: +touchingP } : {}), ...(intervalS ? { intervalS: +intervalS } : {}) };
const faultP = Object.fromEntries(CORE.FAULTS.map(f => [f.key, f.p]));

const L = CORE.LAYOUT, issues = CORE.checkLayout();
console.log(`layout: buffer ${JSON.stringify(L.buffer.bays)}, NG ${JSON.stringify(L.ng.center)} -> ${issues.length ? issues.join("; ") : "reach / overlap OK"}`);

for (const inletCamera of [true, false]) {
  const t0 = Date.now();
  const sim = new CORE.Sim({ order, seed: +seed, repeat: +repeat, faultP, flow: { inletCamera, ...extra } }).run();
  const s = sim.summary();
  const table = {};
  for (const r of sim.results) {
    const key = r.fb.fault === "none" && r.fb.touching ? "touching" : r.fb.fault;
    const kind = r.verdict.kind + (r.verdict.route === "PLAN" && r.verdict.uncertain ? " δ" : "");
    (table[key] ??= {})[kind] = (table[key][kind] || 0) + 1;
  }
  console.log(`\n== inlet camera ${inletCamera ? "ON" : "OFF"} · ${order} x${repeat} seed ${seed} · ${((Date.now() - t0) / 1000).toFixed(1)} s wall`);
  console.log(`done=${sim.done} t=${s.t.toFixed(0)} s, results ${sim.results.length}/${s.boxes}, placed ${s.placed}, held ${s.held}, pallets ${s.pallets} (changes ${s.palletChange}), fill ${(s.fill * 100).toFixed(0)}%`);
  console.log(`wrong ${s.wrong}, weight error >5% ${s.weightErr}, combined weighings ${s.combinedWeigh}, gapping ${s.gapping}, inlet labels ${s.inletLabel}`);
  console.log(`buffer put/get ${s.bufferPut}/${s.bufferGet}, stop classify ${(s.stopClassify * 100).toFixed(1)}%, plan ${(s.stopPlan * 100).toFixed(1)}%, robot ${(s.robotUtil * 100).toFixed(0)}%, ${s.throughput.toFixed(2)} boxes/min`);
  for (const [f, kinds] of Object.entries(table)) console.log("  " + f.padEnd(12), JSON.stringify(kinds));
  const leftover = sim.results.filter(r => r.disposition === "판정됨" || r.disposition.startsWith("버퍼"));
  if (leftover.length) console.log("  NOT FINISHED:", leftover.map(r => r.fb.box_id + ":" + r.disposition).join(", "));
}
