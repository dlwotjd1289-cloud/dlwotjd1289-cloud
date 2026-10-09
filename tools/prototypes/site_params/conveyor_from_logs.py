"""Conveyor timing measured from existing Gazebo V4.3 auto-scale logs (no new simulation).
Per run: inlet->scale travel, scale settle (stop -> measure), scale->pick travel and speed,
overshoot of the stop position past the commanded stop line."""
import re, glob, statistics, json
from pathlib import Path
import os
L = Path(os.environ.get("SCAFFOLD_LOGS", str(Path.home() / "AHEAD/pac2026_hdr50_proxy_scaffold/logs")))  
files = sorted(glob.glob(str(L / "v43_e2e/final*.log"))) + sorted(glob.glob(str(L / "v44_generator_cycle/*/box_*_v43.log")))
ts = lambda line: float(re.search(r"\[(\d+\.\d+)\]", line).group(1))
rows = []
for f in files:
    t = Path(f).read_text(errors="replace").splitlines()
    ev = {}
    for line in t:
        if "start rollers" in line: ev.setdefault("start", ts(line))
        m = re.search(r"\[SETTLING\] x=(-?\d+\.\d+)", line)
        if m: ev["settle_t"], ev["settle_x"] = ts(line), float(m.group(1))
        if "[TO_PICK] WEIGHED" in line: ev["weighed_t"] = ts(line)
        m = re.search(r"\[DONE\] PICK zone reached at x=(-?\d+\.\d+)", line)
        if m: ev["done_t"], ev["done_x"] = ts(line), float(m.group(1))
    if {"start", "settle_t", "weighed_t", "done_t"} <= ev.keys():
        rows.append({"file": Path(f).parent.name + "/" + Path(f).name,
                     "inlet_to_scale_s": ev["settle_t"] - ev["start"],
                     "scale_dwell_s": ev["weighed_t"] - ev["settle_t"],
                     "scale_to_pick_s": ev["done_t"] - ev["weighed_t"],
                     "speed_m_s": (ev["done_x"] - ev["settle_x"]) / (ev["done_t"] - ev["weighed_t"]),
                     "settle_x": ev["settle_x"], "done_x": ev["done_x"]})
def s(k): 
    v = [r[k] for r in rows]; return {"n": len(v), "median": round(statistics.median(v), 3), "min": round(min(v), 3), "max": round(max(v), 3)}
out = {k: s(k) for k in ("inlet_to_scale_s", "scale_dwell_s", "scale_to_pick_s", "speed_m_s")}
# 0.40 m demo box runs only: stop overshoot past the configured stop lines (scale -3.86, pick -1.08)
demo = [r for r in rows if r["file"].startswith("v43_e2e")]
if demo:
    out["overshoot_scale_mm_demo"] = [round(1000 * (r["settle_x"] - (-3.86)), 1) for r in demo]
    out["overshoot_pick_mm_demo"] = [round(1000 * (r["done_x"] - (-1.08)), 1) for r in demo]
print(json.dumps(out, indent=1, ensure_ascii=False))
Path(__file__).with_name("conveyor_from_logs.json").write_text(json.dumps({"summary": out, "rows": rows}, indent=1))
