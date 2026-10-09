"""Structural coupling of final pallets: for every box above the deck, count the
boxes it rests on. bonded = rests on >= 2 boxes (load shared / bricks interlock);
column = rests on exactly one box (pure stack, no load path to neighbours)."""
import json, sys, statistics, multiprocessing as mp
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import tune, physics_check as pc
from pac_candidates.geometry import rotated_dims

def supporters(box, pose, layout):
    dx, dy, _ = rotated_dims(box.size, pose.yaw)
    out = []
    for b, p in layout:
        if b is box: continue
        bx, by, bz = rotated_dims(b.size, p.yaw)
        if abs(p.z + bz - pose.z) > 0.004: continue
        ox = min(pose.x + dx, p.x + bx) - max(pose.x, p.x); oy = min(pose.y + dy, p.y + by) - max(pose.y, p.y)
        if ox > 0.005 and oy > 0.005: out.append(ox * oy / (dx * dy))
    return out

def job(a):
    w, i = a
    _, finals = pc.record_episode(w, "rule" if w is None else "wave", i)
    up = bonded = column = 0
    for layout in finals:
        for b, p in layout:
            if p.z < 1e-6: continue
            s = supporters(b, p, layout); up += 1
            bonded += len(s) >= 2; column += len(s) == 1
    return up, bonded, column

C = Path.home() / "AHEAD/audit_20261009/07_ai_role/sweep/configs"
res = {}
for tag, cand, wfile in (("team_perbox", "perbox.yaml", None), ("wave_lbcp06", "perbox_lbcp06.yaml", "tuned_lbcp06_wave.json")):
    tune.setup(C / cand, "test")
    w = None if wfile is None else json.loads(Path(wfile).read_text())["mean"]
    with mp.get_context("fork").Pool(9) as pool:
        rows = pool.map(job, [(w, i) for i in range(len(tune.G["specs"]))])
    up = sum(r[0] for r in rows); bo = sum(r[1] for r in rows); co = sum(r[2] for r in rows)
    res[tag] = {"boxes_above_deck": up, "bonded_ge2_supporters": bo / up, "single_supporter_column": co / up}
    print(tag, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in res[tag].items()})
Path("interlock_check.json").write_text(json.dumps(res, indent=1))
