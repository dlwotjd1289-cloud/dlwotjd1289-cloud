"""Value of ordering: same boxes, arrival order sorted offline (NOT allowed online).
Bound for what buffer/lookahead could recover. Uses team world + RulePolicy + DBLF."""
import json, multiprocessing as mp, statistics, sys
from dataclasses import replace
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import tune
from pac_highlevel import RulePolicy
from pac_highlevel.trainer import run_policy
from virtual_data.highlevel import world_factory

def run(job):
    key, i = job
    specs = []
    for s in tune.G["specs"]:
        arr = list(s.arrivals)
        if key == "weight_desc":
            arr.sort(key=lambda b: -b.weight_kg)
        elif key == "weight_desc_height":
            arr.sort(key=lambda b: (-round(b.weight_kg / 5), -b.size.z))
        specs.append(replace(s, arrivals=tuple(arr)))
    make = world_factory(tune.G["ds"], specs, tune.G["cand"], tune.G["vcfg"], tune.G["hl"], shuffle_seed=12345)
    out = run_policy(make(i), RulePolicy(tune.G["hl"])); out.pop("pallets", None)
    return key, out

if __name__ == "__main__":
    cand = Path(sys.argv[1]); tag = sys.argv[2]
    tune.setup(cand, "test")
    eps = range(len(tune.G["specs"]) * 3)
    with mp.get_context("fork").Pool(8) as pool:
        rows = pool.map(run, [(k, i) for k in ("random", "weight_desc", "weight_desc_height") for i in eps])
    res = {}
    for k in ("random", "weight_desc", "weight_desc_height"):
        r = [o for kk, o in rows if kk == k]
        res[k] = {m: statistics.fmean(x[m] for x in r) for m in ("pallets_used", "pallet_equivalents", "fill_per_pallet_used")}
        print(tag, k, {m: round(v, 3) for m, v in res[k].items()})
    Path(f"order_bound_{tag}.json").write_text(json.dumps(res, indent=1))
