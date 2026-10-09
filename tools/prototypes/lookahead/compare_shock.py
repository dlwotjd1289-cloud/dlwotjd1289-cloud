"""Run shock_test.py's protocol (STEP_TEST) and the revised one on the same pallets.

Pallets come from the same episodes as run_eval.py (test split, shuffle_seed 12345).
Variant overrides cand= / cap= are not supported here. Linux/WSL (fork workers);
needs pybullet, scipy, networkx, numpy, pyyaml.

    cd tools/prototypes/lookahead
    python3 compare_shock.py team "la_k3;stack_g=0.3" --episodes 6 --workers 7
"""
import argparse
import json
import multiprocessing as mp
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import shock_protocol as SP  # noqa: E402  (first: puts pac_simulation on sys.path)
import run_eval as R  # noqa: E402

PROTOCOLS = (("step", SP.STEP_TEST), ("revised", SP.REVISED))


def episode(job):
    from pac_highlevel import RulePolicy
    from pac_highlevel.trainer import run_policy
    from pac_highlevel.world import PalletizingWorld
    from virtual_data.highlevel import world_factory

    variant, i = job
    finals = []
    oc, of = PalletizingWorld._close_pallet, PalletizingWorld._finish

    def snap(self):
        if self.placed:
            finals.append([(pb, pb.pose) for pb in self.placed])

    PalletizingWorld._close_pallet = lambda self, *a, **k: (snap(self), oc(self, *a, **k))[1]
    PalletizingWorld._finish = lambda self, *a, **k: (snap(self), of(self, *a, **k))[1]
    try:
        placer, policy = R.make_planner(variant, i)
        hl = R.hl_for(R.parse_variant(variant)[2])
        world = world_factory(R.G["ds"], R.G["specs"], R.G["cand"], R.G["vcfg"], hl, shuffle_seed=12345,
                              vary_pallet=False, placer=placer)(i)
        if placer is not None:
            placer.world = world
        run_policy(world, policy or RulePolicy(hl))
    finally:
        PalletizingWorld._close_pallet, PalletizingWorld._finish = oc, of
    rows = []
    for k, layout in enumerate(finals):
        for name, proto in PROTOCOLS:
            r = SP.shock_test(layout, proto)
            r.update(variant=variant, episode=i, pallet=k, protocol=name)
            rows.append(r)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("variants", nargs="+")
    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--workers", type=int, default=7)
    ap.add_argument("--out", type=Path, default=Path("compare_shock.jsonl"))
    a = ap.parse_args()
    R.setup("test")
    jobs = [(v, i) for v in a.variants for i in range(a.episodes)]
    rows = []
    with mp.get_context("fork").Pool(a.workers, maxtasksperchild=1) as pool, a.out.open("w") as fh:
        for rs in pool.imap_unordered(episode, jobs, chunksize=1):
            for r in rs:
                fh.write(json.dumps(r, default=float) + "\n")
            rows.extend(rs)
            if rs:
                print(rs[0]["variant"], rs[0]["episode"], "done", flush=True)

    # step passed_g per episode: compare with docs/jaesung/algorithm_v3/results/raw/stack.jsonl
    print("\n[step passed_g per episode]")
    per_ep = defaultdict(list)
    for r in sorted(rows, key=lambda r: (r["variant"], r["episode"], r["pallet"])):
        if r["protocol"] == "step":
            per_ep[(r["variant"], r["episode"])].append(r["passed_g"])
    for (v, i), gs in sorted(per_ep.items()):
        print(f"  {v} ep{i}: {gs}")

    print("\nvariant | protocol | pallets | 0.3 g pass | pre-test drift | box peak / nominal (median of p90)")
    groups = defaultdict(list)
    for r in rows:
        groups[(r["variant"], r["protocol"])].append(r)
    for (v, name), rs in sorted(groups.items()):
        ok = [r for r in rs if r["passed_g"] is not None]
        n03 = sum(r["passed_g"] >= 0.3 for r in ok)
        drift = sum(r.get("pre_drift_fail", False) for r in ok)
        amp = [lv["box_peak_over_nominal_p90"] for r in ok for lv in r.get("levels", [])]
        amp_s = f"{statistics.median(amp):.2f}" if amp else "-"
        print(f"{v} | {name} | {len(ok)} (+{len(rs) - len(ok)} rebuild err) | "
              f"{100 * n03 / max(1, len(ok)):.0f}% ({n03}/{len(ok)}) | {drift} | {amp_s}")
