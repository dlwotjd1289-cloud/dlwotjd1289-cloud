"""Paired evaluation of a parameter set on generator scenarios.

Every configuration plays the SAME box streams (same noise, same pallet
sizes) with the Rule policy, so differences against the baseline are
paired. Reported: mean used pallets (pallet equivalents, lower is better),
fill, robot time, NG, safety issues (must stay 0) and the paired comparison
with the baseline (better / equal / worse, sign-test p, 95 % interval).
"""

from math import comb
import json
import multiprocessing as mp
import statistics

from pac_highlevel import RulePolicy, run_policy
from virtual_data.highlevel import split_ids, world_factory

from .space import apply_params

_JOB = None


def _episode(i):
    factory, hl = _JOB
    out = run_policy(factory(i), RulePolicy(hl))
    return {k: out[k] for k in ("pallet_equivalents", "fill_per_pallet_used", "time_s", "ng", "safety_issues")}


def sign_test(better, worse):
    n = better + worse
    if n == 0:
        return 1.0
    k = min(better, worse)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)


class Evaluator:
    def __init__(self, dataset, cand_config, vcfg, hl_config, split="val", passes=1, workers=4, seed=4242):
        self.dataset, self.vcfg = dataset, vcfg
        self.base_cand, self.base_hl = cand_config, hl_config
        self.specs = split_ids(dataset, split)
        self.split, self.passes, self.workers, self.seed = split, passes, workers, seed
        self.cache = {}
        self.baseline = self._run({})

    @property
    def episodes(self):
        return len(self.specs) * self.passes

    def _run(self, params):
        key = json.dumps(params, sort_keys=True)
        if key in self.cache:
            return self.cache[key]
        global _JOB
        cand, hl = apply_params(self.base_cand, self.base_hl, params)
        _JOB = (world_factory(self.dataset, self.specs, cand, self.vcfg, hl, shuffle_seed=self.seed), hl)
        try:
            if self.workers > 1:
                with mp.get_context("fork").Pool(self.workers) as pool:
                    rows = pool.map(_episode, range(self.episodes), chunksize=1)
            else:
                rows = [_episode(i) for i in range(self.episodes)]
        finally:
            _JOB = None
        self.cache[key] = rows
        return rows

    def evaluate(self, params):
        rows = self._run(params)
        base = self.baseline
        d = [r["pallet_equivalents"] - b["pallet_equivalents"] for r, b in zip(rows, base)]
        better = sum(x < -1e-9 for x in d)
        worse = sum(x > 1e-9 for x in d)
        mean = statistics.fmean(d)
        se = statistics.stdev(d) / len(d) ** 0.5 if len(d) > 1 else 0.0
        m = lambda k: statistics.fmean(r[k] for r in rows)  # noqa: E731
        return {
            "episodes": len(rows),
            "pallets_mean": round(m("pallet_equivalents"), 4),
            "fill_mean": round(m("fill_per_pallet_used"), 4),
            "robot_time_s_mean": round(m("time_s"), 1),
            "ng_mean": round(m("ng"), 3),
            "safety_issues": int(sum(r["safety_issues"] for r in rows)),
            "vs_baseline": {
                "pallets_diff_mean": round(mean, 4),
                "ci95": [round(mean - 1.96 * se, 4), round(mean + 1.96 * se, 4)],
                "better": better, "equal": len(d) - better - worse, "worse": worse,
                "sign_test_p": round(sign_test(better, worse), 4),
            },
        }
