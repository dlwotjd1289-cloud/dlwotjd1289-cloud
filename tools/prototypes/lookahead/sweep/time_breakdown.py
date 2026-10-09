"""Time breakdown of one look-ahead decision loop (base la_k3), first N decisions of 3 episodes."""
import cProfile, pstats, sys, time, io
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_eval as R
from virtual_data.highlevel import world_factory
R.setup("test")
N = 25
pr = cProfile.Profile(); n_dec = 0; t0 = time.perf_counter()
for ep in range(3):
    placer, policy = R.make_planner("la_k3", ep)
    w = world_factory(R.G["ds"], R.G["specs"], R.G["cand"], R.G["vcfg"], R.G["hl"], shuffle_seed=12345, vary_pallet=False, placer=placer)(ep)
    placer.world = w
    pr.enable()
    k = 0
    while not w.done and k < N:
        w.step(policy(w)); k += 1
    pr.disable(); n_dec += k
wall = time.perf_counter() - t0
st = pstats.Stats(pr)
want = {
    "후보 생성 + 하드마스크 (candidate_set)": "candidate_set",
    "후보별 제약 재검사 (validate_constraints)": "validate_constraints",
    "점수 계산 (Scorer.cost)": "cost",
    "미래 시나리오 평가 (_rollout)": "_rollout",
    "상위 세계 옵션 평가 (world.options)": "options",
    "결정 전체 (Planner.decide)": "decide",
}
tot = {}
for (fn, line, name), (cc, nc, tt, ct, callers) in st.stats.items():
    for label, key in want.items():
        if name == key:
            tot[label] = tot.get(label, 0.0) + ct
print(f"decisions={n_dec}, wall={wall:.1f}s, per decision={1000*wall/n_dec:.0f} ms (cProfile overhead included)")
for label in want:
    v = tot.get(label, 0.0)
    print(f"  {label:42s} {1000*v/n_dec:8.0f} ms/결정  ({v/wall:5.1%} of wall)")
