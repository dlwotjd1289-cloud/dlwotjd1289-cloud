"""Stage-2 confirmation analysis. Decision rules were fixed BEFORE looking at the stage-2 results
(2026-10-09 19:40, see STAGE2_RULES below).

STAGE2_RULES
  R1 change a default only if the paired sign test of J(c_p=2000 s) per episode against the base
     (la_k3) gives p < 0.05 in favour of the variant AND its 0.3 g pass rate is not more than
     5 %p below the base.
  R2 compute-only change (horizon=3): adopt if mean J is within +1 % of the base and the 0.3 g
     pass rate is within 5 %p (non-inferiority), because it cuts decision time.
  R3 the selected setting must beat the team baseline on the unseen dataset (seed 777).
"""
import json, math, random, statistics, sys
from collections import defaultdict
from pathlib import Path

def load(p):
    return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]

def sign_p(w, l):
    n = w + l
    return 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, i) for i in range(min(w, l) + 1)) / 2 ** n)

def boot_ci(xs, n=2000, seed=0):
    rng = random.Random(seed); m = len(xs)
    means = sorted(statistics.fmean(rng.choice(xs) for _ in range(m)) for _ in range(n))
    return means[int(0.025 * n)], means[int(0.975 * n)]

def J(r, cp=2000, tc=60):
    return r["time_s"] + (tc - 60) * max(0, r["pallets_used"] - 1) + cp * r["pallets_used"]

def report(path, base="la_k3"):
    by = defaultdict(dict)
    for r in load(path):
        by[r["variant"]][r["episode"]] = r
    print(f"\n## {Path(path).name}\n")
    print("| 설정 | n | 팔레트 (95% CI) | 시간 s | J2000 (95% CI) | 0.3g 통과 | 0.5g 통과 | 맞물림 | 결정 ms | vs 기준 J (승-패, p) | vs 팀 J (승-패, p) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    res = {}
    for v, eps in by.items():
        rs = [eps[i] for i in sorted(eps)]
        sg = [g for r in rs for g in r["shock_passed_g"]]
        js = [J(r) for r in rs]; ps = [r["pallets_used"] for r in rs]
        row = dict(n=len(rs), pal=statistics.fmean(ps), pal_ci=boot_ci(ps), t=statistics.fmean(r["time_s"] for r in rs),
                   J=statistics.fmean(js), J_ci=boot_ci(js), p3=sum(g >= .3 for g in sg) / len(sg), p5=sum(g >= .5 for g in sg) / len(sg),
                   il=sum(r["bonded"] for r in rs) / max(1, sum(r["boxes_above_deck"] for r in rs)),
                   ms=statistics.fmean(r["decision_ms"] for r in rs))
        for ref in (base, "team"):
            if ref in by and ref != v:
                common = sorted(set(eps) & set(by[ref]))
                d = [J(eps[i]) - J(by[ref][i]) for i in common]
                w, l = sum(x < -1e-9 for x in d), sum(x > 1e-9 for x in d)
                row["vs_" + ref] = (w, l, sign_p(w, l))
        res[v] = row
    for v, r in sorted(res.items(), key=lambda kv: kv[1]["J"]):
        f = lambda k: f"{r[k][0]}-{r[k][1]}, p={r[k][2]:.3f}" if k in r else "-"
        print(f"| `{v}` | {r['n']} | {r['pal']:.2f} ({r['pal_ci'][0]:.2f}–{r['pal_ci'][1]:.2f}) | {r['t']:.0f} | {r['J']:.0f} ({r['J_ci'][0]:.0f}–{r['J_ci'][1]:.0f}) | "
              f"{r['p3']:.1%} | {r['p5']:.1%} | {r['il']:.1%} | {r['ms']:.0f} | {f('vs_' + base)} | {f('vs_team')} |")
    return res

if __name__ == "__main__":
    t = report(sys.argv[1])
    b = t.get("la_k3")
    if b:
        print("\n### 규칙 판정 (테스트 데이터)")
        for v, r in t.items():
            if v in ("la_k3", "team"):
                continue
            w, l, p = r.get("vs_la_k3", (0, 0, 1))
            stab_ok = r["p3"] >= b["p3"] - 0.05
            if v == "la_k3;horizon=3":
                ok = r["J"] <= b["J"] * 1.01 and stab_ok
                print(f"- `{v}` (R2 비열등): J {r['J']:.0f} vs 기준 {b['J']:.0f} ({100*(r['J']/b['J']-1):+.2f}%), 0.3g {r['p3']:.1%} vs {b['p3']:.1%} → {'채택' if ok else '불채택'}")
            else:
                ok = p < 0.05 and w > l and stab_ok
                print(f"- `{v}` (R1): {w}승 {l}패 p={p:.3f}, 0.3g {r['p3']:.1%} vs {b['p3']:.1%} → {'채택' if ok else '불채택'}")
    if len(sys.argv) > 2:
        report(sys.argv[2])
