"""Summarise a parameter sweep from run_eval.py output.

Selection rule (stability first):
  1. shock pass rate at 0.3 g (share of pallets, unwrapped) within 5 %p of the best variant
  2. among those, lowest J = time_s + c_p * pallets_used  (c_p = 2000 s by default)
  3. report the same ranking for c_p = 1000 and 5000 s (sensitivity)
Paired sign test of J per episode against the reference variant.
"""
import json, math, statistics, sys
from collections import defaultdict
from pathlib import Path

src = Path(sys.argv[1])
ref = sys.argv[2] if len(sys.argv) > 2 else "la_k3"
if src.suffix == ".jsonl":
    d = {"rows": [json.loads(l) for l in src.read_text().splitlines() if l.strip()]}
else:
    d = json.loads(src.read_text())
rows = defaultdict(dict)
for r in d["rows"]:
    rows[r["variant"]][r["episode"]] = r

def J(r, cp):
    return r["time_s"] + cp * r["pallets_used"]

def sign_p(w, l):
    n = w + l
    if n == 0:
        return 1.0
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(0, min(w, l) + 1)) / 2 ** n)

table = []
for v, eps in rows.items():
    rs = [eps[i] for i in sorted(eps)]
    sg = [g for r in rs for g in r["shock_passed_g"]]
    pass3 = sum(g >= 0.3 for g in sg) / len(sg) if sg else float("nan")
    pass5 = sum(g >= 0.5 for g in sg) / len(sg) if sg else float("nan")
    row = {"variant": v, "n": len(rs),
           "pallets": statistics.fmean(r["pallets_used"] for r in rs),
           "time_s": statistics.fmean(r["time_s"] for r in rs),
           "pass_0.3g": pass3, "pass_0.5g": pass5,
           "decision_ms": statistics.fmean(r["decision_ms"] for r in rs),
           "interlock": sum(r["bonded"] for r in rs) / max(1, sum(r["boxes_above_deck"] for r in rs)),
           "ng": statistics.fmean(r["ng"] for r in rs)}
    for cp in (1000, 2000, 5000):
        row[f"J{cp}"] = statistics.fmean(J(r, cp) for r in rs)
    if ref in rows and v != ref:
        common = sorted(set(eps) & set(rows[ref]))
        diff = [J(eps[i], 2000) - J(rows[ref][i], 2000) for i in common]
        w = sum(x < -1e-9 for x in diff); l = sum(x > 1e-9 for x in diff)
        row["vs_ref_J2000"] = f"{w}승 {l}패 (p={sign_p(w, l):.3f})"
    table.append(row)

# physical constraint values (stability target) are reported for sensitivity only, never selected
SENSITIVITY_ONLY = ("a_target_g",)
def selectable(r):
    return not any(k in r["variant"] for k in SENSITIVITY_ONLY) and r["n"] >= 18
best3 = max(r["pass_0.3g"] for r in table if selectable(r))
eligible = [r for r in table if selectable(r) and r["pass_0.3g"] >= best3 - 0.05]
print(f"# {src.name}  (ref = {ref}, best 0.3 g pass = {best3:.1%}, eligible = within 5 %p)\n")
print("| variant | 팔레트 | 시간(s) | 0.3g 통과 | 0.5g 통과 | 맞물림 | J(1000) | J(2000) | J(5000) | 결정 ms | vs ref (J2000) | 후보 |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|")
for r in sorted(table, key=lambda r: r["J2000"]):
    print(f"| `{r['variant']}` | {r['pallets']:.2f} | {r['time_s']:.0f} | {r['pass_0.3g']:.1%} | {r['pass_0.5g']:.1%} | "
          f"{r['interlock']:.1%} | {r['J1000']:.0f} | {r['J2000']:.0f} | {r['J5000']:.0f} | {r['decision_ms']:.0f} | "
          f"{r.get('vs_ref_J2000', '-')} | {'O' if r in eligible else ('참고' if not selectable(r) else '')} |")
for cp in (1000, 2000, 5000):
    b = min(eligible, key=lambda r: r[f"J{cp}"])
    print(f"\n선택 (c_p = {cp} s): `{b['variant']}`  J={b[f'J{cp}']:.0f}, 팔레트 {b['pallets']:.2f}, 시간 {b['time_s']:.0f} s, 0.3 g 통과 {b['pass_0.3g']:.1%}")
