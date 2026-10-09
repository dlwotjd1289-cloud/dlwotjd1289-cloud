"""Choose the prototype settings per site profile from a sweep (run_eval jsonl/json).

Selection (stability first, same rule for every site):
  1. shock pass rate at 0.3 g within 5 %p of the best selectable variant
     (0.3 g = provisional target assuming stretch wrapping; a_target_g variants are
      sensitivity-only and never selected)
  2. lowest J = T_total' + c_p * N_pallet, where T_total' re-prices pallet changes:
     T_total' = time_s + (T_change - 60 s) * (pallets_used - 1)
     (the simulator charges 60 s per closed pallet; the last pallet is not closed)
"""
import json, statistics, sys
from collections import defaultdict
from pathlib import Path

SITES = {
    "A 기본(대회 가정)": {"T_change": 60, "c_p": 2000,
        "근거": "자동 팔레트 교환 + 인라인 래퍼 가정. c_p는 팀 상위정책 보상 비율(1 : 0.0005/s)과 동일"},
    "B 수동 교체 현장": {"T_change": 180, "c_p": 2000,
        "근거": "지게차 반출입 + 랩 감기 포함 교체"},
    "C 운송비 높은 현장": {"T_change": 120, "c_p": 5000,
        "근거": "장거리·외부 운송, 팔레트 1개(트럭 칸) 비용이 큼"},
    "D 처리량 우선 현장": {"T_change": 60, "c_p": 1000,
        "근거": "근거리 자체 운송, 인건비·처리시간이 상대적으로 비쌈"},
}

src = Path(sys.argv[1])
rows = ([json.loads(l) for l in src.read_text().splitlines() if l.strip()] if src.suffix == ".jsonl"
        else json.loads(src.read_text())["rows"])
by = defaultdict(list)
for r in rows:
    by[r["variant"]].append(r)

def selectable(v, rs):
    return "a_target_g" not in v and len(rs) >= 18

stats = {}
for v, rs in by.items():
    sg = [g for r in rs for g in r["shock_passed_g"]]
    stats[v] = {"n": len(rs), "pass3": sum(g >= 0.3 for g in sg) / len(sg) if sg else 0.0,
                "pallets": statistics.fmean(r["pallets_used"] for r in rs),
                "time60": statistics.fmean(r["time_s"] for r in rs)}
best = max(s["pass3"] for v, s in stats.items() if selectable(v, by[v]))
eligible = [v for v, s in stats.items() if selectable(v, by[v]) and s["pass3"] >= best - 0.05]

print(f"안정성 기준: 0.3 g 통과율 최고 {best:.1%}, 후보 = 최고 대비 −5%p 이내 ({len(eligible)}개)\n")
print("| 현장 | 교체시간 | 팔레트 1개 환산 | 선택 설정 | 팔레트 | 작업시간 | J | 팀 방식 J | 팀 대비 |")
print("|---|---|---|---|---|---|---|---|---|")
out = {}
for name, p in SITES.items():
    def J(v):
        rs = by[v]
        t = statistics.fmean(r["time_s"] + (p["T_change"] - 60) * max(0, r["pallets_used"] - 1) for r in rs)
        return t + p["c_p"] * stats[v]["pallets"], t
    pick = min(eligible, key=lambda v: J(v)[0])
    jt, tt = J(pick)
    team_j = J("team")[0] if "team" in by else float("nan")
    out[name] = {"pick": pick, "J": jt, "time": tt, "pallets": stats[pick]["pallets"], "team_J": team_j, **p}
    print(f"| {name} | {p['T_change']} s | {p['c_p']} s | `{pick}` | {stats[pick]['pallets']:.2f} | {tt:.0f} s | {jt:.0f} | {team_j:.0f} | {100 * (jt / team_j - 1):+.1f}% |")
print("\n현장 프로필 근거:")
for name, p in SITES.items():
    print(f"- {name}: {p['근거']}")
Path(__file__).with_name("site_selection.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
