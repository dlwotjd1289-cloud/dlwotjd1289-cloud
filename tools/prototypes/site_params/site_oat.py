"""One-factor-at-a-time sensitivity: base site, change ONE input, keep the rest at base.
Uses stage-2 paired episodes (la_k3 vs team); forklift wait = M/M/1 (pessimistic)."""
import json, statistics, sys
from collections import defaultdict
from site_cost import derive

BASE = dict(swap_s=60, wrap_s=0, cells_per_forklift=1, forklift_service_s=180, pallet_interval_s=900,
            freight_krw=300_000, pallets_per_truck=14, forklift_krw_h=35_000, film_krw=500, cell_krw_s=10)
FACTORS = [
    ("팔레트 바꾸는 시간 s", "swap_s", [60, 90, 180]),
    ("셀에서 랩 감는 시간 s", "wrap_s", [0, 60]),
    ("지게차 1대당 셀 수", "cells_per_forklift", [1, 2, 3, 4]),
    ("트럭 1회 운임 원", "freight_krw", [120_000, 300_000, 700_000]),
    ("트럭당 팔레트 수", "pallets_per_truck", [10, 14, 20]),
    ("셀 1초 비용 원", "cell_krw_s", [6, 10, 15]),
]

def load(f):
    by = defaultdict(dict)
    for l in open(f):
        if l.strip():
            r = json.loads(l); by[r["variant"]][r["episode"]] = r
    return by

def compare(by, tc, cp):
    J = lambda r: r["time_s"] + (tc - 60) * max(0, r["pallets_used"] - 1) + cp * r["pallets_used"]
    c = sorted(set(by["la_k3"]) & set(by["team"]))
    a = statistics.fmean(J(by["la_k3"][i]) for i in c); b = statistics.fmean(J(by["team"][i]) for i in c)
    w = sum(J(by["la_k3"][i]) < J(by["team"][i]) for i in c)
    return 100 * (a / b - 1), b - a, w, len(c)

test = load(sys.argv[1]); hold = load(sys.argv[2])
print("| 바꾼 항목 | 값 | 교체시간 s | c_p s | 팀 대비 J (테스트) | 주문당 절감 s | 승 | 처음 보는 데이터 |")
print("|---|---|---|---|---|---|---|---|")
for label, key, vals in FACTORS:
    for v in vals:
        p = {**BASE, key: v}; d = derive(p); tc = d["T_change"][1]
        pt, sv, w, n = compare(test, tc, d["c_p"]); ph, _, wh, nh = compare(hold, tc, d["c_p"])
        mark = " (기준)" if BASE[key] == v else ""
        print(f"| {label} | {v:,}{mark} | {tc:.0f} | {d['c_p']:.0f} | {pt:+.1f}% | {sv:.0f} | {w}/{n} | {ph:+.1f}% ({wh}/{nh}) |")
