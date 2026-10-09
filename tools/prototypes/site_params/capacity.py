"""Per-SKU allowable top load = BCT_SKU x site factors / residual safety factor.

BCT_SKU source priority (recorded per SKU):
  1. supplier compression test        (none available yet)
  2. board grade from the box spec    (none available yet)
  3. ESTIMATE: board grade by weight class (this file) — heavier goods ship in
     stronger board. ECT range mid-points and thicknesses are the team's generic
     grades in tools/virtual_data/virtual_data/strength.py (CARTON_GRADES).

Site factors are industry rules of thumb for corrugated boxes (vendor tables,
cited in docs as rules of thumb, not peer-reviewed):
  humidity  RH50 1.00 / RH70 0.80 / RH80 0.68 / RH90 0.48
  storage   10 d 0.63 / 30 d 0.59 / 90 d 0.55 / 180 d 0.50
  stacking misalignment / handling 0.60-0.90 -> 0.80 used
  residual safety factor 1.5
"""
import math

GRADES = {  # name: (ECT N/m = mid of the team's range, board thickness m)
    "single_B": (4250.0, 0.0030),
    "single_C": (5250.0, 0.0040),
    "double_BC": (8500.0, 0.0070),
}
WEIGHT_CLASS = ((5.0, "single_B"), (15.0, "single_C"), (math.inf, "double_BC"))  # by SKU max weight

HUMIDITY = {50: 1.00, 70: 0.80, 80: 0.68, 90: 0.48}
STORAGE_DAYS = {10: 0.63, 30: 0.59, 90: 0.55, 180: 0.50}
HANDLING = 0.80
RESIDUAL_SF = 1.5

SITES = {
    "general": {"rh": 50, "days": 10, "label": "일반 출고 창고 (습도 50%, 10일 이내)"},
    "humid": {"rh": 80, "days": 10, "label": "다습 창고 (습도 80%)"},
    "long": {"rh": 70, "days": 90, "label": "장기 보관 (습도 70%, 90일)"},
}


def bct_n(dx, dy, ect, t):
    return 5.87 * ect * math.sqrt(t * 2.0 * (dx + dy))


def grade_for(weight_max_kg):
    for limit, grade in WEIGHT_CLASS:
        if weight_max_kg <= limit:
            return grade
    return WEIGHT_CLASS[-1][1]


def site_factor(site):
    s = SITES[site]
    return HUMIDITY[s["rh"]] * STORAGE_DAYS[s["days"]] * HANDLING / RESIDUAL_SF


def table(sku_ranges, site):
    """sku -> (allowable N, grade, BCT N, source)."""
    out = {}
    f = site_factor(site)
    for sku, r in sorted(sku_ranges.items()):
        grade = grade_for(r.weight_max_kg)
        ect, t = GRADES[grade]
        bct = bct_n(r.size.x, r.size.y, ect, t)
        out[sku] = (round(bct * f, 3), grade, round(bct, 1), "estimate:weight-class board")
    return out


def patched_build_catalog(site):
    """Drop-in replacement for virtual_data.scenario_source.build_catalog."""
    from pac_common import SkuSpec

    def build(sku_ranges, config, load_model):
        caps = table(sku_ranges, site)
        catalog = {}
        for sku_id, r in sorted(sku_ranges.items()):
            weight = r.weight_max_kg if config.catalog.nominal_weight == "max" else 0.5 * (r.weight_min_kg + r.weight_max_kg)
            catalog[sku_id] = SkuSpec(sku_id, r.size, round(weight, 6), r.allowed_yaws_rad, caps[sku_id][0])
        return catalog
    return build


if __name__ == "__main__":
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path.home() / "AHEAD/pac2026_integrated/tools/highlevel/scripts"))
    import _common  # noqa: F401
    from virtual_data.scenario_source import load_sku_ranges
    ranges = load_sku_ranges(Path.home() / "AHEAD/audit_20261009/03_planner/highlevel_work/dataset80_x10")
    print("현재(균일 판지 ECT 5000, t 3 mm, SF 4) vs 제안(무게 등급별 판지 × 현장 계수 ÷ 1.5)")
    for site in SITES:
        print(f"  site {site}: factor {site_factor(site):.3f} (= BCT ÷ {1/site_factor(site):.2f})  {SITES[site]['label']}")
    print("\nSKU  최대kg  판지      현재(kg)  일반(kg)  다습(kg)  장기(kg)")
    tabs = {s: table(ranges, s) for s in SITES}
    for sku, r in sorted(ranges.items()):
        cur = bct_n(r.size.x, r.size.y, 5000.0, 0.003) / 4 / 9.80665
        g = tabs["general"][sku][1]
        print(f"{sku}  {r.weight_max_kg:5.1f}  {g:9s} {cur:7.1f}  " + "  ".join(f"{tabs[s][sku][0]/9.80665:7.1f}" for s in SITES))
