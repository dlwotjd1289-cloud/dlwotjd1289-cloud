"""Site profile from cost components (replaces the hand-picked 1000/2000/5000 s).

  T_change [s]  = swap (removal + empty pallet in) + wrap (if inline at the cell) + forklift wait
  c_p [s]       = (truck slot + forklift handling + wrap film) / cell cost per second
  truck slot    = freight per trip / pallets per truck            (pallets themselves are plentiful)
  forklift wait = queue wait when one forklift serves n cells:
                  rho = n * service / interval,  Wq = rho * service / (1 - rho)   (M/M/1, pessimistic)
                  M/D/1 (constant service) gives half of that -> reported as the range
All inputs are ASSUMPTIONS until replaced by site quotes.
"""
import json
from pathlib import Path

PROFILES = {
    "A 자동 교환·자체 지게차": dict(swap_s=60, wrap_s=0, cells_per_forklift=1, forklift_service_s=180,
        pallet_interval_s=900, freight_krw=300_000, pallets_per_truck=14, forklift_krw_h=35_000,
        film_krw=500, cell_krw_s=12),
    "B 수동 교체·지게차 4셀 공유": dict(swap_s=90, wrap_s=60, cells_per_forklift=4, forklift_service_s=180,
        pallet_interval_s=900, freight_krw=300_000, pallets_per_truck=14, forklift_krw_h=35_000,
        film_krw=500, cell_krw_s=8),
    "C 장거리 운송": dict(swap_s=60, wrap_s=60, cells_per_forklift=2, forklift_service_s=180,
        pallet_interval_s=900, freight_krw=700_000, pallets_per_truck=14, forklift_krw_h=35_000,
        film_krw=500, cell_krw_s=8),
    "D 근거리·처리량 우선": dict(swap_s=60, wrap_s=0, cells_per_forklift=1, forklift_service_s=180,
        pallet_interval_s=900, freight_krw=120_000, pallets_per_truck=14, forklift_krw_h=35_000,
        film_krw=500, cell_krw_s=15),
}


def derive(p):
    rho = p["cells_per_forklift"] * p["forklift_service_s"] / p["pallet_interval_s"]
    if rho >= 1:
        raise ValueError(f"forklift overloaded (rho={rho:.2f}) - add forklifts")
    wq_mm1 = rho * p["forklift_service_s"] / (1 - rho) if p["cells_per_forklift"] > 1 else 0.0
    wq = (0.5 * wq_mm1, wq_mm1)                       # M/D/1 .. M/M/1
    slot = p["freight_krw"] / p["pallets_per_truck"]
    fork = p["forklift_service_s"] / 3600 * p["forklift_krw_h"]
    per_pallet = slot + fork + p["film_krw"]
    base = p["swap_s"] + p["wrap_s"]
    return dict(rho=rho, T_change=(base + wq[0], base + wq[1]), krw_per_pallet=per_pallet,
                truck_slot=slot, forklift=fork, c_p=per_pallet / p["cell_krw_s"])


if __name__ == "__main__":
    print("| 현장 | 지게차 가동률 | 교체시간 s (M/D/1–M/M/1) | 팔레트 1개 원 (트럭칸+지게차+필름) | c_p s |")
    print("|---|---|---|---|---|")
    out = {}
    for name, p in PROFILES.items():
        d = derive(p); out[name] = {**p, **d}
        print(f"| {name} | {d['rho']:.0%} | {d['T_change'][0]:.0f}–{d['T_change'][1]:.0f} | "
              f"{d['krw_per_pallet']:,.0f} ({d['truck_slot']:,.0f}+{d['forklift']:,.0f}+{p['film_krw']}) | {d['c_p']:.0f} |")
    Path(__file__).with_name("site_cost.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
