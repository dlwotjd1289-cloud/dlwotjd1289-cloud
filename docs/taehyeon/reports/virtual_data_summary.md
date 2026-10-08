# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 30 | 720 | 47.7 | 14.1 | 30 | 145 | 80 | 0.191 | 0.379 | 0.69 | 0 | 0 | 14.2/30.9 |
| late_heavy | 5 | 120 | 49.9 | 13.7 | 27 | 34 | 72 | 0.117 | 0.369 | 0.45 | 0 | 0 | 16.2/35.5 |
| late_large | 5 | 120 | 48.1 | 16.5 | 34 | 25 | 79 | 0.120 | 0.301 | 0.56 | 0 | 0 | 15.3/31.3 |
| normal | 5 | 120 | 48.1 | 14.7 | 31 | 23 | 81 | 0.206 | 0.426 | 0.69 | 0 | 0 | 12.4/24.3 |
| repeated_sku | 5 | 120 | 40.5 | 12.3 | 30 | 29 | 76 | 0.253 | 0.401 | 0.87 | 0 | 0 | 11.8/27.8 |
| size_mixed | 5 | 120 | 49.8 | 14.2 | 28 | 14 | 88 | 0.213 | 0.420 | 0.70 | 0 | 0 | 16.2/35.4 |
| weight_mixed | 5 | 120 | 50.1 | 13.4 | 27 | 20 | 83 | 0.236 | 0.358 | 0.88 | 0 | 0 | 13.5/25.5 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 5 | 79 | 0.155 | 0.61 | 0 | 0.00% | 0 | 0.81 | 0 |
| humid | 5 | 74 | 0.208 | 0.70 | 0 | 0.00% | 0 | 0.56 | 0 |
| mixed | 5 | 75 | 0.182 | 0.61 | 0 | 0.00% | 0 | 0.52 | 0 |
| nominal | 5 | 87 | 0.187 | 0.75 | 0 | 0.00% | 0 | 0.26 | 0 |
| strong | 5 | 82 | 0.194 | 0.66 | 0 | 0.00% | 0 | 0.13 | 0 |
| weak | 5 | 82 | 0.219 | 0.83 | 0 | 0.00% | 0 | 0.74 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 12 | 288 | 14.6 | 50 | 83 | 0.199 | 0.72 | 0 | 0 | 0 |
| 1.20x0.80 | 6 | 144 | 9.9 | 41 | 72 | 0.239 | 0.79 | 0 | 0 | 0 |
| 1.20x1.00 | 12 | 288 | 15.8 | 54 | 81 | 0.159 | 0.62 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 85.8%
- LBCP_UNSTABLE: 68.9%
- HEAVY_ON_LIGHT: 56.0%
- BOX_CAPACITY: 3.4%
- MAX_STACK_HEIGHT: 2.9%
