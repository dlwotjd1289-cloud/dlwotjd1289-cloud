# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 30 | 720 | 47.8 | 14.1 | 29 | 146 | 80 | 0.190 | 0.377 | 0.70 | 0 | 0 | 17.3/37.0 |
| late_heavy | 5 | 120 | 50.8 | 13.8 | 27 | 32 | 73 | 0.119 | 0.378 | 0.46 | 0 | 0 | 19.9/41.9 |
| late_large | 5 | 120 | 48.2 | 16.4 | 34 | 26 | 78 | 0.118 | 0.298 | 0.56 | 0 | 0 | 18.1/41.3 |
| normal | 5 | 120 | 48.1 | 14.7 | 31 | 23 | 81 | 0.206 | 0.426 | 0.69 | 0 | 0 | 14.7/28.7 |
| repeated_sku | 5 | 120 | 39.8 | 12.0 | 30 | 31 | 74 | 0.250 | 0.380 | 0.91 | 0 | 0 | 14.4/31.4 |
| size_mixed | 5 | 120 | 49.8 | 14.2 | 28 | 14 | 88 | 0.213 | 0.420 | 0.70 | 0 | 0 | 19.3/37.3 |
| weight_mixed | 5 | 120 | 50.4 | 13.5 | 27 | 20 | 83 | 0.236 | 0.358 | 0.88 | 0 | 0 | 17.6/37.5 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 5 | 79 | 0.155 | 0.61 | 0 | 0.00% | 0 | 0.81 | 0 |
| humid | 5 | 74 | 0.208 | 0.67 | 0 | 0.00% | 0 | 0.47 | 0 |
| mixed | 5 | 77 | 0.185 | 0.73 | 0 | 0.00% | 0 | 0.52 | 0 |
| nominal | 5 | 84 | 0.181 | 0.69 | 0 | 0.00% | 0 | 0.20 | 0 |
| strong | 5 | 82 | 0.194 | 0.66 | 0 | 0.00% | 0 | 0.13 | 0 |
| weak | 5 | 82 | 0.219 | 0.83 | 0 | 0.00% | 0 | 0.74 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 12 | 288 | 14.4 | 52 | 82 | 0.197 | 0.73 | 0 | 0 | 0 |
| 1.20x0.80 | 6 | 144 | 9.9 | 39 | 73 | 0.241 | 0.81 | 0 | 0 | 0 |
| 1.20x1.00 | 12 | 288 | 15.9 | 55 | 81 | 0.158 | 0.61 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 86.2%
- LBCP_UNSTABLE: 69.6%
- HEAVY_ON_LIGHT: 53.0%
- MAX_STACK_HEIGHT: 3.6%
- BOX_CAPACITY: 3.4%
