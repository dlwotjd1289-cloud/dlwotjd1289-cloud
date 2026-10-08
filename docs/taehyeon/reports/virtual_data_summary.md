# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 30 | 720 | 46.7 | 13.4 | 29 | 175 | 76 | 0.169 | 0.363 | 0.67 | 0 | 0 | 10.6/20.3 |
| late_heavy | 5 | 120 | 46.5 | 12.8 | 27 | 45 | 62 | 0.086 | 0.411 | 0.33 | 0 | 0 | 9.6/16.9 |
| late_large | 5 | 120 | 47.7 | 15.8 | 33 | 31 | 74 | 0.108 | 0.297 | 0.53 | 0 | 0 | 8.8/14.8 |
| normal | 5 | 120 | 47.6 | 14.6 | 31 | 23 | 81 | 0.206 | 0.426 | 0.69 | 0 | 0 | 11.5/19.7 |
| repeated_sku | 5 | 120 | 39.6 | 11.4 | 29 | 36 | 70 | 0.179 | 0.317 | 0.81 | 0 | 0 | 10.4/22.3 |
| size_mixed | 5 | 120 | 49.0 | 12.8 | 26 | 16 | 87 | 0.207 | 0.357 | 0.80 | 0 | 0 | 12.9/21.7 |
| weight_mixed | 5 | 120 | 49.9 | 13.0 | 26 | 24 | 80 | 0.227 | 0.368 | 0.85 | 0 | 0 | 10.2/19.1 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 5 | 78 | 0.153 | 0.60 | 0 | 0.00% | 0 | 0.81 | 0 |
| humid | 5 | 67 | 0.141 | 0.51 | 0 | 0.00% | 0 | 0.36 | 0 |
| mixed | 5 | 69 | 0.154 | 0.64 | 0 | 0.00% | 0 | 0.66 | 0 |
| nominal | 5 | 83 | 0.179 | 0.82 | 0 | 0.00% | 0 | 0.50 | 0 |
| strong | 5 | 78 | 0.178 | 0.62 | 0 | 0.00% | 0 | 0.13 | 0 |
| weak | 5 | 78 | 0.209 | 0.82 | 0 | 0.00% | 0 | 0.74 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 12 | 288 | 13.8 | 62 | 78 | 0.182 | 0.74 | 0 | 0 | 0 |
| 1.20x0.80 | 6 | 144 | 9.2 | 46 | 68 | 0.192 | 0.77 | 0 | 0 | 0 |
| 1.20x1.00 | 12 | 288 | 15.0 | 67 | 77 | 0.145 | 0.55 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 84.4%
- LBCP_UNSTABLE: 66.7%
- HEAVY_ON_LIGHT: 59.8%
- BOX_CAPACITY: 3.6%
- MAX_STACK_HEIGHT: 2.6%
