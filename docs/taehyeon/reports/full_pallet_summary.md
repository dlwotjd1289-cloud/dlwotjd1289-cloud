# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 12 | 960 | 75.8 | 8.1 | 11 | 452 | 53 | 0.283 | 0.343 | 1.14 | 0 | 0 | 48.9/110.8 |
| late_heavy | 2 | 160 | 71.1 | 7.8 | 11 | 101 | 37 | 0.145 | 0.285 | 0.90 | 0 | 0 | 40.7/63.4 |
| late_large | 2 | 160 | 73.2 | 9.7 | 13 | 74 | 54 | 0.233 | 0.305 | 1.06 | 0 | 0 | 67.9/140.4 |
| normal | 2 | 160 | 75.6 | 6.3 | 8 | 80 | 50 | 0.277 | 0.295 | 1.27 | 0 | 0 | 38.0/73.7 |
| repeated_sku | 2 | 160 | 76.6 | 8.4 | 11 | 70 | 56 | 0.388 | 0.402 | 1.31 | 0 | 0 | 45.3/98.7 |
| size_mixed | 2 | 160 | 81.3 | 7.8 | 10 | 60 | 62 | 0.343 | 0.369 | 1.25 | 0 | 0 | 52.0/122.9 |
| weight_mixed | 2 | 160 | 77.1 | 8.4 | 11 | 67 | 58 | 0.311 | 0.404 | 1.07 | 0 | 0 | 49.7/102.1 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 2 | 62 | 0.321 | 1.07 | 3 | 3.00% | 1 | 1.75 | 0 |
| humid | 2 | 47 | 0.229 | 0.82 | 0 | 0.00% | 0 | 0.68 | 0 |
| mixed | 2 | 52 | 0.377 | 1.30 | 0 | 0.00% | 0 | 0.64 | 0 |
| nominal | 2 | 50 | 0.238 | 1.05 | 0 | 0.00% | 0 | 0.22 | 0 |
| strong | 2 | 54 | 0.272 | 1.27 | 0 | 0.00% | 0 | 0.18 | 0 |
| weak | 2 | 52 | 0.259 | 1.33 | 0 | 0.00% | 0 | 0.93 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 6 | 480 | 7.3 | 227 | 53 | 0.303 | 1.30 | 0 | 0 | 0 |
| 1.20x1.00 | 6 | 480 | 8.8 | 225 | 53 | 0.263 | 0.98 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 90.3%
- LBCP_UNSTABLE: 80.1%
- HEAVY_ON_LIGHT: 60.1%
- MAX_STACK_HEIGHT: 7.4%
- BOX_CAPACITY: 7.3%
