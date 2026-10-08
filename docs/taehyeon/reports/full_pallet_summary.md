# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 12 | 960 | 73.2 | 7.8 | 11 | 475 | 51 | 0.244 | 0.353 | 0.94 | 0 | 0 | 28.1/57.3 |
| late_heavy | 2 | 160 | 68.3 | 6.7 | 10 | 110 | 31 | 0.121 | 0.280 | 0.60 | 0 | 0 | 23.2/36.2 |
| late_large | 2 | 160 | 65.5 | 8.4 | 13 | 88 | 45 | 0.181 | 0.363 | 0.70 | 0 | 0 | 23.7/37.7 |
| normal | 2 | 160 | 72.7 | 6.9 | 9 | 82 | 49 | 0.263 | 0.352 | 1.02 | 0 | 0 | 26.9/50.3 |
| repeated_sku | 2 | 160 | 82.2 | 9.2 | 11 | 59 | 63 | 0.293 | 0.357 | 1.13 | 0 | 0 | 33.7/77.7 |
| size_mixed | 2 | 160 | 73.6 | 7.7 | 10 | 63 | 61 | 0.322 | 0.380 | 1.17 | 0 | 0 | 29.4/55.0 |
| weight_mixed | 2 | 160 | 76.9 | 7.9 | 10 | 73 | 54 | 0.284 | 0.388 | 1.02 | 0 | 0 | 31.7/59.7 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 2 | 55 | 0.252 | 0.84 | 2 | 2.27% | 1 | 1.48 | 0 |
| humid | 2 | 45 | 0.210 | 1.01 | 0 | 0.00% | 0 | 0.72 | 0 |
| mixed | 2 | 62 | 0.326 | 1.31 | 0 | 0.00% | 0 | 0.64 | 0 |
| nominal | 2 | 46 | 0.210 | 0.81 | 0 | 0.00% | 0 | 0.16 | 0 |
| strong | 2 | 48 | 0.234 | 0.91 | 0 | 0.00% | 0 | 0.23 | 0 |
| weak | 2 | 47 | 0.234 | 0.76 | 0 | 0.00% | 0 | 0.37 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 90.6%
- LBCP_UNSTABLE: 78.6%
- HEAVY_ON_LIGHT: 62.7%
- BOX_CAPACITY: 6.4%
- MAX_STACK_HEIGHT: 2.8%
