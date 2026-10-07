# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 12 | 960 | 72.3 | 7.8 | 11 | 474 | 51 | 0.245 | 0.349 | 0.95 | 0 | 0 | 51.5/104.7 |
| late_heavy | 2 | 160 | 68.3 | 6.7 | 10 | 110 | 31 | 0.121 | 0.280 | 0.60 | 0 | 0 | 40.8/62.0 |
| late_large | 2 | 160 | 62.0 | 8.4 | 14 | 88 | 45 | 0.182 | 0.365 | 0.70 | 0 | 0 | 43.8/66.8 |
| normal | 2 | 160 | 72.7 | 6.9 | 9 | 82 | 49 | 0.263 | 0.352 | 1.02 | 0 | 0 | 49.5/89.3 |
| repeated_sku | 2 | 160 | 82.2 | 9.2 | 11 | 59 | 63 | 0.293 | 0.357 | 1.13 | 0 | 0 | 59.9/153.9 |
| size_mixed | 2 | 160 | 73.6 | 7.7 | 10 | 63 | 61 | 0.322 | 0.380 | 1.17 | 0 | 0 | 60.7/135.1 |
| weight_mixed | 2 | 160 | 75.1 | 7.9 | 11 | 72 | 55 | 0.288 | 0.361 | 1.08 | 0 | 0 | 54.2/97.9 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 2 | 56 | 0.256 | 0.90 | 3 | 3.37% | 2 | 1.48 | 0 |
| humid | 2 | 45 | 0.210 | 1.01 | 0 | 0.00% | 0 | 0.72 | 0 |
| mixed | 2 | 62 | 0.326 | 1.31 | 0 | 0.00% | 0 | 0.64 | 0 |
| nominal | 2 | 46 | 0.210 | 0.81 | 0 | 0.00% | 0 | 0.16 | 0 |
| strong | 2 | 48 | 0.235 | 0.91 | 0 | 0.00% | 0 | 0.23 | 0 |
| weak | 2 | 47 | 0.234 | 0.76 | 0 | 0.00% | 0 | 0.37 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 90.2%
- LBCP_UNSTABLE: 78.2%
- HEAVY_ON_LIGHT: 63.1%
- BOX_CAPACITY: 6.2%
- MAX_STACK_HEIGHT: 2.8%
