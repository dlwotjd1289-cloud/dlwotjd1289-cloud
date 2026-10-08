# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 30 | 720 | 49.2 | 14.2 | 29 | 154 | 79 | 0.178 | 0.354 | 0.70 | 0 | 0 | 12.1/23.3 |
| late_heavy | 5 | 120 | 48.3 | 13.5 | 28 | 43 | 64 | 0.091 | 0.395 | 0.31 | 0 | 0 | 12.3/23.2 |
| late_large | 5 | 120 | 49.1 | 15.8 | 32 | 28 | 77 | 0.109 | 0.278 | 0.56 | 0 | 0 | 12.6/21.4 |
| normal | 5 | 120 | 50.9 | 14.7 | 29 | 18 | 85 | 0.213 | 0.396 | 0.76 | 0 | 0 | 10.9/21.0 |
| repeated_sku | 5 | 120 | 40.8 | 12.4 | 30 | 30 | 75 | 0.222 | 0.341 | 0.88 | 0 | 0 | 11.1/24.5 |
| size_mixed | 5 | 120 | 52.8 | 14.6 | 28 | 16 | 87 | 0.197 | 0.365 | 0.74 | 0 | 0 | 12.2/19.4 |
| weight_mixed | 5 | 120 | 53.0 | 14.0 | 26 | 19 | 84 | 0.237 | 0.347 | 0.94 | 0 | 0 | 13.4/27.2 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 5 | 80 | 0.152 | 0.59 | 0 | 0.00% | 0 | 0.81 | 0 |
| humid | 5 | 69 | 0.158 | 0.62 | 0 | 0.00% | 0 | 0.38 | 0 |
| mixed | 5 | 73 | 0.170 | 0.70 | 0 | 0.00% | 0 | 0.66 | 0 |
| nominal | 5 | 84 | 0.171 | 0.76 | 0 | 0.00% | 0 | 0.33 | 0 |
| strong | 5 | 82 | 0.195 | 0.75 | 0 | 0.00% | 0 | 0.28 | 0 |
| weak | 5 | 82 | 0.223 | 0.78 | 0 | 0.00% | 0 | 0.68 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 84.5%
- LBCP_UNSTABLE: 66.8%
- HEAVY_ON_LIGHT: 59.0%
- BOX_CAPACITY: 3.9%
- MAX_STACK_HEIGHT: 1.3%
