# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 12 | 960 | 70.7 | 8.2 | 12 | 466 | 51 | 0.254 | 0.355 | 0.95 | 0 | 0 | 29.2/65.3 |
| late_heavy | 2 | 160 | 64.7 | 6.6 | 10 | 110 | 31 | 0.122 | 0.282 | 0.60 | 0 | 0 | 22.6/36.2 |
| late_large | 2 | 160 | 69.4 | 8.5 | 12 | 87 | 46 | 0.184 | 0.319 | 0.78 | 0 | 0 | 23.4/37.4 |
| normal | 2 | 160 | 71.0 | 6.8 | 10 | 85 | 47 | 0.255 | 0.373 | 0.92 | 0 | 0 | 26.5/51.2 |
| repeated_sku | 2 | 160 | 73.3 | 10.4 | 14 | 49 | 69 | 0.345 | 0.352 | 1.32 | 0 | 0 | 33.4/81.3 |
| size_mixed | 2 | 160 | 72.3 | 8.4 | 12 | 63 | 61 | 0.328 | 0.408 | 1.09 | 0 | 0 | 36.3/73.3 |
| weight_mixed | 2 | 160 | 73.6 | 8.8 | 12 | 72 | 55 | 0.289 | 0.396 | 1.02 | 0 | 0 | 32.9/63.9 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 2 | 62 | 0.309 | 1.03 | 7 | 7.07% | 1 | 8.70 | 0 |
| humid | 2 | 45 | 0.216 | 0.93 | 0 | 0.00% | 0 | 0.55 | 0 |
| mixed | 2 | 62 | 0.326 | 1.31 | 0 | 0.00% | 0 | 0.64 | 0 |
| nominal | 2 | 44 | 0.205 | 0.80 | 0 | 0.00% | 0 | 0.10 | 0 |
| strong | 2 | 48 | 0.234 | 0.91 | 0 | 0.00% | 0 | 0.23 | 0 |
| weak | 2 | 47 | 0.234 | 0.76 | 0 | 0.00% | 0 | 0.37 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 6 | 480 | 7.7 | 228 | 52 | 0.264 | 0.99 | 0 | 0 | 0 |
| 1.20x1.00 | 6 | 480 | 8.8 | 238 | 50 | 0.243 | 0.92 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 90.3%
- LBCP_UNSTABLE: 78.0%
- HEAVY_ON_LIGHT: 62.7%
- BOX_CAPACITY: 6.3%
- MAX_STACK_HEIGHT: 3.7%
