# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 12 | 960 | 76.9 | 8.0 | 10 | 463 | 52 | 0.275 | 0.331 | 1.14 | 0 | 0 | 59.4/123.8 |
| late_heavy | 2 | 160 | 71.7 | 7.8 | 11 | 101 | 37 | 0.145 | 0.285 | 0.90 | 0 | 0 | 55.6/100.3 |
| late_large | 2 | 160 | 73.6 | 9.4 | 13 | 81 | 49 | 0.202 | 0.286 | 0.96 | 0 | 0 | 76.2/136.2 |
| normal | 2 | 160 | 78.6 | 6.3 | 8 | 81 | 49 | 0.274 | 0.297 | 1.25 | 0 | 0 | 51.3/103.0 |
| repeated_sku | 2 | 160 | 78.0 | 8.6 | 11 | 71 | 56 | 0.380 | 0.384 | 1.34 | 0 | 0 | 55.4/103.2 |
| size_mixed | 2 | 160 | 81.2 | 7.6 | 9 | 61 | 62 | 0.338 | 0.372 | 1.22 | 0 | 0 | 60.7/130.6 |
| weight_mixed | 2 | 160 | 78.5 | 8.4 | 11 | 68 | 57 | 0.308 | 0.361 | 1.15 | 0 | 0 | 57.5/126.2 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 2 | 61 | 0.314 | 1.16 | 0 | 0.00% | 0 | 0.96 | 0 |
| humid | 2 | 47 | 0.229 | 0.82 | 0 | 0.00% | 0 | 0.68 | 0 |
| mixed | 2 | 52 | 0.374 | 1.33 | 0 | 0.00% | 0 | 0.64 | 0 |
| nominal | 2 | 48 | 0.228 | 1.11 | 0 | 0.00% | 0 | 0.22 | 0 |
| strong | 2 | 51 | 0.249 | 1.09 | 0 | 0.00% | 0 | 0.20 | 0 |
| weak | 2 | 52 | 0.254 | 1.30 | 0 | 0.00% | 0 | 0.93 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 6 | 480 | 7.3 | 232 | 52 | 0.292 | 1.24 | 0 | 0 | 1 |
| 1.20x1.00 | 6 | 480 | 8.8 | 231 | 52 | 0.257 | 1.03 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 90.5%
- LBCP_UNSTABLE: 79.8%
- HEAVY_ON_LIGHT: 51.4%
- MAX_STACK_HEIGHT: 6.8%
- BOX_CAPACITY: 6.5%
