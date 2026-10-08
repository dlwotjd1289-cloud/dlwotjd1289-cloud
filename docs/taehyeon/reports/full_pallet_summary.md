# Virtual data summary (stage 5-1/5-2)

| family | scen | steps | gen | valid | valid% | no-valid | placed% | util | density | H(m) | issues | true overlap | ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ALL | 12 | 960 | 76.7 | 8.0 | 10 | 466 | 51 | 0.274 | 0.332 | 1.13 | 0 | 0 | 56.4/114.2 |
| late_heavy | 2 | 160 | 71.9 | 7.8 | 11 | 101 | 37 | 0.145 | 0.285 | 0.90 | 0 | 0 | 53.2/95.9 |
| late_large | 2 | 160 | 73.0 | 9.4 | 13 | 83 | 48 | 0.194 | 0.274 | 0.96 | 0 | 0 | 68.8/125.5 |
| normal | 2 | 160 | 77.3 | 6.3 | 8 | 80 | 50 | 0.277 | 0.295 | 1.27 | 0 | 0 | 48.5/90.6 |
| repeated_sku | 2 | 160 | 78.0 | 8.2 | 11 | 75 | 53 | 0.375 | 0.387 | 1.31 | 0 | 0 | 53.5/110.0 |
| size_mixed | 2 | 160 | 81.8 | 7.9 | 10 | 59 | 63 | 0.346 | 0.393 | 1.19 | 0 | 0 | 57.1/118.6 |
| weight_mixed | 2 | 160 | 78.5 | 8.5 | 11 | 68 | 57 | 0.308 | 0.361 | 1.15 | 0 | 0 | 57.3/120.7 |

## True carton strength (hidden from the planner)

| strength | scen | placed% | util | H(m) | true-overloaded boxes | rate | scenarios w/ overload | max true load ratio | on detected-damaged |
|---|---|---|---|---|---|---|---|---|---|
| extreme | 2 | 61 | 0.314 | 1.16 | 0 | 0.00% | 0 | 0.96 | 0 |
| humid | 2 | 47 | 0.229 | 0.82 | 0 | 0.00% | 0 | 0.68 | 0 |
| mixed | 2 | 50 | 0.369 | 1.31 | 0 | 0.00% | 0 | 0.64 | 0 |
| nominal | 2 | 49 | 0.230 | 1.13 | 0 | 0.00% | 0 | 0.22 | 0 |
| strong | 2 | 49 | 0.241 | 1.09 | 0 | 0.00% | 0 | 0.19 | 0 |
| weak | 2 | 53 | 0.262 | 1.26 | 0 | 0.00% | 0 | 0.93 | 0 |

## Pallet footprints (m)

| pallet | scen | steps | valid | no-valid | placed% | util | H(m) | issues | true overlap | true protrusion |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.10x1.10 | 6 | 480 | 7.2 | 236 | 51 | 0.291 | 1.22 | 0 | 0 | 1 |
| 1.20x1.00 | 6 | 480 | 8.8 | 230 | 52 | 0.258 | 1.04 | 0 | 0 | 0 |

## Mask reasons (share of masked candidates, ALL)

- SUPPORT_RATIO: 90.2%
- LBCP_UNSTABLE: 79.7%
- HEAVY_ON_LIGHT: 50.3%
- MAX_STACK_HEIGHT: 7.1%
- BOX_CAPACITY: 6.3%
