# Tuning report (random)

- validation: 36 episodes, 12/12 evaluations
- chosen parameters: `{"repack.enabled": true, "close.fill_before_buffer": 0.624}`
- test (108 episodes): pallets 3.8285 -> 3.6646 (diff -0.164, 95 % CI [-0.2471, -0.0806], better/equal/worse 50/46/12, sign-test p 0.0)

## Rationale

random search: best acceptable evaluation

## Further ideas



## Validation history

| # | params | pallets | diff | p | ok |
|---|---|---|---|---|---|
| 1 | `{"generation.dedup_distance_m": 0.054, "rules.good_support": 0.805}` | 4.2271 | +0.068 | 0.8318 | True |
| 2 | `{"repack.enabled": true, "close.fill_before_buffer": 0.624}` | 3.9716 | -0.188 | 0.0015 | True |
| 3 | `{"rules.max_buffer_age": 20}` | 4.2844 | +0.125 | 0.1686 | True |
| 4 | `{"rules.retrieve_margin_m": 0.093}` | 4.0869 | -0.072 | 0.8145 | True |
| 5 | `{"rules.max_buffer_age": 8, "generation.balance_anchors": false}` | 4.2084 | +0.049 | 0.3075 | True |
| 6 | `{"repack.max_moves": 1, "close.fill_before_buffer": 0.248, "repack.enabled": true}` | 4.3957 | +0.237 | 0.0 | True |
| 7 | `{"close.fill_before_buffer": 0.31, "repack.enabled": false, "generation.balance_anchors": true}` | 4.3527 | +0.194 | 0.2632 | True |
| 8 | `{"rules.good_support": 0.936, "generation.balance_anchors": false, "repack.max_moves": 3}` | 4.2467 | +0.088 | 0.5413 | True |
| 9 | `{"rules.good_support": 0.931, "repack.enabled": true, "close.fill_before_buffer": 0.511}` | 4.0193 | -0.140 | 0.0146 | True |
| 10 | `{"rules.retrieve_margin_m": 0.033, "generation.balance_anchors": false, "repack.enabled": true}` | 4.2934 | +0.134 | 0.1892 | True |
| 11 | `{"generation.balance_anchors": false, "rules.good_support": 0.95, "generation.dedup_distance_m": 0.15}` | 4.2639 | +0.105 | 0.4049 | True |
| 12 | `{"rules.retrieve_margin_m": 0.083, "rules.max_buffer_age": 9}` | 4.163 | +0.004 | 0.845 | True |
