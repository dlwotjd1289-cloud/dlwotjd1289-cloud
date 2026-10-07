# tools/virtual_data (owner: taehyun)

Virtual data generator and benchmarks for AHEAD stages 5-1/5-2.
Documentation: [docs/taehyun/virtual_data.md](../../docs/taehyun/virtual_data.md),
results: [docs/taehyun/VALIDATION.md](../../docs/taehyun/VALIDATION.md).

| script | purpose |
|---|---|
| `scripts/generate_virtual_data.py` | replay jaesung's scenarios through 5-1/5-2, write scenes + labels |
| `scripts/validate_virtual_data.py` | consistency / leakage / reproducibility checks |
| `scripts/benchmark_candidates.py` | candidate recall vs a dense grid-search oracle |
| `scripts/physics_crosscheck.py` | hard-mask verdicts vs jaesung's PyBullet simulator |
| `scripts/planner_benchmark.py` | donghan's planner (5-3~5-6) driven by this backend |
