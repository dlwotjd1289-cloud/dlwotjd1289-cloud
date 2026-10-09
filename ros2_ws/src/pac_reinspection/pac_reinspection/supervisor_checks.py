"""Stage 3 proposal: RI-E3 inventory consistency.

``Supervisor.outstanding()`` already computes expected - arrived - missing per
SKU. What is new is comparing it with the boxes seen upstream (inlet camera /
conveyor queue) that have not reached stage 2 yet.
"""
from __future__ import annotations

from collections import Counter


def inventory_consistency(expected_by_sku, arrived, missing, seen_upstream) -> list[str]:
    """Returns RI-E3 issues (empty when consistent). ``seen_upstream``: SKU counts of boxes
    identified on the conveyor before stage 2; unidentified boxes under the key None."""
    expected, arrived, missing, seen = map(Counter, (expected_by_sku, arrived, missing, seen_upstream))
    issues = []
    outstanding = {k: expected[k] - arrived[k] - missing[k] for k in set(expected) | set(arrived) | set(missing)}
    for sku, n in sorted(outstanding.items()):
        if n < 0:
            issues.append(f"RI-E3: {sku} 도착 {arrived[sku]} + MISSING {missing[sku]} > 기대 {expected[sku]}")
    for sku, n in sorted(((k, v) for k, v in seen.items() if k is not None), key=lambda kv: kv[0]):
        if n > max(outstanding.get(sku, 0), 0):
            issues.append(f"RI-E3: {sku} 대기열 {n} > 미도착 기대 {max(outstanding.get(sku, 0), 0)}")
    total_out = sum(max(v, 0) for v in outstanding.values())
    if sum(seen.values()) > total_out:
        issues.append(f"RI-E3: 대기열 박스 {sum(seen.values())} > 미도착 기대 합계 {total_out}")
    return issues
