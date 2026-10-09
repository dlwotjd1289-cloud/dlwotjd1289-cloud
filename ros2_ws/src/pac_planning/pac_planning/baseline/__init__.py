"""Comparison baselines: simple algorithms to contrast with the AHEAD algorithm.

Nothing in this subpackage is the team's algorithm, and nothing outside it
imports from here.

    field_rule   field-practice rules, no look-ahead / search / learning / repack
                 (docs/donghan/baseline/field_rule.md)
"""

from .field_rule import (
    FieldRuleConfig,
    FieldRulePlacer,
    FieldRulePolicy,
    field_rule_highlevel_config,
    field_rule_loaded_policy,
    load_field_rule_config,
)

__all__ = [
    "FieldRuleConfig",
    "FieldRulePlacer",
    "FieldRulePolicy",
    "field_rule_highlevel_config",
    "field_rule_loaded_policy",
    "load_field_rule_config",
]
