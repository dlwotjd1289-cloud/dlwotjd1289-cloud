"""LLM-assisted tuning of the rule / scoring parameters (taehyeon).

An LLM (Claude) proposes parameter sets, a paired evaluator scores them on
the validation scenarios, and the best one is checked once on the held-out
test scenarios. A random-search baseline with the same budget tells whether
the LLM actually helps.
"""

from .space import PARAMS, apply_params, default_params, validate_params
from .evaluate import Evaluator

__all__ = ["PARAMS", "apply_params", "default_params", "validate_params", "Evaluator"]
