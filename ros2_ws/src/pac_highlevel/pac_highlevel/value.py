"""Per-option future value ("선택지별 AI Future Value 값, 학습·실전 동일").

The provider name is stored in the trained policy and checked on load, so a
policy trained with one provider is never deployed with another.

* ``proxy``  : heightmap flatness after the placement (1 - std/H). Fast, no
               team dependency; used for training in this repository.
* ``donghan``: donghan's 5-4 value head (``PlacementPlanner.plan`` in
               ``ranking`` mode, no rollout): predicted mean future added
               volume / pallet capacity for the chosen candidate.
"""


def proxy_value(world, box, candidate, state, option):
    return option.flatness_after


class DonghanValue:
    """Wraps donghan's PlacementPlanner value head (optional dependency)."""

    name = "donghan"

    def __init__(self, model_path=None, planner_config=None):
        from pac_planning import PlacementPlanner, PlannerConfig

        self._planner_cls = PlacementPlanner
        self._config = planner_config or PlannerConfig()
        self._model_path = model_path

    def __call__(self, world, box, candidate, state, option):
        backend = world.backend()
        planner = self._planner_cls(
            context=backend.context,
            config=self._config,
            generate_candidates=backend.generate_candidates,
            validate_constraints=backend.validate_constraints,
            model_path=self._model_path,
        )
        result = planner.plan(box, state, [candidate], mode="ranking", use_time_budget=False)
        if not result.evaluations:
            return 0.0
        return float(result.evaluations[0].future.mean)


def make_value_provider(name, **kwargs):
    if name == "proxy":
        return proxy_value
    if name == "donghan":
        return DonghanValue(**kwargs)
    raise ValueError(f"Unknown value provider {name}")
