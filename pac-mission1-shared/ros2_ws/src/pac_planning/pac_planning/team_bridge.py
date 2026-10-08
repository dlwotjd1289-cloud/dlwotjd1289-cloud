"""Stage 4 -> team's stage 5-1/2 -> stage 5-3/6, without robot I/O.

Keep pac_common/pac_planning from the planner repository on the import path.
The workcell repository has different packages with those same names.
"""

from dataclasses import replace

from .config import PlannerConfig
from .planner import PlacementPlanner


def plan_with_backend(box, state, backend, *, candidates=None, config=None,
                      model_path=None, seed=7, use_time_budget=True, mode="ahead"):
    """Preserve the authoritative context and attach real EMS to each candidate.

No model is loaded by default: the shipped reference-backend model is not a
validated model for the teammate's EMS/LBCP backend. The caller must explicitly
choose and validate a model before passing model_path.
"""
    if backend.context is None:
        raise ValueError("A PlanningContext with explicit load limits is required")
    generation = backend.generate_with_report(box, state)
    context = backend.context_with_ems(
        replace(backend.context, ems_upper_by_candidate={}), generation
    )
    if candidates is None:
        candidates = list(generation.candidates)
    else:
        candidates = list(candidates)
        generated = {c.candidate_id: c for c in generation.candidates}
        # A cached ID is not enough: reject stale/modified poses and versions.
        for c in candidates:
            expected = generated.get(c.candidate_id)
            if (expected is None or c.box_id != expected.box_id
                    or c.target_pose != expected.target_pose
                    or c.base_state_version != expected.base_state_version):
                raise ValueError("Candidate does not match this backend snapshot")
    planner = PlacementPlanner(
        context=context, config=config or PlannerConfig(), model_path=model_path,
        generate_candidates=backend.generate_candidates,
        validate_constraints=backend.validate_constraints,
    )
    return planner.plan(box, state, candidates, seed=seed, mode=mode,
                        use_time_budget=use_time_budget)


class TeamPlacer:
    """Optional pac_highlevel.PalletizingWorld placer; all states stay SIMULATED.

Only changes placement selection. It does NOT replace the PPO's trained
value_provider=proxy observation with a different future-value definition.
"""

    wants_context = True
    name = "donghan_ems_rollout_v1"

    def __init__(self, config=None, *, seed=7, use_time_budget=True, on_plan=None):
        self.config = config or PlannerConfig()
        self.seed = seed
        self.use_time_budget = use_time_budget
        self.last_result = None
        self.calls = 0
        self.on_plan = on_plan

    def __call__(self, valid, box, state, backend):
        self.last_result = None
        self.calls += 1
        # High-level retains the arrival object while a buffered snapshot has
        # status BUFFERED. Use the State Manager's exact current object.
        box = state.inventory.tracked_boxes[box.box_id]
        self.last_result = plan_with_backend(
            box, state, backend, candidates=valid, config=self.config,
            seed=self.seed, use_time_budget=self.use_time_budget,
        )
        if self.on_plan is not None:
            self.on_plan(box, state, self.last_result)
        return self.last_result.ranked[0] if self.last_result.ranked else None


def check_policy_contract(contract, *, placer_name, value_provider):
    """Changing the low-level placer changes PPO observations and transitions.

Do not relabel a DBLF-trained policy as compatible by rewriting its metadata.
Train/evaluate the policy with the intended placer before deploying it.
"""
    if (contract.get("placer") != placer_name
            or contract.get("value_provider") != value_provider):
        raise ValueError("PPO placer/value-provider mismatch: retraining or explicit offline evaluation required")
