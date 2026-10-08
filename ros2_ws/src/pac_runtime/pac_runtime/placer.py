"""Stages 5 + 6 together: the low-level choice only returns candidates the
robot can execute ("실패 시 다음 후보").

Ranking: DBLF (default) or donghan's planner (``pac_planning``). The first
``max_checks`` ranked candidates go through stage 6; when none passes, the
option counts as infeasible for stage 4 (its action is masked).
"""

from collections import Counter

from pac_highlevel.world import dblf_choice  # noqa: F401  (same DBLF key below)


def dblf_order(valid):
    def key(c):
        p = c.target_pose
        return (round(p.z, 6), round(p.y, 6), round(p.x, 6), c.candidate_id)

    return sorted(valid, key=key)


class RobotAwarePlacer:
    wants_context = True

    def __init__(self, robot, max_checks=12, ranker=None):
        self.robot = robot
        self.max_checks = max_checks
        self.ranker = ranker            # (valid, box, state, backend) -> ordered list; None = DBLF
        self.stats = Counter()
        self.last_rejected = {}

    def __call__(self, valid, box, state, backend):
        ordered = self.ranker(valid, box, state, backend) if self.ranker else dblf_order(valid)
        self.stats["calls"] += 1
        chosen, verdict, rejected = self.robot.first_executable(box, ordered[: self.max_checks], state)
        self.last_rejected = rejected
        self.stats["robot_rejected"] += len(rejected)
        for v in rejected.values():
            for code in v.codes:
                self.stats["code:" + code.value] += 1
        if chosen is None:
            self.stats["no_executable"] += 1
        elif rejected:
            self.stats["fell_back"] += 1
        return chosen


def donghan_ranker(model_path=None, planner_config=None, seed=7):
    """Ranking by donghan's 5-3~5-6 planner; candidates it does not rank keep
    their DBLF order behind the ranked ones."""
    from pac_planning import PlacementPlanner, PlannerConfig

    cfg = planner_config or PlannerConfig()
    model = None
    if model_path is not None:
        from pac_highlevel.value import _load_ranker

        model = _load_ranker(model_path)

    def rank(valid, box, state, backend):
        planner = PlacementPlanner(context=backend.context, config=cfg,
                                   generate_candidates=backend.generate_candidates,
                                   validate_constraints=backend.validate_constraints, model=model)
        box = state.inventory.tracked_boxes.get(box.box_id, box)
        ranked = list(planner.plan(box, state, valid, seed=seed).ranked)
        seen = {c.candidate_id for c in ranked}
        return ranked + [c for c in dblf_order(valid) if c.candidate_id not in seen]

    return rank
