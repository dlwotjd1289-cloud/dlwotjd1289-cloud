import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parent))  # perception_testkit (helpers, not a test module)
from pac_planning.demo import scene_from_file
from pac_planning import PlannerConfig, PlacementPlanner
from pac_candidates import CandidateBackend, CandidateConfig


@pytest.fixture
def scene():
    return scene_from_file(
        Path(__file__).parents[2] / "test_data/scenario_001_basic.json"
    )


@pytest.fixture
def planner(scene):
    context = scene[3]
    config = PlannerConfig(horizon=2, scenario_count=7)
    backend = CandidateBackend(context, CandidateConfig())
    return PlacementPlanner(
        context=context,
        config=config,
        generate_candidates=backend.generate_candidates,
        validate_constraints=backend.validate_constraints,
    )
