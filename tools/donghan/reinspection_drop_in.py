"""pytest plugin: run the runtime tests with RuntimeCore using ReinspectionValidator.

Checks that pac_reinspection is a drop-in replacement for pac_runtime's
StateValidator without editing pac_runtime. From the repository root:

    PYTHONPATH=tools/donghan python -m pytest -p reinspection_drop_in tests/taehyeon/test_th_runtime.py

(Windows: ``set PYTHONPATH=tools\\donghan``.) Prints how often the wrapper ran
and which RI codes it tagged.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CALLS = {"validate": 0}
TAGS = {}


def pytest_sessionstart(session):
    import pac_runtime.core as core
    from pac_runtime.state_validator import StateValidator

    from pac_reinspection import ReinspectionValidator, load_policy

    policy = load_policy(ROOT / "config" / "donghan" / "reinspection_policy.yaml")

    class Counting(ReinspectionValidator):
        def validate(self, *args, **kwargs):
            CALLS["validate"] += 1
            verdict = super().validate(*args, **kwargs)
            for code in self.last.ri_codes:
                TAGS[code] = TAGS.get(code, 0) + 1
            return verdict

    core.StateValidator = lambda catalog, cfg, weight_ranges=None: Counting(StateValidator(catalog, cfg, weight_ranges), policy)


def pytest_terminal_summary(terminalreporter):
    terminalreporter.write_line(f"reinspection drop-in: ReinspectionValidator.validate called {CALLS['validate']} times")
    terminalreporter.write_line(f"RI codes tagged: {dict(sorted(TAGS.items()))}")
