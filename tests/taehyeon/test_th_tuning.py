"""LLM tuning assistant (tools/tuning): search space, session, Claude tool loop (fake client)."""

from dataclasses import dataclass, field
import json
from pathlib import Path
import sys

import pytest

from th_helpers import REPO  # noqa: F401  (bootstraps sys.path)

sys.path.insert(0, str(REPO / "tools" / "tuning"))

from pac_candidates import CandidateConfig  # noqa: E402
from pac_highlevel import HighLevelConfig  # noqa: E402
from tuning import apply_params, default_params, validate_params  # noqa: E402
from tuning.agent import MODEL, TuningSession, run_llm, run_random  # noqa: E402


def test_search_space_validation_and_apply():
    clean, problems = validate_params({"close.fill_before_buffer": "0.4", "rules.max_buffer_age": 7.6,
                                       "repack.enabled": "false", "nope": 1, "rules.good_support": 3})
    assert clean == {"close.fill_before_buffer": 0.4, "rules.max_buffer_age": 8, "repack.enabled": False}
    assert any("nope" in p for p in problems) and any("good_support" in p for p in problems)
    cand, hl = apply_params(CandidateConfig(), HighLevelConfig(),
                            {"close.fill_before_buffer": 0.5, "generation.dedup_distance_m": 0.05})
    assert hl.close.fill_before_buffer == 0.5 and cand.generation.dedup_distance_m == 0.05
    assert hl.rules == HighLevelConfig().rules  # untouched
    d = default_params(CandidateConfig(), HighLevelConfig())
    assert d["close.fill_before_buffer"] == HighLevelConfig().close.fill_before_buffer


class FakeEvaluator:
    """pallets = 4 - 0.5 * fill_before_buffer; time explodes if repack is off (constraint)."""
    episodes = 10

    def __init__(self):
        self.calls = 0

    def evaluate(self, params):
        self.calls += 1
        pallets = 4.0 - 0.5 * params.get("close.fill_before_buffer", 0.3)
        time_s = 1000.0 * (1.5 if params.get("repack.enabled") is False else 1.0)
        return {"episodes": 10, "pallets_mean": pallets, "fill_mean": 0.25, "robot_time_s_mean": time_s,
                "ng_mean": 0.0, "safety_issues": 0,
                "vs_baseline": {"pallets_diff_mean": pallets - 3.85, "ci95": [0, 0], "better": 0, "equal": 10,
                                "worse": 0, "sign_test_p": 1.0}}


def test_session_budget_cache_and_constraints():
    ev = FakeEvaluator()
    s = TuningSession(ev, budget=2)
    r = s.evaluate({"close.fill_before_buffer": 0.6})
    assert r["acceptable"] and r["budget_left"] == 1
    again = s.evaluate({"close.fill_before_buffer": 0.6})
    assert again["note"].startswith("already") and s.used == 1
    slow = s.evaluate({"repack.enabled": False})
    assert slow["acceptable"] is False  # robot time +50 %
    assert "budget" in s.evaluate({"close.fill_before_buffer": 0.1})["error"]
    assert "outside" in s.evaluate({"close.fill_before_buffer": 9})["error"]
    params, best = s.best()
    assert params == {"close.fill_before_buffer": 0.6}
    assert run_random(TuningSession(FakeEvaluator(), budget=3))["params"] is not None


@dataclass
class Block:
    type: str
    text: str = ""
    name: str = ""
    input: dict = field(default_factory=dict)
    id: str = ""


@dataclass
class Resp:
    content: list
    stop_reason: str = "tool_use"


class FakeClient:
    """Scripted Claude: describe -> good eval -> bad eval -> finish."""

    def __init__(self):
        self.requests = []
        self.script = [
            Resp([Block("text", text="Start with the space."), Block("tool_use", name="describe_search_space", id="t1")]),
            Resp([Block("tool_use", name="evaluate_config", id="t2",
                        input={"params": {"close.fill_before_buffer": 0.5}, "hypothesis": "close earlier"})]),
            Resp([Block("tool_use", name="evaluate_config", id="t3",
                        input={"params": {"close.fill_before_buffer": 5}, "hypothesis": "too far"})]),
            Resp([Block("tool_use", name="finish", id="t4",
                        input={"params": {"close.fill_before_buffer": 0.5}, "rationale": "마감을 조금 늦춤",
                               "further_ideas": "팔레트 2개 동시 운용"})]),
            Resp([Block("text", text="done")], stop_reason="end_turn"),
        ]
        self.beta = self
        self.messages = self

    def create(self, **kwargs):
        self.requests.append(json.loads(json.dumps(kwargs, default=lambda o: o.__dict__)))
        return self.script[len(self.requests) - 1]


def test_llm_loop_with_fake_client():
    client = FakeClient()
    session = TuningSession(FakeEvaluator(), budget=4, defaults={"close.fill_before_buffer": 0.3})
    out = run_llm(session, client=client)
    assert out["finished_by_model"] and out["params"] == {"close.fill_before_buffer": 0.5}
    assert out["rationale"] == "마감을 조금 늦춤" and session.used == 1
    first = client.requests[0]
    assert first["model"] == MODEL == "claude-opus-5-5"
    assert first["fallbacks"] == "default" and first["betas"] == ["server-side-fallback-2026-07-01"]
    assert {t["name"] for t in first["tools"]} == {"describe_search_space", "evaluate_config", "finish"}
    # describe result carries the current values; the out-of-bounds proposal came back as an error
    results = [m for m in client.requests[-1]["messages"] if m["role"] == "user" and isinstance(m["content"], list)]
    describe = json.loads(results[0]["content"][0]["content"])
    assert describe["current_values"] == {"close.fill_before_buffer": 0.3}
    bad = results[2]["content"][0]
    assert bad.get("is_error") is True and "outside" in bad["content"]
