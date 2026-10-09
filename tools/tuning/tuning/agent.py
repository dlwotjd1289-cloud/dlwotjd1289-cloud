"""Claude as a tuning assistant: proposes parameter sets, reads the paired
validation results, explains its reasoning, and picks a final set.

The model only sees validation results; the final choice is checked once on
the held-out test split by the caller. Safety constraints are outside the
search space, and every proposal is validated against the bounds.
"""

import json
import random
import time

from .space import BY_NAME, PARAMS, describe, validate_params

MODEL = "claude-opus-5-5"

SYSTEM = """You are tuning the decision rules of a robotic mixed-palletizing cell (HD Hyundai HDR50-22 robot).
Boxes arrive one at a time in an unknown order; the SKU list and counts are known in advance. A rule policy
decides PLACE_CURRENT / BUFFER_CURRENT / RETRIEVE_BUFFER / PALLET_CLOSE / PARTIAL_REPACK, and a candidate
generator proposes placements that a hard safety mask (support, load-bearing polygon, carton strength,
heavy-on-light, pallet load and CoG) filters. You can only change the rule and generation parameters listed by
describe_search_space; the safety mask is fixed and must never be weakened.

Objective: minimise the mean number of pallets used (pallet equivalents) on the validation scenarios.
Constraints: safety_issues must stay 0, mean robot time may grow at most 10 % over the baseline, NG must not grow.
Every evaluation is paired with the baseline on identical box streams; look at better/equal/worse, the sign-test p
and the 95 % interval, not only at the mean - the validation set is small and noisy.

Work like a careful engineer: start from the baseline, form a hypothesis about why pallets are wasted, change one
or two parameters at a time, and use what each result tells you. You have a limited evaluation budget.
When you are done (or the budget is spent), call finish with the best parameter set you found (only the
parameters you changed), a short rationale and ideas for rules outside this search space that could help.
Write the rationale and ideas in Korean."""

TOOLS = [
    {
        "name": "describe_search_space",
        "description": "List the tunable parameters (name, type, range, meaning), their current values, the "
                       "baseline validation metrics and the remaining evaluation budget.",
        "input_schema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "evaluate_config",
        "description": "Run the validation scenarios with these parameter overrides (unlisted parameters keep "
                       "their current value) and return metrics plus the paired comparison with the baseline. "
                       "Costs one evaluation from the budget (identical repeats are free).",
        "input_schema": {
            "type": "object",
            "properties": {
                "params": {"type": "object", "description": "parameter name -> value, e.g. "
                           "{\"close.fill_before_buffer\": 0.4}"},
                "hypothesis": {"type": "string", "description": "what you expect and why (one or two sentences)"},
            },
            "required": ["params", "hypothesis"],
            "additionalProperties": False,
        },
    },
    {
        "name": "finish",
        "description": "Stop and report the chosen parameter set.",
        "input_schema": {
            "type": "object",
            "properties": {
                "params": {"type": "object", "description": "the chosen overrides ({} = keep the baseline)"},
                "rationale": {"type": "string", "description": "why this set (Korean)"},
                "further_ideas": {"type": "string", "description": "rules outside the search space worth trying (Korean)"},
            },
            "required": ["params", "rationale", "further_ideas"],
            "additionalProperties": False,
        },
    },
]


def acceptable(result, baseline):
    """Constraint check of one evaluation against the baseline evaluation."""
    if result["safety_issues"] > 0:
        return False
    if result["robot_time_s_mean"] > 1.10 * baseline["robot_time_s_mean"]:
        return False
    return result["ng_mean"] <= baseline["ng_mean"] + 1e-9


class TuningSession:
    """Shared bookkeeping for the LLM and the random-search baseline."""

    def __init__(self, evaluator, budget, log=None, defaults=None):
        self.ev = evaluator
        self.ev_defaults = dict(defaults or {})
        self.budget = budget
        self.used = 0
        self.baseline = evaluator.evaluate({})
        self.history = []          # (params, result)
        self.log = log or (lambda row: None)

    def evaluate(self, params, hypothesis=""):
        clean, problems = validate_params(params)
        if problems:
            return {"error": "; ".join(problems)}
        key = json.dumps(clean, sort_keys=True)
        seen = next((r for p, r in self.history if json.dumps(p, sort_keys=True) == key), None)
        if seen is not None:
            return {**seen, "note": "already evaluated (free)"}
        if self.used >= self.budget:
            return {"error": "evaluation budget exhausted - call finish"}
        self.used += 1
        t = time.perf_counter()
        result = self.ev.evaluate(clean)
        result["acceptable"] = acceptable(result, self.baseline)
        result["budget_left"] = self.budget - self.used
        self.history.append((clean, result))
        self.log({"event": "evaluate", "params": clean, "hypothesis": hypothesis, "result": result,
                  "seconds": round(time.perf_counter() - t, 1)})
        return result

    def best(self):
        ok = [(p, r) for p, r in self.history if r.get("acceptable")]
        if not ok:
            return {}, self.baseline
        return min(ok, key=lambda pr: (pr[1]["pallets_mean"], pr[1]["robot_time_s_mean"]))


def _text(blocks):
    return "\n".join(b.text for b in blocks if getattr(b, "type", "") == "text")


def run_llm(session, client=None, model=MODEL, effort="high", max_turns=60, log=None):
    """Tool loop with Claude. Returns dict(params, rationale, further_ideas, finished_by_model, transcript)."""
    if client is None:
        import anthropic

        client = anthropic.Anthropic()
    log = log or (lambda row: None)
    messages = [{"role": "user", "content": (
        f"Tune the parameters. Evaluation budget: {session.budget} evaluations on "
        f"{session.ev.episodes} validation episodes. Begin with describe_search_space.")}]
    finished = None
    transcript = []
    for turn in range(max_turns):
        response = client.beta.messages.create(
            model=model,
            max_tokens=16000,
            system=SYSTEM,
            tools=TOOLS,
            messages=messages,
            output_config={"effort": effort},
            betas=["server-side-fallback-2026-07-01"],
            fallbacks="default",
        )
        if response.stop_reason == "refusal":
            log({"event": "refusal", "details": str(getattr(response, "stop_details", None))})
            break
        messages.append({"role": "assistant", "content": response.content})
        text = _text(response.content)
        if text:
            transcript.append(text)
            log({"event": "assistant_text", "text": text})
        uses = [b for b in response.content if getattr(b, "type", "") == "tool_use"]
        if response.stop_reason == "max_tokens" and not uses:
            messages.append({"role": "user", "content": "Continue."})
            continue
        if not uses:
            if finished is None:
                messages.append({"role": "user", "content": "Call finish with your chosen parameter set."})
                continue
            break
        results = []
        for use in uses:
            args = use.input if isinstance(use.input, dict) else {}
            if use.name == "describe_search_space":
                out = {"parameters": describe(),
                       "current_values": {k: v for k, v in session.ev_defaults.items()},
                       "baseline": session.baseline,
                       "budget_left": session.budget - session.used}
            elif use.name == "evaluate_config":
                out = session.evaluate(args.get("params", {}), args.get("hypothesis", ""))
            elif use.name == "finish":
                clean, problems = validate_params(args.get("params", {}))
                if problems:
                    out = {"error": "; ".join(problems)}
                else:
                    finished = {"params": clean, "rationale": args.get("rationale", ""),
                                "further_ideas": args.get("further_ideas", "")}
                    out = {"ok": True}
            else:
                out = {"error": f"unknown tool {use.name}"}
            is_error = "error" in out
            results.append({"type": "tool_result", "tool_use_id": use.id, "content": json.dumps(out),
                            **({"is_error": True} if is_error else {})})
        messages.append({"role": "user", "content": results})
        if finished is not None:
            break
    if finished is None:  # model stopped without finish: take the best acceptable evaluation
        params, _ = session.best()
        finished = {"params": params, "rationale": "(the model did not call finish; best acceptable evaluation)",
                    "further_ideas": ""}
        finished["finished_by_model"] = False
    else:
        finished["finished_by_model"] = True
    finished["transcript"] = transcript
    return finished


def run_random(session, seed=0):
    """Baseline optimiser: random parameter sets with the same budget."""
    rng = random.Random(seed)
    while session.used < session.budget:
        params = {}
        for p in rng.sample(PARAMS, k=rng.randint(1, 3)):
            if p.kind == "bool":
                params[p.name] = rng.random() < 0.5
            elif p.kind == "int":
                params[p.name] = rng.randint(int(p.low), int(p.high))
            else:
                params[p.name] = round(rng.uniform(p.low, p.high), 3)
        session.evaluate(params, "random")
    params, _ = session.best()
    return {"params": params, "rationale": "random search: best acceptable evaluation", "further_ideas": "",
            "finished_by_model": False, "transcript": []}


__all__ = ["MODEL", "SYSTEM", "TOOLS", "TuningSession", "run_llm", "run_random", "acceptable", "BY_NAME"]
