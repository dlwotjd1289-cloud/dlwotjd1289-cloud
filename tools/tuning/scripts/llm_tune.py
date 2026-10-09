#!/usr/bin/env python3
"""LLM-assisted tuning of the rule / generation parameters.

    export ANTHROPIC_API_KEY=...            # or `ant auth login`
    python tools/tuning/scripts/llm_tune.py --dataset tools/highlevel/output/dataset80_x40_s8 \
        --optimizer llm --budget 12 --output tools/tuning/output/llm

    # same budget, random search (baseline for judging the LLM; no API key needed)
    python tools/tuning/scripts/llm_tune.py --dataset ... --optimizer random --budget 12 --output tools/tuning/output/random

The optimiser only sees the validation split. The chosen set is then
evaluated once on the test split (paired with the current configuration);
report.json / report.md hold both, plus the model's reasoning.
"""

import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2] / "highlevel" / "scripts"))
sys.path.insert(0, str(HERE.parents[1]))
from _common import add_common_args, load_all  # noqa: E402

from tuning import Evaluator, default_params  # noqa: E402
from tuning.agent import MODEL, TuningSession, run_llm, run_random  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    parser.add_argument("--optimizer", choices=("llm", "random"), default="llm")
    parser.add_argument("--budget", type=int, default=12, help="validation evaluations")
    parser.add_argument("--val-split", default="val")
    parser.add_argument("--val-passes", type=int, default=1)
    parser.add_argument("--test-split", default="test")
    parser.add_argument("--test-passes", type=int, default=3)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--model", default=MODEL)
    parser.add_argument("--effort", default="high", choices=("low", "medium", "high", "xhigh", "max"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    log_path = args.output / "log.jsonl"
    log_file = log_path.open("w", encoding="utf-8")

    def log(row):
        log_file.write(json.dumps(row, ensure_ascii=False, default=str) + "\n")
        log_file.flush()
        if row.get("event") == "evaluate":
            r = row["result"]
            print(f"eval {row['params']} -> pallets {r['pallets_mean']} "
                  f"(diff {r['vs_baseline']['pallets_diff_mean']:+.3f}, p {r['vs_baseline']['sign_test_p']}) "
                  f"ok={r['acceptable']}", flush=True)
        elif row.get("event") == "assistant_text":
            print("model:", row["text"][:400].replace("\n", " "), flush=True)

    dataset, cand, vcfg, hl = load_all(args)
    val = Evaluator(dataset, cand, vcfg, hl, split=args.val_split, passes=args.val_passes, workers=args.workers)
    session = TuningSession(val, args.budget, log=log, defaults=default_params(cand, hl))
    print("baseline (val):", json.dumps(session.baseline), flush=True)
    if args.optimizer == "llm":
        choice = run_llm(session, model=args.model, effort=args.effort, log=log)
    else:
        choice = run_random(session, seed=args.seed)
    print("chosen:", choice["params"], flush=True)

    test = Evaluator(dataset, cand, vcfg, hl, split=args.test_split, passes=args.test_passes, workers=args.workers)
    test_result = test.evaluate(choice["params"])
    report = {
        "optimizer": args.optimizer, "model": args.model if args.optimizer == "llm" else None,
        "budget": args.budget, "evaluations_used": session.used,
        "val_split": args.val_split, "val_episodes": val.episodes,
        "test_split": args.test_split, "test_episodes": test.episodes,
        "baseline_params": default_params(cand, hl),
        "val_baseline": session.baseline,
        "val_history": [{"params": p, "result": r} for p, r in session.history],
        "chosen": {k: choice[k] for k in ("params", "rationale", "further_ideas", "finished_by_model")},
        "test_baseline": test.evaluate({}), "test_chosen": test_result,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=1, ensure_ascii=False, default=str),
                                              encoding="utf-8")
    t = test_result["vs_baseline"]
    md = [f"# Tuning report ({args.optimizer})", "",
          f"- validation: {val.episodes} episodes, {session.used}/{args.budget} evaluations",
          f"- chosen parameters: `{json.dumps(choice['params'], ensure_ascii=False)}`",
          f"- test ({test.episodes} episodes): pallets {report['test_baseline']['pallets_mean']} -> "
          f"{test_result['pallets_mean']} (diff {t['pallets_diff_mean']:+.3f}, 95 % CI {t['ci95']}, "
          f"better/equal/worse {t['better']}/{t['equal']}/{t['worse']}, sign-test p {t['sign_test_p']})",
          "", "## Rationale", "", choice.get("rationale", ""), "", "## Further ideas", "",
          choice.get("further_ideas", ""), "", "## Validation history", "",
          "| # | params | pallets | diff | p | ok |", "|---|---|---|---|---|---|"]
    for i, (p, r) in enumerate(session.history, 1):
        v = r["vs_baseline"]
        md.append(f"| {i} | `{json.dumps(p)}` | {r['pallets_mean']} | {v['pallets_diff_mean']:+.3f} | "
                  f"{v['sign_test_p']} | {r['acceptable']} |")
    (args.output / "report.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md[:6]))
    log_file.close()


if __name__ == "__main__":
    main()
