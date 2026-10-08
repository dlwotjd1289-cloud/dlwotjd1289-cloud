#!/usr/bin/env python3
"""PAC 2026: stand-alone pallet physics demonstration.

Uses existing AHEAD PyBullet server; it does not run Gazebo or a learned planner.
Six-box mode uses the built-in sequence. Mixed20 delegates to the repo's
existing deterministic height-map baseline demo.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]


class DemoError(RuntimeError):
    pass


def api(url: str, method: str, path: str, payload: dict | None = None) -> dict:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url.rstrip("/") + path,
        data=body,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as response:
            data = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        description = exc.read().decode("utf-8", errors="replace")
        raise DemoError(f"HTTP {exc.code} at {path}: {description}") from exc
    except (urllib.error.URLError, TimeoutError) as exc:
        raise DemoError(
            f"서버 연결 실패: {url}. 먼저 Terminal 4에서 "
            "python3 scripts/run_ahead_simulator.py 를 실행하세요."
        ) from exc
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise DemoError(f"Invalid JSON from {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise DemoError(f"Unexpected response from {path}")
    if data.get("ok") is False:
        raise DemoError(f"API error at {path}: {data.get('error', data)}")
    return data


def print_metrics(state: dict, *, label: str) -> None:
    m = state.get("metrics", {})
    print(
        f"{label:<19} "
        f"박스 {m.get('on_pallet_box_count', '?')}/{m.get('box_count', '?')} | "
        f"하중 {float(m.get('on_pallet_mass_kg', 0)):.1f} kg | "
        f"적재 높이 {float(m.get('current_height_m', 0)):.3f} m | "
        f"체적 사용률 {float(m.get('allowed_volume_utilization', 0)) * 100:.1f}% | "
        f"CoM 편차 {float(m.get('com_xy_offset_m', 0)) * 1000:.1f} mm | "
        f"이동 {m.get('moving_box_count', '?')} | "
        f"팔레트 밖 {m.get('outside_pallet_box_count', '?')}"
    )


def wait_and_state(url: str, delay: float) -> dict:
    if delay > 0:
        time.sleep(delay)
    return api(url, "GET", "/api/state")


def run_builtin(url: str, count: int, delay: float, settle: float) -> dict:
    api(url, "POST", "/api/reset", {})
    print("[RESET] 팔레트와 데모 순서 초기화")
    for i in range(1, count + 1):
        result = api(url, "POST", "/api/demo/next", {})
        placed = result.get("placed")
        if not placed:
            raise DemoError(f"기본 데모가 {i}번째 박스를 반환하지 않았습니다.")
        print(
            f"[{i}/{count}] {placed['id']} | "
            f"크기 {placed['size_m']} m | "
            f"질량 {placed['mass_kg']} kg"
        )
        state = wait_and_state(url, delay)
        print_metrics(state, label="  현재 물리 상태")
    print(f"[SETTLE] {settle:.1f}초 동안 물리 안정화 관찰")
    state = wait_and_state(url, settle)
    print_metrics(state, label="최종 물리 상태")
    return state


def run_mixed20(url: str, seed: int, count: int, delay: float, settle: float) -> dict:
    script = ROOT / "scripts" / "run_random_20_box_scenario.py"
    if not script.is_file():
        raise DemoError(f"기존 랜덤박스 시나리오 파일을 찾을 수 없습니다: {script}")
    command = [
        sys.executable, str(script),
        "--url", url, "--seed", str(seed), "--count", str(count),
        "--delay", str(delay), "--settle", str(settle),
    ]
    print("[MIXED] 기존 height-map baseline 배치 시나리오 실행", flush=True)
    print(f"[MIXED] seed={seed}, boxes={count}, delay={delay}", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)
    return api(url, "GET", "/api/state")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Pallet-only AHEAD PyBullet 실시간 3D 데모"
    )
    parser.add_argument("--mode", choices=("showcase6", "stable5", "mixed20"),
                        default="showcase6")
    parser.add_argument("--url", default="http://127.0.0.1:4173")
    parser.add_argument("--delay", type=float, default=1.7,
                        help="박스 투입 간 실제 대기시간 (초)")
    parser.add_argument("--settle", type=float, default=3.0,
                        help="마지막 박스 이후 관찰시간 (초)")
    parser.add_argument("--seed", type=int, default=20261008)
    parser.add_argument("--count", type=int, default=20,
                        help="mixed20에서 생성할 박스 수")
    parser.add_argument("--output", type=Path, default=None,
                        help="최종 상태 JSON 저장 경로")
    args = parser.parse_args()

    if args.delay < 0 or args.settle < 0 or args.count < 1:
        parser.error("delay/settle은 0 이상, count는 1 이상이어야 합니다.")

    print("=" * 64)
    print("PAC2026 | PyBullet 팔레트 단독 물리 시연")
    print(f"모드: {args.mode} | Viewer: {args.url}")
    print("주의: 박스 배치는 데모/기준선 방식입니다. 학습된 AHEAD AI가 아닙니다.")
    print("=" * 64, flush=True)
    try:
        state = api(args.url, "GET", "/api/state")
        if state.get("type") != "state" or "pallet" not in state:
            raise DemoError("연결된 주소가 AHEAD PyBullet 시뮬레이터가 아닙니다.")

        if args.mode == "showcase6":
            state = run_builtin(args.url, 6, args.delay, args.settle)
        elif args.mode == "stable5":
            state = run_builtin(args.url, 5, args.delay, args.settle)
        else:
            state = run_mixed20(args.url, args.seed, args.count,
                                args.delay, args.settle)

        out = args.output
        if out is None:
            name = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            out = ROOT / "artifacts" / f"pallet_demo_{args.mode}_{name}.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(state, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[DONE] 최종 상태 저장: {out}")
        if args.mode == "showcase6":
            print("마지막 B006_UNSTABLE은 의도적으로 불안정하게 투입됩니다.")
            print("실제로 넘어지는지/떨어지는지는 PyBullet 결과로 확인하세요.")
        print("브라우저 화면은 서버를 종료하기 전까지 유지됩니다.")
    except (DemoError, subprocess.CalledProcessError, OSError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
