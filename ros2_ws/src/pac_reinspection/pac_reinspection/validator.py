"""Stage 2 extension: reinspection (RI) reasons on top of pac_runtime's StateValidator.

The wrapped ``StateValidator`` is not modified; its Anomaly kinds, routes and
reject codes stay as they are. This layer

1. checks raw stage-1 signals that ``RawObservation`` does not carry
   (``PerceptionSignals``) and asks stage 1 to sense again through
   ``reacquire(kind, box_id, attempt) -> Reacquired | None`` within per-family
   retry limits and a per-box time budget,
2. runs ``StateValidator.validate`` on the re-sensed observation (again after a
   re-read / re-weigh / re-capture triggered by its result),
3. records RI codes as ``"RI-xxx: ..."`` notes, and
4. applies the few routes the policy adds: unresolved C5 / C8 / A6 / A7 and the
   F6 loop guard go to INSPECTION; unresolved sensor doubts stay PLAN with
   ``uncertain=True`` (delta) and ``SENSOR_UNCERTAIN`` in the codes.

``validate(obs, perception, field_box)`` keeps the StateValidator signature, so
``RuntimeCore`` could use this class unchanged (proposal; see the docs).
"""
from __future__ import annotations

import math
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass, replace

from pac_common import RejectCode, Size3D
from pac_perception.signals import PerceptionSignals
from pac_runtime.state_validator import Anomaly, Verdict

from .policy import ReinspectionPolicy


@dataclass(frozen=True)
class Reacquired:
    """What stage 1 returns after sensing again (None fields: unchanged)."""
    obs: object = None
    signals: PerceptionSignals | None = None


@dataclass(frozen=True)
class ReinspectionResult:
    verdict: Verdict
    ri_codes: tuple
    attempts: dict
    elapsed_s: float


class ReinspectionValidator:
    def __init__(self, base, policy: ReinspectionPolicy, reacquire=None, remaining=None):
        """``base``: pac_runtime StateValidator. ``remaining``: mapping or callable
        returning ``remaining_by_sku`` (for RI-A6); None disables A6."""
        self.base = base
        self.policy = policy
        self.reacquire = reacquire
        self.remaining = remaining
        self.history = defaultdict(list)     # box_id -> verdict kinds over all calls (RI-F6)
        self.last: ReinspectionResult | None = None

    def validate(self, obs, perception=None, field_box=None, signals: PerceptionSignals | None = None):
        run = _Run(self, obs, signals or PerceptionSignals())
        verdict = run.execute(perception, field_box)
        self.last = ReinspectionResult(verdict, tuple(c for c, _ in run.ri if c.startswith("RI-")),
                                       dict(run.attempts), run.elapsed)
        return verdict


class _CountingPerception:
    def __init__(self, inner):
        self.inner = inner
        self.calls = 0

    def base_view(self, field_box, previous):
        self.calls += 1
        return self.inner.base_view(field_box, previous)


def _yaw_deviation(yaw: float) -> float:
    """Distance to the nearest multiple of 90 deg (long side along or across the conveyor)."""
    q = math.pi / 2
    return abs((yaw + q / 2) % q - q / 2)


class _Run:
    def __init__(self, owner: ReinspectionValidator, obs, signals: PerceptionSignals):
        self.o, self.p, self.obs, self.sig = owner, owner.policy, obs, signals
        self.attempts = Counter()
        self.elapsed = 0.0
        self.ri = []                 # (code, text)
        self.sensor_uncertain = False
        self.force = None            # (Anomaly, codes, code)
        self.kinds = []

    # ---------------------------------------------------------------- helpers
    def tag(self, code, text):
        if code not in (c for c, _ in self.ri):
            self.ri.append((code, text))

    def retry(self, kind, family, code):
        p = self.p
        if not p.enabled(code) or self.o.reacquire is None or self.attempts[family] >= p.retries[family]:
            return None
        if self.elapsed + p.costs_s[kind] > p.total_time_s:
            self.tag("budget", f"재인식 시간 예산 {p.total_time_s:.0f} s 소진")
            return None
        self.attempts[family] += 1
        self.elapsed += p.costs_s[kind]
        r = self.o.reacquire(kind, self.obs.box_id, self.attempts[family])
        if r is None:
            return None
        if r.obs is not None:
            self.obs = r.obs
        if r.signals is not None:
            self.sig = r.signals
        return r

    def resolve(self, code, text, bad, kind, family):
        """Re-sense while ``bad()`` holds; tag the code with the outcome. True when resolved."""
        if not self.p.enabled(code) or not bad():
            return True
        n0 = self.attempts[family]
        while bad():
            if self.retry(kind, family, code) is None:
                self.tag(code, f"{text} → 미해소 ({self.attempts[family] - n0}회 재시도)")
                return False
        self.tag(code, f"{text} → 해소 ({self.attempts[family] - n0}회 재시도)")
        return True

    def spec(self, sku):
        return self.o.base.catalog.get(sku)

    def remaining(self):
        r = self.o.remaining
        return r() if callable(r) else r

    def size_bad(self, measured, nominal):
        """StateValidator's size rule (ValidatorConfig tolerances, x/y may swap)."""
        cfg = self.o.base.cfg
        tol = lambda v: max(cfg.size_tolerance_m, cfg.size_tolerance_ratio * v)  # noqa: E731
        m = (measured.x, measured.y, measured.z)
        return not any(all(abs(a - b) <= tol(b) for a, b in zip(m, n))
                       for n in ((nominal.x, nominal.y, nominal.z), (nominal.y, nominal.x, nominal.z)))

    def fits_any_sku(self, size):
        return any(not self.size_bad(size, s.size) for s in self.o.base.catalog.values())

    def merged_pair(self, size):
        """Footprint matches no SKU but two SKUs placed end to end along the conveyor."""
        tol = self.p.param("RI-C5", "pair_tolerance_m")
        length, width = max(size.x, size.y), min(size.x, size.y)
        sides = [((s.size.x, s.size.y), (s.size.y, s.size.x)) for s in self.o.base.catalog.values()]
        if self.fits_any_sku(size):
            return False
        for a in sides:
            for b in sides:
                for la, wa in a:
                    for lb, wb in b:
                        if abs(length - (la + lb)) <= tol and abs(width - max(wa, wb)) <= tol:
                            return True
        return False

    def frame_spread(self):
        f = self.sig.frame_sizes
        if len(f) < 2:
            return 0.0
        return max(max(getattr(s, a) for s in f) - min(getattr(s, a) for s in f) for a in "xyz")

    # ---------------------------------------------------------------- main
    def execute(self, perception, field_box):
        self.pre_checks()
        verdict = self.validate_base(perception, field_box)
        verdict = self.post_checks(verdict, perception, field_box)
        if not (self.force and self.force[2] == "RI-F6"):
            self.loop_guard()                       # flips across calls for the same box id
        return self.finish(verdict)

    def pre_checks(self):
        p, s = self.p, lambda: self.sig
        # scale (B7 zero, B6 settle)
        if not self.resolve("RI-B7", "저울 영점 이상", lambda: s().tare_ok is False, "rezero", "weigh"):
            self.sensor_uncertain = True
        if p.enabled("RI-B6"):
            unstable = lambda: s().scale_settled is False or (  # noqa: E731
                s().scale_stdev_kg is not None and s().scale_stdev_kg > p.param("RI-B6", "max_stdev_kg"))
            if not self.resolve("RI-B6", "저울 미안정", unstable, "reweigh", "weigh"):
                self.sensor_uncertain = True
        # label payload (A3)
        if not self.resolve("RI-A3", "라벨 부분 판독/체크섬 불일치",
                            lambda: self.obs.label_sku is not None and s().label_payload_ok is False, "relabel", "label"):
            self.obs = replace(self.obs, label_sku=None)
        # top vs second-view label (A7): third read, majority
        top, other = self.obs.label_sku, s().base_label_sku
        if p.enabled("RI-A7") and top is not None and other is not None and top != other:
            r = self.retry("relabel", "label", "RI-A7")
            votes = Counter([top, other] + ([r.obs.label_sku] if r and r.obs is not None and r.obs.label_sku else []))
            label, n = votes.most_common(1)[0]
            if n >= 2:
                self.obs = replace(self.obs, label_sku=label)
                self.tag("RI-A7", f"상단 {top} / 보조 {other} 불일치 → 다수결 {label}")
            else:
                self.tag("RI-A7", f"상단 {top} / 보조 {other} 불일치, 다수결 실패")
                self.force = (Anomaly.RECOGNITION_FAIL, (RejectCode.TRACKING_LOST,), "RI-A7")
        # perception quality (C3, C7, C2)
        if not self.resolve("RI-C3", "윤곽 일부 가림/시야 경계", lambda: s().in_full_view is False, "recapture", "frames"):
            self.sensor_uncertain = True
        if p.enabled("RI-C7"):
            low = lambda: s().valid_ratio is not None and s().valid_ratio < p.param("RI-C7", "min_valid_ratio")  # noqa: E731
            if not self.resolve("RI-C7", "깊이 결측률 초과", low, "recapture", "frames"):
                self.sensor_uncertain = True
        if p.enabled("RI-C2"):
            spread = lambda: self.frame_spread() > p.param("RI-C2", "max_size_spread_m")  # noqa: E731
            if not self.resolve("RI-C2", "프레임 간 크기 분산 초과", spread, "frames", "frames"):
                self.sensor_uncertain = True
            f = self.sig.frame_sizes
            if len(f) >= 2:
                self.obs = replace(self.obs, size=Size3D(*(statistics.median(getattr(v, a) for v in f) for a in "xyz")))
        # arrival pose (C8)
        if p.enabled("RI-C8"):
            off = lambda: s().arrival_yaw_rad is not None and _yaw_deviation(s().arrival_yaw_rad) > p.param("RI-C8", "max_yaw_deviation_rad")  # noqa: E731
            if not self.resolve("RI-C8", "도착 자세 범위 밖", off, "recapture", "frames"):
                self.force = self.force or (Anomaly.RECOGNITION_FAIL, (RejectCode.SENSOR_UNCERTAIN,), "RI-C8")
        # two boxes seen as one (C5)
        if p.enabled("RI-C5") and not self.resolve("RI-C5", "두 박스가 하나로 인식", lambda: self.merged_pair(self.obs.size), "separate", "frames"):
            self.force = self.force or (Anomaly.SPEC_MISMATCH, (RejectCode.SIZE_MISMATCH,), "RI-C5")
        # damage cue near the threshold (D2)
        if p.enabled("RI-D2"):
            lo, hi = p.param("RI-D2", "ambiguous_band")
            near = lambda: s().damage_score is not None and lo <= s().damage_score < hi  # noqa: E731
            if self.resolve("RI-D2", "파손 판단 경계값 부근", near, "second_view", "frames"):
                if self.sig.damage_score is not None and any(c == "RI-D2" for c, _ in self.ri):
                    self.obs = replace(self.obs, visual_damage=self.sig.damage_score >= hi)
            else:
                self.obs = replace(self.obs, visual_damage=True)

    def validate_base(self, perception, field_box):
        counting = _CountingPerception(perception) if perception is not None else None
        verdict = self.o.base.validate(self.obs, counting, field_box)
        if counting is not None:
            self.attempts["base_view"] += counting.calls
        self.kinds.append(verdict.kind)
        return verdict

    def post_checks(self, verdict, perception, field_box):
        base_cfg = self.o.base.cfg
        if self.obs.label_sku is None:
            self.tag("RI-A1", "라벨 미판독 → Base-view")
        elif self.obs.confidence < base_cfg.min_label_confidence:
            self.tag("RI-A2", f"확신도 {self.obs.confidence:.2f} < {base_cfg.min_label_confidence} → Base-view")
        for _ in range(6):                            # each round needs a successful re-sensing
            step = self.post_step(verdict)
            if step is None:
                break
            code, kind, family, text = step
            if self.retry(kind, family, code) is None:
                self.tag(code, text + " → 재시도 불가/상한")
                break
            self.tag(code, text + " → 재시도")
            verdict = self.validate_base(perception, field_box)
            if self.loop_guard():
                break
        return self.post_tags(verdict)

    def post_step(self, verdict):
        """Next re-sensing the base verdict calls for, or None."""
        p = self.p
        if verdict.kind == Anomaly.UNKNOWN_SKU and p.enabled("RI-A4") and self.attempts["label"] < p.retries["label"]:
            return "RI-A4", "relabel", "label", f"catalog에 없는 SKU {verdict.sku}"
        if verdict.kind == Anomaly.SPEC_MISMATCH:
            codes = set(verdict.codes)
            if {RejectCode.SIZE_MISMATCH, RejectCode.WEIGHT_MISMATCH} <= codes and p.enabled("RI-B3") and self.attempts["label"] < p.retries["label"]:
                return "RI-B3", "relabel", "label", "크기·무게 동시 불일치 → 라벨 재판독"
            if codes == {RejectCode.WEIGHT_MISMATCH} and p.enabled("RI-B2") and self.attempts["weigh"] < p.retries["weigh"]:
                return "RI-B2", "reweigh", "weigh", "무게 범위 밖 → 재계량"
            if codes == {RejectCode.SIZE_MISMATCH} and self.z_only(verdict) and p.enabled("RI-B4") and self.attempts["frames"] < p.retries["frames"]:
                return "RI-B4", "recapture", "frames", "높이만 불일치 → 재촬영"
        rem = self.remaining()
        if (verdict.route == "PLAN" and rem is not None and verdict.sku is not None and rem.get(verdict.sku, 0) <= 0
                and p.enabled("RI-A6") and self.attempts["label"] < p.retries["label"]):
            return "RI-A6", "relabel", "label", f"{verdict.sku} 잔여 수량 0"
        return None

    def z_only(self, verdict):
        spec = self.spec(verdict.sku)
        if spec is None:
            return False
        flat = replace(self.obs.size, z=spec.size.z)
        return not self.size_bad(flat, spec.size)

    def loop_guard(self):
        if not self.p.enabled("RI-F6"):
            return False
        kinds = self.o.history[self.obs.box_id] + self.kinds
        changes = sum(1 for a, b in zip(kinds, kinds[1:]) if a != b)
        if changes >= self.p.max_verdict_changes:
            self.tag("RI-F6", f"판정이 {changes}회 바뀜 → INSPECTION")
            self.force = (None, (RejectCode.SENSOR_UNCERTAIN,), "RI-F6")
            return True
        return False

    def post_tags(self, verdict):
        p, codes = self.p, set(verdict.codes)
        if verdict.kind == Anomaly.UNKNOWN_SKU:
            self.tag("RI-A4", f"catalog에 없는 SKU {verdict.sku} → INSPECTION")
        if verdict.kind == Anomaly.DAMAGED:
            self.tag("RI-D1", f"파손 → {verdict.route}")
        if RejectCode.SIZE_MISMATCH in codes and RejectCode.WEIGHT_MISMATCH in codes:
            self.tag("RI-B3", "크기·무게 동시 불일치 → 실측값 + δ")
        elif RejectCode.SIZE_MISMATCH in codes:
            if self.z_only(verdict):
                self.tag("RI-B4", "높이만 불일치 → 실측값 + δ")
            else:
                self.tag("RI-B1", "크기 불일치 → 실측값 + δ")
        elif RejectCode.WEIGHT_MISMATCH in codes:
            self.tag("RI-B2", "무게 범위 밖 → 실측값 + δ")
        if RejectCode.WEIGHT_MISMATCH in codes and p.enabled("RI-D3"):
            lo = self.o.base.weight_ranges.get(verdict.sku, (None, None))[0]
            if lo is not None and self.obs.weight_kg < lo * (1 - self.o.base.cfg.weight_tolerance_ratio):
                self.tag("RI-D3", "무게가 범위 아래: 내용물 이탈 의심 (DAMAGED 재분류는 팀 결정)")
        spec = self.spec(verdict.sku)
        if verdict.route == "PLAN" and spec is not None and p.enabled("RI-B5"):
            m, n = self.obs.size, spec.size
            if abs(m.x - n.y) < abs(m.x - n.x) and not self.size_bad(m, n):
                self.tag("RI-B5", "x/y 뒤바뀜 (정상, 기록만)")
        rem = self.remaining()
        if verdict.route == "PLAN" and rem is not None and verdict.sku is not None and rem.get(verdict.sku, 0) <= 0 and p.enabled("RI-A6"):
            self.tag("RI-A6", f"{verdict.sku} 잔여 수량 0 → INSPECTION")
            self.force = self.force or (Anomaly.UNKNOWN_SKU, (RejectCode.INVALID_STATE,), "RI-A6")
        if verdict.route == "PLAN" and verdict.uncertain and self.obs.confidence < 1.0:
            self.tag("RI-C1", f"확신도 {self.obs.confidence:.2f} → δ")
        return verdict

    def finish(self, verdict):
        self.o.history[self.obs.box_id].extend(self.kinds)
        notes = list(verdict.notes)
        if self.force is not None:
            kind, codes, code = self.force
            verdict = replace(verdict, kind=kind or verdict.kind, route="INSPECTION", box=None, uncertain=False,
                              no_load_on_top=False, codes=tuple(dict.fromkeys(tuple(verdict.codes) + codes)))
        elif self.sensor_uncertain and verdict.route == "PLAN":
            verdict = replace(verdict, uncertain=True,
                              codes=tuple(dict.fromkeys(tuple(verdict.codes) + (RejectCode.SENSOR_UNCERTAIN,))))
            if verdict.box is not None:          # like StateValidator's uncertain OK: measured size
                verdict = replace(verdict, box=replace(verdict.box, size=self.obs.size))
        notes += [f"{code}: {text}" for code, text in self.ri]
        return replace(verdict, notes=notes)
