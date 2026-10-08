"""Stage 5-4: NumPy MLP with LambdaRank and independent future regression.

Rank logits are comparable only WITHIN one decision. The regression predicts
additional volume / pallet capacity, lower-tail value, blocking and unserved
fractions; it is the only head used as the future value in final scoring.
"""

import json
from pathlib import Path
import numpy as np
from pac_common import FutureStats
from .features import FEATURE_NAMES, FEATURE_SCHEMA

OUTPUT_NAMES = ("mean", "cvar", "blocking_rate", "failure_rate")
MODEL_FORMAT = "ahead-dual-head-lambdarank-v1"


class DualHeadRanker:
    def __init__(self, payload):
        if (
            payload.get("format") != MODEL_FORMAT
            or payload.get("feature_schema") != FEATURE_SCHEMA
        ):
            raise ValueError(
                "Incompatible model schema; retraining is required"
            )
        if payload.get("feature_names") != list(FEATURE_NAMES) or payload.get(
            "output_names"
        ) != list(OUTPUT_NAMES):
            raise ValueError("Incompatible feature/output order")
        self.payload = payload
        names = ("mean", "scale", "w1", "b1", "wr", "br", "wv", "bv")
        for name in names:
            setattr(self, name, np.asarray(payload[name], dtype=float))
        f = len(FEATURE_NAMES)
        if self.w1.ndim != 2:
            raise ValueError("Bad hidden weights")
        h = self.w1.shape[1]
        shapes = {
            "mean": (f,),
            "scale": (f,),
            "w1": (f, h),
            "b1": (h,),
            "wr": (h,),
            "br": (),
            "wv": (h, 4),
            "bv": (4,),
        }
        if any(getattr(self, n).shape != shape for n, shape in shapes.items()):
            raise ValueError("Incompatible model shapes")
        if any(
            not np.isfinite(getattr(self, n)).all() for n in names
        ) or np.any(self.scale <= 0):
            raise ValueError("Invalid model values")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path):
        Path(path).write_text(
            json.dumps(self.payload, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )

    def predict(self, vectors):
        x = np.asarray([v.values for v in vectors], dtype=float)
        if (
            any(v.names != FEATURE_NAMES for v in vectors)
            or x.ndim != 2
            or x.shape[1] != len(FEATURE_NAMES)
        ):
            raise ValueError("Bad feature schema")
        h = np.tanh((x - self.mean) / self.scale @ self.w1 + self.b1)
        ranks = h @ self.wr + self.br
        values = np.clip(h @ self.wv + self.bv, 0.0, 1.0)
        if not np.isfinite(ranks).all() or not np.isfinite(values).all():
            raise ValueError("Non-finite model output")
        values[:, 1] = np.minimum(values[:, 1], values[:, 0])
        return [
            (
                float(r),
                FutureStats(
                    float(v[0]), None, float(v[1]), float(v[2]), float(v[3])
                ),
            )
            for r, v in zip(ranks, values)
        ]


def lambda_gradient(scores, relevance):
    """Pairwise RankNet derivative weighted by absolute NDCG swap change."""
    n = len(scores)
    gradient = np.zeros(n)
    if n < 2 or np.ptp(relevance) < 1e-10:
        return gradient
    # Continuous teacher utilities -> graded relevance, local to a query.
    rel = 3 * (relevance - np.min(relevance)) / np.ptp(relevance)
    gain = 2**rel - 1
    discount = 1 / np.log2(np.arange(n) + 2)
    ideal = float(np.sort(gain)[::-1] @ discount)
    order = np.argsort(-scores, kind="stable")
    rank = np.empty(n, dtype=int)
    rank[order] = np.arange(n)
    for i in range(n):
        for j in range(n):
            if rel[i] <= rel[j]:
                continue
            delta = abs(
                (gain[i] - gain[j]) * (discount[rank[i]] - discount[rank[j]])
            ) / max(ideal, 1e-12)
            rho = 1 / (1 + np.exp(np.clip(scores[i] - scores[j], -40, 40)))
            gradient[i] -= delta * rho
            gradient[j] += delta * rho
    return gradient


def train_model(groups, validation_groups, *, seed=42, epochs=100, hidden=32):
    if not groups or not validation_groups:
        raise ValueError(
            "Separate nonempty training and validation groups required"
        )
    train_ids = {g["base_group"] for g in groups}
    val_ids = {g["base_group"] for g in validation_groups}
    if train_ids & val_ids:
        raise ValueError("Inventory-group train/validation leakage")
    x = np.asarray(
        [r["features"] for g in groups for r in g["rows"]], dtype=float
    )
    y = np.asarray(
        [
            [r["future"][n] for n in OUTPUT_NAMES]
            for g in groups
            for r in g["rows"]
        ]
    )
    mean = x.mean(0)
    scale = np.maximum(x.std(0), 0.05)
    z = (x - mean) / scale
    vx = np.asarray(
        [r["features"] for g in validation_groups for r in g["rows"]],
        dtype=float,
    )
    vy = np.asarray(
        [
            [r["future"][n] for n in OUTPUT_NAMES]
            for g in validation_groups
            for r in g["rows"]
        ]
    )
    vz = (vx - mean) / scale
    rng = np.random.default_rng(seed)
    params = [
        rng.normal(0, 1 / np.sqrt(x.shape[1]), (x.shape[1], hidden)),
        np.zeros(hidden),
        rng.normal(0, 0.05, hidden),
        np.asarray(0.0),
        rng.normal(0, 0.05, (hidden, 4)),
        y.mean(0),
    ]
    offsets = np.cumsum([0] + [len(g["rows"]) for g in groups])
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    best = None
    best_loss = float("inf")
    history = []
    step = 0
    for epoch in range(epochs):
        for gi in rng.permutation(len(groups)):
            a, b = offsets[gi : gi + 2]
            batch = z[a:b]
            w1, b1, wr, br, wv, bv = params
            h = np.tanh(batch @ w1 + b1)
            rank = h @ wr + br
            values = h @ wv + bv
            truth = np.asarray(
                [r["teacher_score"] for r in groups[gi]["rows"]]
            )
            dr = lambda_gradient(rank, truth)
            dv = 2 * (values - y[a:b]) / max(1, values.size)
            dh = (dr[:, None] * wr[None, :] + dv @ wv.T) * (1 - h * h)
            grads = [
                batch.T @ dh,
                dh.sum(0),
                h.T @ dr,
                np.asarray(dr.sum()),
                h.T @ dv,
                dv.sum(0),
            ]
            step += 1
            for k, (p, grad) in enumerate(zip(params, grads)):
                grad = np.clip(grad + 1e-5 * p, -3, 3)
                m[k] = 0.9 * m[k] + 0.1 * grad
                v[k] = 0.999 * v[k] + 0.001 * grad * grad
                p -= (
                    0.002
                    * (m[k] / (1 - 0.9**step))
                    / (np.sqrt(v[k] / (1 - 0.999**step)) + 1e-8)
                )
        vh = np.tanh(vz @ params[0] + params[1])
        regression = float(np.mean((vh @ params[4] + params[5] - vy) ** 2))
        # Validation includes ranking regret, not training loss alone.
        ranks = vh @ params[2] + params[3]
        offset = 0
        regrets = []
        for group in validation_groups:
            rows = group["rows"]
            chosen = int(np.argmax(ranks[offset : offset + len(rows)]))
            regrets.append(
                max(r["teacher_score"] for r in rows)
                - rows[chosen]["teacher_score"]
            )
            offset += len(rows)
        loss = regression + float(np.mean(regrets))
        history.append(
            {
                "epoch": epoch + 1,
                "validation_mse": regression,
                "validation_regret": float(np.mean(regrets)),
            }
        )
        if loss < best_loss:
            best_loss = loss
            best = ([p.copy() for p in params], epoch + 1)
    p, selected_epoch = best
    payload = {
        "format": MODEL_FORMAT,
        "feature_schema": FEATURE_SCHEMA,
        "feature_names": list(FEATURE_NAMES),
        "output_names": list(OUTPUT_NAMES),
        "mean": mean.tolist(),
        "scale": scale.tolist(),
    }
    for name, value in zip(("w1", "b1", "wr", "br", "wv", "bv"), p):
        payload[name] = value.tolist()
    payload["training"] = {
        "seed": seed,
        "selected_epoch": selected_epoch,
        "train_base_groups": sorted(train_ids),
        "validation_base_groups": sorted(val_ids),
        "objective": "LambdaRank + independent future-outcome MSE",
    }
    return DualHeadRanker(payload), {
        "selected_epoch": selected_epoch,
        "history": history,
    }
