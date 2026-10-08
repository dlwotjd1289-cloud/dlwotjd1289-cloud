"""PyTorch backend: sb3-contrib ``MaskablePPO`` on the same world and features.

Optional (``pip install torch sb3-contrib gymnasium``). The NumPy
implementation in ``ppo.py`` stays as the dependency-free fallback; both use
the identical world, observation, masks and rule-teacher warm start, so their
results are directly comparable.

Files: ``<name>.zip`` (sb3 model) + ``<name>.contract.json`` (feature layout
and value-provider contract, checked on load like the NumPy policy).
"""

import json
from pathlib import Path

import numpy as np

from .actions import action_count, from_index
from .features import feature_names, observe
from .gym_env import HighLevelGymEnv

FORMAT = "pac_highlevel.sb3_maskable_ppo.v1"


def _require():
    try:
        import sb3_contrib  # noqa: F401
        import torch
    except ImportError as error:  # pragma: no cover - depends on the install
        raise RuntimeError("needs PyTorch: pip install torch sb3-contrib gymnasium") from error
    # tiny MLP: one thread is fastest and keeps forked workers safe
    torch.set_num_threads(1)


def make_vec_env(make_world, slots, n_envs=4, subprocess=True):
    """``n_envs`` environments; env k plays episodes k, k + n, k + 2n, ..."""
    _require()
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

    def env_fn(k):
        def make():
            return HighLevelGymEnv(lambda i: make_world(k + i * n_envs), slots)

        return make

    fns = [env_fn(k) for k in range(n_envs)]
    if subprocess and n_envs > 1:
        return SubprocVecEnv(fns, start_method="fork")
    return DummyVecEnv(fns)


def new_model(env, ppo_config, seed=None, verbose=0):
    _require()
    from sb3_contrib import MaskablePPO

    cfg = ppo_config
    return MaskablePPO(
        "MlpPolicy",
        env,
        learning_rate=cfg.learning_rate,
        n_steps=max(1, cfg.n_steps // env.num_envs),
        batch_size=cfg.batch_size,
        n_epochs=cfg.n_epochs,
        gamma=cfg.gamma,
        gae_lambda=cfg.gae_lambda,
        clip_range=cfg.clip_range,
        ent_coef=cfg.entropy_coef,
        vf_coef=cfg.value_coef,
        max_grad_norm=cfg.max_grad_norm,
        policy_kwargs={"net_arch": {"pi": list(cfg.hidden), "vf": list(cfg.hidden)}},
        seed=cfg.seed if seed is None else seed,
        verbose=verbose,
        device="cpu",
    )


def imitate(model, data, epochs=20, batch_size=128, lr=1e-3):
    """Rule-teacher warm start: masked cross-entropy + value regression."""
    _require()
    import torch

    policy = model.policy
    opt = torch.optim.Adam(policy.parameters(), lr=lr)
    obs = torch.as_tensor(data["obs"], dtype=torch.float32)
    masks = data["masks"]
    actions = torch.as_tensor(data["actions"], dtype=torch.long)
    returns = torch.as_tensor(data["returns"], dtype=torch.float32)
    n = len(actions)
    rng = np.random.default_rng(0)
    hist = []
    for _ in range(epochs):
        order = rng.permutation(n)
        losses, accs = [], []
        for start in range(0, n, batch_size):
            idx = order[start : start + batch_size]
            dist = policy.get_distribution(obs[idx], action_masks=masks[idx])
            logp = dist.log_prob(actions[idx])
            values = policy.predict_values(obs[idx]).squeeze(-1)
            loss = -logp.mean() + 0.5 * torch.nn.functional.mse_loss(values, returns[idx])
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 0.5)
            opt.step()
            with torch.no_grad():
                probs = dist.distribution.probs
                accs.append(float((probs.argmax(-1) == actions[idx]).float().mean()))
            losses.append(float(-logp.mean().detach()))
        hist.append({"phase": "imitation", "bc_loss": float(np.mean(losses)),
                     "bc_accuracy": float(np.mean(accs))})
    return hist


def save(model, path, slots, contract, extra=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(path.with_suffix("")))
    meta = {"format": FORMAT, "feature_names": list(feature_names(slots)),
            "n_actions": action_count(slots), "contract": contract, **(extra or {})}
    path.with_suffix(".contract.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")


def load(path, *, slots, contract=None):
    _require()
    from sb3_contrib import MaskablePPO

    path = Path(path)
    meta = json.loads(path.with_suffix(".contract.json").read_text(encoding="utf-8"))
    if meta.get("format") != FORMAT:
        raise ValueError("not a pac_highlevel sb3 policy")
    if tuple(meta["feature_names"]) != feature_names(slots):
        raise ValueError("feature layout differs from the trained policy")
    for key, value in (contract or {}).items():
        if meta["contract"].get(key) != value:
            raise ValueError(f"policy contract mismatch on {key}: trained "
                             f"{meta['contract'].get(key)!r}, deployed {value!r}")
    return MaskablePPO.load(str(path.with_suffix("")), device="cpu")


def chooser(model, deterministic=True):
    def choose(world):
        action, _ = model.predict(observe(world), action_masks=world.action_mask(),
                                  deterministic=deterministic)
        return from_index(int(action), world.slots)

    return choose
