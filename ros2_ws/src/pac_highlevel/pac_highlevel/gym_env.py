"""Gymnasium wrapper (sb3-contrib ``MaskablePPO`` compatible).

```python
from sb3_contrib import MaskablePPO          # where PyTorch is available
env = HighLevelGymEnv(make_world)
model = MaskablePPO("MlpPolicy", env).learn(200_000)
```

``action_masks()`` is the hook sb3-contrib's ``ActionMasker`` /
``MaskablePPO`` look for. The same world, features and masks are used by the
NumPy trainer in this package.
"""

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover - optional dependency
    gym = None

from .actions import action_count, from_index
from .features import feature_names, observe

if gym is not None:

    class HighLevelGymEnv(gym.Env):
        metadata = {"render_modes": []}

        def __init__(self, make_world, slots):
            super().__init__()
            self.make_world = make_world
            self.slots = slots
            n = len(feature_names(slots))
            self.observation_space = spaces.Box(-np.inf, np.inf, shape=(n,), dtype=np.float32)
            self.action_space = spaces.Discrete(action_count(slots))
            self.world = None
            self.episode = -1

        def reset(self, *, seed=None, options=None):
            super().reset(seed=seed)
            self.episode += 1
            self.world = self.make_world(self.episode)
            if self.world.done:  # nothing to decide (e.g. every box went to NG)
                return observe(self.world), {"summary": self.world.summary()}
            return observe(self.world), {}

        def step(self, action):
            reward = self.world.step(from_index(action, self.slots))
            done = self.world.done
            info = {"summary": self.world.summary()} if done else {}
            return observe(self.world), float(reward), done, False, info

        def action_masks(self):
            return self.world.action_mask()

else:  # pragma: no cover
    HighLevelGymEnv = None
