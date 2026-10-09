"""Episode roll-out shared by scripts, tools and tests.

``run_policy(world, choose)`` plays one :class:`PalletizingWorld` episode
with ``choose(world) -> HighLevelAction`` (Rule, Greedy or Lookahead).
"""


def run_policy(world, choose):
    """Roll one episode with ``choose(world) -> HighLevelAction``."""
    total = 0.0
    while not world.done:
        total += world.step(choose(world))
    out = world.summary()
    out["return"] = total
    return out


__all__ = ["run_policy"]
