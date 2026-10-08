from dataclasses import dataclass


@dataclass(frozen=True)
class PalletizerCommand:
    """Robot-independent placement command used by AHEAD -> robot adapter.

    AHEAD intentionally exposes only x/y/z/yaw for the palletizing task.
    Robot-specific adapters may use more joints internally, but must not leak
    that extra freedom back into the core contract.
    """

    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
