import math
from .models import BoxState, PlacementCandidate, Pose3D, Size3D, SystemState

def _finite(*v): return all(math.isfinite(x) for x in v)

def validate_size(s: Size3D):
    if not _finite(s.x,s.y,s.z): raise ValueError('size contains NaN/inf')
    if min(s.x,s.y,s.z) <= 0: raise ValueError('size.x/y/z must be > 0')

def validate_pose(p: Pose3D):
    if not p.frame_id: raise ValueError('frame_id must not be empty')
    if not _finite(p.x,p.y,p.z,p.roll,p.pitch,p.yaw): raise ValueError('pose contains NaN/inf')

def validate_box(b: BoxState):
    validate_size(b.size); validate_pose(b.pose)
    if not math.isfinite(b.weight_kg) or b.weight_kg < 0: raise ValueError('weight_kg must be finite and >= 0')
    if not math.isfinite(b.confidence) or not 0 <= b.confidence <= 1: raise ValueError('confidence must be in [0,1]')
    if any(not math.isfinite(y) for y in b.allowed_yaws_rad): raise ValueError('invalid yaw')

def validate_candidate(c: PlacementCandidate, s: SystemState):
    if c.target_pose.frame_id != 'pallet': raise ValueError("target_pose.frame_id must be 'pallet'")
    if c.base_state_version != s.state_version: raise ValueError('stale candidate')
    validate_pose(c.target_pose)
