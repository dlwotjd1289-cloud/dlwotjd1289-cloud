import pytest
from pac_common.models import *
from pac_common.validation import validate_box, validate_candidate

def state(): return SystemState(7,1.0,PalletState('P001',Size3D(1.2,1.0,0.15),()),InventoryState({}, {'SKU_A':1}))
def test_valid_box():
    validate_box(BoxState('B001','SKU_A',Size3D(.4,.3,.2),12.0,Pose3D('conveyor',0,0,.9),(0.0,1.57079632679),BoxStatus.READY_FOR_PICK,1.0,1.0,'fixture'))
def test_candidate_requires_pallet_frame():
    c=PlacementCandidate('S0007-B001-C000','B001',Pose3D('world',.1,.1,.2),7)
    with pytest.raises(ValueError): validate_candidate(c,state())
def test_candidate_rejects_stale_state():
    c=PlacementCandidate('S0006-B001-C000','B001',Pose3D('pallet',.1,.1,.2),6)
    with pytest.raises(ValueError): validate_candidate(c,state())
