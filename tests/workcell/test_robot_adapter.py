from pac_common.models import *
from pac_robot.hdp160_31_adapter import Hdp16031Adapter

def test_hdp_adapter_fails_closed_without_verified_backend():
    s=SystemState(3,1.0,PalletState('P001',Size3D(1.2,1,.15),()),InventoryState({},{}))
    b=BoxState('B001','SKU_A',Size3D(.4,.3,.2),12.0,Pose3D('conveyor',0,0,.9),(0.0,1.57079632679),BoxStatus.READY_FOR_PICK,1.0,1.0,'fixture')
    c=PlacementCandidate('S0003-B001-C000','B001',Pose3D('pallet',.2,.2,.2),3)
    result=Hdp16031Adapter().validate_robot_motion(b,c,s,None)
    assert not result.success and RejectCode.INVALID_STATE in result.codes
