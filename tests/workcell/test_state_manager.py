from pac_common.models import *
from pac_common.state_manager import StateManager

def initial(): return SystemState(0,0.0,PalletState('P001',Size3D(1.2,1.0,.15),()),InventoryState({}, {'SKU_A':1}))
def test_state_manager_commit_flow():
    m=StateManager(initial())
    b=BoxState('B001','SKU_A',Size3D(.4,.3,.2),12.0,Pose3D('conveyor',0,0,.9),(0.0,1.57079632679),BoxStatus.READY_FOR_PICK,1.0,1.0,'sim')
    s1=m.commit_observation(b,1.0); assert s1.state_version==1
    r=ExecutionResult(True,'B001','S0001-B001-C000',Pose3D('pallet',.2,.2,.25),(),2.0)
    s2=m.commit_execution(r); assert s2.state_version==2; assert s2.inventory.tracked_boxes['B001'].status is BoxStatus.PLACED; assert len(s2.pallet.boxes)==1
