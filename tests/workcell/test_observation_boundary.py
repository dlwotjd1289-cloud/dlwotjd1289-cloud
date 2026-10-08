from pac_common.models import *
from pac_perception.observation_generator import GroundTruthObservationGenerator

def test_observation_is_immutable_boundary():
    b=BoxState('B001','SKU_A',Size3D(.4,.3,.2),12.0,Pose3D('conveyor',0,0,.9),(0.0,1.57079632679),BoxStatus.ON_CONVEYOR,.8,1.0,'ground_truth')
    o=GroundTruthObservationGenerator().observe_box(b,2.0)
    assert b.stamp_sec==1.0 and o.stamp_sec==2.0 and o.source=='sim_ground_truth_top_view'
