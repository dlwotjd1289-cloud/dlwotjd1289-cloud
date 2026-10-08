from pac_common.models import RejectCode, ValidationResult
from .capability import RobotCapability

class Hdp16031Adapter:
    """Fail-closed until a verified HDP160-31 planning backend is loaded."""
    capability = RobotCapability('hdp160_31',4,160.0,3.128,None,None)
    def __init__(self, backend=None): self._backend=backend
    @property
    def backend_ready(self): return self._backend is not None
    def validate_robot_motion(self, box, candidate, state, robot_state):
        if candidate.base_state_version != state.state_version:
            return ValidationResult(False,(RejectCode.STALE_PLAN,),{'candidate_version':candidate.base_state_version,'state_version':state.state_version})
        if self._backend is None:
            return ValidationResult(False,(RejectCode.INVALID_STATE,),{'reason':'verified_hdp16031_backend_not_loaded','robot_model':'hdp160_31'})
        return self._backend.validate_robot_motion(box=box,candidate=candidate,state=state,robot_state=robot_state)

class SimulatedPassThroughRobotAdapter:
    """TEST ONLY. Never use as ACTUAL feasibility."""
    def validate_robot_motion(self, box, candidate, state, robot_state):
        if candidate.base_state_version != state.state_version:
            return ValidationResult(False,(RejectCode.STALE_PLAN,),{'mode':'SIMULATED'})
        return ValidationResult(True,(),{'mode':'SIMULATED','warning':'no physical robot validation'})
