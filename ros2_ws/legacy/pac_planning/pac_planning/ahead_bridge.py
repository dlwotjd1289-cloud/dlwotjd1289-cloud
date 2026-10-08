"""Migration boundary for the existing AHEAD core.
Existing Scenario / EP / Hard Constraint code should be adapted behind pac_common
contracts instead of importing ROS messages into the algorithm core.
"""
def migration_status():
    return {'scenario':'reuse_existing_core','state':'adapt_to_pac_common',
            'candidate':'adapt_to_PlacementCandidate','hard_constraint':'reuse_and_contract_test',
            'teacher':'keep_robot_agnostic'}
