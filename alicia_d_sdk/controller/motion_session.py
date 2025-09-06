from alicia_d_sdk.kinematics import IKController, AliciaFollower
from alicia_d_sdk.driver import ArmController
from alicia_d_sdk.execution import TrajectoryExecutor

class MotionSession:
    def __init__(self, ik_controller: IKController, 
                 robot_model: AliciaFollower, 
                 joint_controller: ArmController,
                 executor: TrajectoryExecutor):
        
        self.ik_controller = ik_controller
        self.robot_model = robot_model
        self.joint_controller = joint_controller
        self.executor = executor

    def __repr__(self):
        return "<MotionSession Alicia-D>"
