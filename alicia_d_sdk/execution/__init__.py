"""
Execution Layer

Provides trajectory execution functionality, including:
- Joint space trajectory execution
- Cartesian space trajectory execution
- Drag teaching and pose recording
"""

from alicia_d_sdk.execution.trajectory_executor import JointTrajectoryExecutor, CartesianTrajectoryExecutor

__all__ = [
    "JointTrajectoryExecutor",
    "CartesianTrajectoryExecutor"
]