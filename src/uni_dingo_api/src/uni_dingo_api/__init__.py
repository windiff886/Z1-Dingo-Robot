# Uni-Dingo API - Unified Robot Framework
# ========================================
#
# Usage:
#   from uni_dingo_api import RobotAPI
#   robot = RobotAPI()
#   robot.base.move_to(x=1.0, y=0.0, theta=0.0).wait()

from .api.robot_api import RobotAPI
from .api.base_api import BaseAPI
from .api.arm_api import ArmAPI
from .api.action_future import ActionFuture

__version__ = '0.1.0'
__all__ = ['RobotAPI', 'BaseAPI', 'ArmAPI', 'ActionFuture']
