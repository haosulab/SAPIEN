import sapien.core.pysapien_ros2.ros2
from typing import *
from typing import Iterable as iterable
from typing import Iterator as iterator
from numpy import float64
_Shape = Tuple[int, ...]
import numpy
import sapien.core.pysapien_ros2.core
__all__  = [
"CartesianVelocityController",
"JointVelocityController",
"KinematicsConfig",
"KinematicsSolverType",
"MotionPlan",
"MotionPlanner",
"MotionPlanningConfig",
"MoveType",
"RobotDescriptor",
"RobotLoader",
"RobotManager",
"SceneManager",
"get_resources_directory",
"init_spd_logger",
"rclcpp_init",
"set_resources_directory",
"set_ros2_logging_level"
]
class CartesianVelocityController():
    def move_cartesian(self, arg0: numpy.ndarray[float64], arg1: MoveType) -> None: ...
    def move_twist(self, arg0: numpy.ndarray[float64], arg1: MoveType) -> None: ...
    @property
    def joint_velocity_limit(self) -> float:
        """
        :type: float
        """
    @joint_velocity_limit.setter
    def joint_velocity_limit(self, arg1: float) -> None:
        pass
    pass
class JointVelocityController():
    @overload
    def move_joint(self, joint_names: List[str], velocities: List[float], continuous: bool = True) -> None: 
        """
        Move joints with given names, same velocity for all joints.

        Move joints with given names, velocity is specified for each joints.

        Move joints with velocity, given default order.
        """
    @overload
    def move_joint(self, joint_names: List[str], velocity: float, continuous: bool = True) -> None: ...
    @overload
    def move_joint(self, velocity: List[float], continuous: bool = True) -> None: ...
    pass
class KinematicsConfig():
    def __init__(self) -> None: ...
    @property
    def kinematics_solver_attempts(self) -> int:
        """
        :type: int
        """
    @kinematics_solver_attempts.setter
    def kinematics_solver_attempts(self, arg0: int) -> None:
        pass
    @property
    def kinematics_solver_search_resolution(self) -> float:
        """
        :type: float
        """
    @kinematics_solver_search_resolution.setter
    def kinematics_solver_search_resolution(self, arg0: float) -> None:
        pass
    @property
    def kinematics_solver_timeout(self) -> float:
        """
        :type: float
        """
    @kinematics_solver_timeout.setter
    def kinematics_solver_timeout(self, arg0: float) -> None:
        pass
    @property
    def kinematics_solver_type(self) -> str:
        """
        :type: str
        """
    @kinematics_solver_type.setter
    def kinematics_solver_type(self, arg0: str) -> None:
        pass
    pass
class KinematicsSolverType():
    """
    Members:

      KDL
    """
    def __eq__(self, arg0: object) -> bool: ...
    def __getstate__(self) -> int_: ...
    def __hash__(self) -> int_: ...
    def __init__(self, arg0: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, arg0: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, arg0: int) -> None: ...
    @property
    def name(self) -> None:
        """
        :type: None
        """
    KDL: sapien.core.pysapien_ros2.ros2.KinematicsSolverType # value = KinematicsSolverType.KDL
    __entries: dict # value = {'KDL': (KinematicsSolverType.KDL, None)}
    __members__: dict # value = {'KDL': KinematicsSolverType.KDL}
    pass
class MotionPlan():
    @property
    def acceleration(self) -> numpy.ndarray[float64, _Shape[m, n]]:
        """
        :type: numpy.ndarray[float64, _Shape[m, n]]
        """
    @property
    def duration(self) -> numpy.ndarray[float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[float64, _Shape[m, 1]]
        """
    @property
    def joint_names(self) -> List[str]:
        """
        :type: List[str]
        """
    @property
    def position(self) -> numpy.ndarray[float64, _Shape[m, n]]:
        """
        :type: numpy.ndarray[float64, _Shape[m, n]]
        """
    @property
    def velocity(self) -> numpy.ndarray[float64, _Shape[m, n]]:
        """
        :type: numpy.ndarray[float64, _Shape[m, n]]
        """
    pass
class MotionPlanner():
    def plan(self) -> MotionPlan: ...
    @overload
    def set_goal_state(self, pose: sapien.core.pysapien_ros2.core.Pose, link_name: str = '') -> bool: ...
    @overload
    def set_goal_state(self, qpos: numpy.ndarray[float64, _Shape[m, 1]]) -> bool: ...
    def set_start_state(self, qpos: numpy.ndarray[float64, _Shape[m, 1]]) -> bool: ...
    def set_start_state_to_current_state(self) -> None: ...
    def update_collision_objects(self, arg0: List[sapien.core.pysapien_ros2.core.ActorBase]) -> None: ...
    pass
class MotionPlanningConfig():
    def __init__(self) -> None: ...
    @property
    def max_acceleration_scaling_factor(self) -> float:
        """
        :type: float
        """
    @max_acceleration_scaling_factor.setter
    def max_acceleration_scaling_factor(self, arg0: float) -> None:
        pass
    @property
    def max_velocity_scaling_factor(self) -> float:
        """
        :type: float
        """
    @max_velocity_scaling_factor.setter
    def max_velocity_scaling_factor(self, arg0: float) -> None:
        pass
    @property
    def planning_attempts(self) -> int:
        """
        :type: int
        """
    @planning_attempts.setter
    def planning_attempts(self, arg0: int) -> None:
        pass
    @property
    def planning_plugin(self) -> str:
        """
        :type: str
        """
    @planning_plugin.setter
    def planning_plugin(self, arg0: str) -> None:
        pass
    @property
    def request_adapter(self) -> str:
        """
        :type: str
        """
    @request_adapter.setter
    def request_adapter(self, arg0: str) -> None:
        pass
    @property
    def start_state_max_bounds_error(self) -> float:
        """
        :type: float
        """
    @start_state_max_bounds_error.setter
    def start_state_max_bounds_error(self, arg0: float) -> None:
        pass
    pass
class MoveType():
    """
    Members:

      WORLD_TRANSLATE

      WORLD_ROTATE

      LOCAL_TRANSLATE

      LOCAL_ROTATE

      BODY_TWIST

      SPATIAL_TWIST
    """
    def __eq__(self, arg0: object) -> bool: ...
    def __getstate__(self) -> int_: ...
    def __hash__(self) -> int_: ...
    def __init__(self, arg0: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, arg0: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, arg0: int) -> None: ...
    @property
    def name(self) -> None:
        """
        :type: None
        """
    BODY_TWIST: sapien.core.pysapien_ros2.ros2.MoveType # value = MoveType.BODY_TWIST
    LOCAL_ROTATE: sapien.core.pysapien_ros2.ros2.MoveType # value = MoveType.LOCAL_ROTATE
    LOCAL_TRANSLATE: sapien.core.pysapien_ros2.ros2.MoveType # value = MoveType.LOCAL_TRANSLATE
    SPATIAL_TWIST: sapien.core.pysapien_ros2.ros2.MoveType # value = MoveType.SPATIAL_TWIST
    WORLD_ROTATE: sapien.core.pysapien_ros2.ros2.MoveType # value = MoveType.WORLD_ROTATE
    WORLD_TRANSLATE: sapien.core.pysapien_ros2.ros2.MoveType # value = MoveType.WORLD_TRANSLATE
    __entries: dict # value = {'WORLD_TRANSLATE': (MoveType.WORLD_TRANSLATE, None), 'WORLD_ROTATE': (MoveType.WORLD_ROTATE, None), 'LOCAL_TRANSLATE': (MoveType.LOCAL_TRANSLATE, None), 'LOCAL_ROTATE': (MoveType.LOCAL_ROTATE, None), 'BODY_TWIST': (MoveType.BODY_TWIST, None), 'SPATIAL_TWIST': (MoveType.SPATIAL_TWIST, None)}
    __members__: dict # value = {'WORLD_TRANSLATE': MoveType.WORLD_TRANSLATE, 'WORLD_ROTATE': MoveType.WORLD_ROTATE, 'LOCAL_TRANSLATE': MoveType.LOCAL_TRANSLATE, 'LOCAL_ROTATE': MoveType.LOCAL_ROTATE, 'BODY_TWIST': MoveType.BODY_TWIST, 'SPATIAL_TWIST': MoveType.SPATIAL_TWIST}
    pass
class RobotDescriptor():
    def __init__(self, urdf: str, srdf: str, substitute_path: str = '') -> None: ...
    @staticmethod
    def from_path(urdf_path: str, srdf_path: str) -> RobotDescriptor: ...
    @staticmethod
    def from_ros(ros2_package_name: str, urdf_relative_path: str, srdf_relative_path: str) -> RobotDescriptor: ...
    def get_srdf(self) -> str: ...
    def get_standard_urdf(self) -> str: ...
    def get_urdf(self) -> str: ...
    pass
class RobotLoader():
    def __init__(self, scene_manager: SceneManager) -> None: ...
    def load_robot_and_manager(self, robot_descriptor: RobotDescriptor, robot_name: str, config: dict = {}) -> Tuple[sapien.core.pysapien_ros2.core.Articulation, RobotManager]: ...
    @property
    def collision_is_visual(self) -> bool:
        """
        :type: bool
        """
    @collision_is_visual.setter
    def collision_is_visual(self, arg1: bool) -> None:
        pass
    @property
    def fix_root_link(self) -> bool:
        """
        :type: bool
        """
    @fix_root_link.setter
    def fix_root_link(self, arg1: bool) -> None:
        pass
    pass
class RobotManager():
    def balance_passive_force(self, gravity: bool = True, coriolis_centrifugal: bool = True, external: bool = True) -> None: ...
    def build_cartesian_velocity_controller(self, group_name: str, service_name: str = '', latency: float = 0) -> sapien::ros2::CartesianVelocityController: ...
    def build_joint_velocity_controller(self, joint_names: List[str], service_name: str = '', latency: float = 0) -> sapien::ros2::JointVelocityController: ...
    def build_motion_planner(self, group_name: str, service_name: str = '') -> sapien::ros2::MotionPlanner: ...
    def create_joint_publisher(self, frequency: float) -> None: ...
    def get_group_names(self) -> List[str]: ...
    def get_kinematics_config(self) -> KinematicsConfig: ...
    def get_motion_planning_config(self) -> MotionPlanningConfig: ...
    def set_drive_property(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, joint_index: List[int] = []) -> None: ...
    def set_kinematics_config(self, config: KinematicsConfig = <KinematicsConfig object at 0x7f41729c1848>) -> bool: ...
    def set_motion_planning_config(self, config: MotionPlanningConfig = <MotionPlanningConfig object at 0x7f41729c1880>) -> bool: ...
    pass
class SceneManager():
    def __init__(self, scene: sapien.core.pysapien_ros2.core.Scene, scene_name: str) -> None: ...
    def create_robot_loader(self) -> RobotLoader: ...
    def now(self) -> float: ...
    def start(self) -> None: ...
    pass
def get_resources_directory() -> str:
    pass
def init_spd_logger() -> None:
    pass
def rclcpp_init(args: List[str]) -> None:
    pass
def set_resources_directory(path: str) -> None:
    pass
def set_ros2_logging_level(arg0: str) -> None:
    pass
