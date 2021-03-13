"""SAPIEN core module"""
import sapien.core.pysapien
import typing
import numpy
import renderer
_Shape = typing.Tuple[int, ...]

__all__ = [
    "Actor",
    "ActorBase",
    "ActorBuilder",
    "ActorDynamicBase",
    "ActorStatic",
    "ActorType",
    "Articulation",
    "ArticulationBase",
    "ArticulationBuilder",
    "ArticulationDrivable",
    "ArticulationJointType",
    "ArticulationType",
    "BoxGeometry",
    "CameraSpec",
    "CapsuleGeometry",
    "CollisionGeometry",
    "CollisionShape",
    "Contact",
    "ContactPoint",
    "ConvexMeshGeometry",
    "DYNAMIC",
    "Drive",
    "Engine",
    "FIX",
    "ICamera",
    "IPxrRenderer",
    "ISensor",
    "Input",
    "Joint",
    "JointBase",
    "JointRecord",
    "KINEMATIC",
    "KINEMATIC_LINK",
    "KinematicArticulation",
    "KinematicJoint",
    "KinematicJointFixed",
    "KinematicJointPrismatic",
    "KinematicJointRevolute",
    "KinematicJointSingleDof",
    "KinematicLink",
    "LINK",
    "Link",
    "LinkBase",
    "LinkBuilder",
    "OptifuserCamera",
    "OptifuserConfig",
    "OptifuserController",
    "OptifuserMaterial",
    "OptifuserRenderer",
    "PGS",
    "PRISMATIC",
    "PhysicalMaterial",
    "PinocchioModel",
    "PlaneGeometry",
    "Pose",
    "REVOLUTE",
    "RenderBody",
    "RenderGeometry",
    "RenderMaterial",
    "RenderScene",
    "RenderShape",
    "SPHERICAL",
    "STATIC",
    "Scene",
    "SceneConfig",
    "ShapeRecord",
    "SolverType",
    "SphereGeometry",
    "Subscription",
    "TGS",
    "Trigger",
    "UNDEFINED",
    "URDFLoader",
    "VisualRecord",
    "VulkanCamera",
    "VulkanMaterial",
    "VulkanRenderer",
    "VulkanScene",
    "VulkanWindow",
    "renderer"
]


class ActorBase():
    def __repr__(self) -> str: ...
    def get_collision_shapes(self) -> typing.List[CollisionShape]: ...
    def get_id(self) -> int: ...
    def get_name(self) -> str: ...
    def get_pose(self) -> Pose: ...
    def get_scene(self) -> Scene: ...
    def get_visual_bodies(self) -> typing.List[RenderBody]: ...
    def hide_visual(self) -> None: ...
    def is_hiding_visual(self) -> bool: ...
    def on_contact(self, func: typing.Callable[[ActorBase, ActorBase, Contact], None]) -> None: ...
    def on_step(self, func: typing.Callable[[ActorBase, float], None]) -> None: ...
    def on_trigger(self, func: typing.Callable[[ActorBase, ActorBase, Trigger], None]) -> None: ...
    def render_collision(self, render: bool = True) -> None: ...
    def set_name(self, name: str) -> None: ...
    def unhide_visual(self) -> None: ...
    @property
    def col1(self) -> int:
        """
        :type: int
        """
    @property
    def col2(self) -> int:
        """
        :type: int
        """
    @property
    def col3(self) -> int:
        """
        :type: int
        """
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class ActorDynamicBase(ActorBase):
    def add_force_at_point(self, force: numpy.ndarray[numpy.float32], point: numpy.ndarray[numpy.float32]) -> None: ...
    def add_force_torque(self, force: numpy.ndarray[numpy.float32], torque: numpy.ndarray[numpy.float32]) -> None: ...
    def get_angular_velocity(self) -> numpy.ndarray[numpy.float32]: ...
    def get_cmass_local_pose(self) -> Pose: ...
    def get_inertia(self) -> numpy.ndarray[numpy.float32]: ...
    def get_mass(self) -> float: ...
    def get_velocity(self) -> numpy.ndarray[numpy.float32]: ...
    def set_damping(self, linear: float, angular: float) -> None: ...
    @property
    def angular_velocity(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def cmass_local_pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def inertia(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def mass(self) -> float:
        """
        :type: float
        """
    @property
    def velocity(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class ActorBuilder():
    def add_box_shape(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), size: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> None: ...
    def add_box_visual_complex(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), size: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None, name: str = '') -> None: ...
    def add_capsule_shape(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, half_length: float = 1, material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> None: ...
    def add_capsule_visual_complex(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, half_length: float = 1, material: RenderMaterial = None, name: str = '') -> None: ...
    def add_collision_group(self, arg0: int, arg1: int, arg2: int) -> None: ...
    def add_convex_shape_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> None: ...
    def add_multiple_convex_shapes_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> None: ...
    def add_sphere_shape(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> None: ...
    def add_sphere_visual_complex(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, material: RenderMaterial = None, name: str = '') -> None: ...
    def add_visual_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), name: str = '') -> None: ...
    def build(self, is_kinematic: bool = False, name: str = '') -> Actor: ...
    def build_static(self, name: str = '') -> ActorStatic: ...
    def get_shapes(self) -> typing.List[ShapeRecord]: ...
    def get_visuals(self) -> typing.List[VisualRecord]: ...
    def remove_all_shapes(self) -> None: ...
    def remove_all_visuals(self) -> None: ...
    def remove_shape_at(self, index: int) -> None: ...
    def remove_visual_at(self, index: int) -> None: ...
    def reset_collision_group(self) -> None: ...
    def set_collision_group(self, arg0: int, arg1: int, arg2: int) -> None: ...
    def set_mass_and_inertia(self, arg0: float, arg1: Pose, arg2: numpy.ndarray[numpy.float32]) -> None: ...
    def set_scene(self, arg0: Scene) -> None: ...
    pass
class Actor(ActorDynamicBase, ActorBase):
    def lock_motion(self, x: bool = True, y: bool = True, z: bool = True, rx: bool = True, ry: bool = True, rz: bool = True) -> None: ...
    def pack(self) -> typing.List[float]: ...
    def set_angular_velocity(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    def set_solver_iterations(self, position: int, velocity: int = 1) -> None: ...
    def set_velocity(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    def unpack(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ActorStatic(ActorBase):
    def pack(self) -> typing.List[float]: ...
    def set_pose(self, pose: Pose) -> None: ...
    def unpack(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ActorType():
    """
    Members:

      STATIC

      KINEMATIC

      DYNAMIC

      LINK

      KINEMATIC_LINK
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    DYNAMIC: sapien.core.pysapien.ActorType # value = <ActorType.DYNAMIC: 2>
    KINEMATIC: sapien.core.pysapien.ActorType # value = <ActorType.KINEMATIC: 1>
    KINEMATIC_LINK: sapien.core.pysapien.ActorType # value = <ActorType.KINEMATIC_LINK: 4>
    LINK: sapien.core.pysapien.ActorType # value = <ActorType.LINK: 3>
    STATIC: sapien.core.pysapien.ActorType # value = <ActorType.STATIC: 0>
    __members__: dict # value = {'STATIC': <ActorType.STATIC: 0>, 'KINEMATIC': <ActorType.KINEMATIC: 1>, 'DYNAMIC': <ActorType.DYNAMIC: 2>, 'LINK': <ActorType.LINK: 3>, 'KINEMATIC_LINK': <ActorType.KINEMATIC_LINK: 4>}
    pass
class ArticulationBase():
    def create_pinocchio_model(self) -> PinocchioModel: ...
    def get_base_joints(self) -> typing.List[JointBase]: ...
    def get_base_links(self) -> typing.List[LinkBase]: ...
    def get_name(self) -> str: ...
    def get_pose(self) -> Pose: 
        """
        same as get_root_pose
        """
    def get_qacc(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qf(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qlimits(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qpos(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qvel(self) -> numpy.ndarray[numpy.float32]: ...
    def get_root_pose(self) -> Pose: ...
    def set_name(self, name: str) -> None: ...
    def set_pose(self, pose: Pose) -> None: 
        """
        same as set_root_pose
        """
    def set_qacc(self, qacc: numpy.ndarray[numpy.float32]) -> None: ...
    def set_qf(self, qf: numpy.ndarray[numpy.float32]) -> None: ...
    def set_qlimits(self, qlimits: numpy.ndarray[numpy.float32]) -> None: ...
    def set_qpos(self, qpos: numpy.ndarray[numpy.float32]) -> None: ...
    def set_qvel(self, qvel: numpy.ndarray[numpy.float32]) -> None: ...
    def set_root_pose(self, pose: Pose) -> None: ...
    @property
    def dof(self) -> int:
        """
        :type: int
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def pose(self) -> Pose:
        """
        same as get_root_pose()

        :type: Pose
        """
    @property
    def type(self) -> ArticulationType:
        """
        :type: ArticulationType
        """
    pass
class ArticulationDrivable(ArticulationBase):
    def get_drive_target(self) -> numpy.ndarray[numpy.float32]: ...
    def set_drive_target(self, drive_target: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ArticulationBuilder():
    def build(self, fix_base: bool = False) -> Articulation: ...
    def build_kinematic(self) -> KinematicArticulation: ...
    def create_link_builder(self, parent: LinkBuilder = None) -> LinkBuilder: ...
    def get_link_builders(self) -> typing.List[LinkBuilder]: ...
    def get_scene(self) -> Scene: ...
    def set_scene(self, scene: Scene) -> None: ...
    pass
class Articulation(ArticulationDrivable, ArticulationBase):
    def compute_adjoint_matrix(self, source_link_ik: int, target_link_id: int) -> numpy.ndarray[numpy.float32, _Shape[6, 6]]: ...
    def compute_cartesian_diff_ik(self, world_velocity: numpy.ndarray[numpy.float32, _Shape[6, 1]], commanded_link_id: int, active_joint_ids: typing.List[int] = []) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def compute_forward_dynamics(self, arg0: numpy.ndarray[numpy.float32]) -> numpy.ndarray[numpy.float32]: ...
    def compute_inverse_dynamics(self, arg0: numpy.ndarray[numpy.float32]) -> numpy.ndarray[numpy.float32]: ...
    def compute_manipulator_inertia_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[m, n]]: ...
    def compute_passive_force(self, gravity: bool = True, coriolis_and_centrifugal: bool = True, external: bool = True) -> numpy.ndarray[numpy.float32]: ...
    def compute_spatial_twist_jacobian(self) -> numpy.ndarray[numpy.float32, _Shape[m, n]]: ...
    def compute_transformation_matrix(self, source_link_id: int, target_link_id: int) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: ...
    def compute_twist_diff_ik(self, spatial_twist: numpy.ndarray[numpy.float32, _Shape[6, 1]], commanded_link_id: int, active_joint_ids: typing.List[int] = []) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def compute_world_cartesian_jacobian(self) -> numpy.ndarray[numpy.float32, _Shape[m, n]]: ...
    def get_active_joints(self) -> typing.List[Joint]: ...
    def get_joints(self) -> typing.List[Joint]: ...
    def get_links(self) -> typing.List[Link]: ...
    def pack(self) -> typing.List[float]: ...
    def set_root_angular_velocity(self, vel: numpy.ndarray[numpy.float32]) -> None: ...
    def set_root_velocity(self, vel: numpy.ndarray[numpy.float32]) -> None: ...
    def unpack(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ArticulationJointType():
    """
    Members:

      PRISMATIC

      REVOLUTE

      SPHERICAL

      FIX

      UNDEFINED
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    FIX: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.FIX: 3>
    PRISMATIC: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.PRISMATIC: 0>
    REVOLUTE: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.REVOLUTE: 1>
    SPHERICAL: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.SPHERICAL: 2>
    UNDEFINED: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.UNDEFINED: 4>
    __members__: dict # value = {'PRISMATIC': <ArticulationJointType.PRISMATIC: 0>, 'REVOLUTE': <ArticulationJointType.REVOLUTE: 1>, 'SPHERICAL': <ArticulationJointType.SPHERICAL: 2>, 'FIX': <ArticulationJointType.FIX: 3>, 'UNDEFINED': <ArticulationJointType.UNDEFINED: 4>}
    pass
class ArticulationType():
    """
    Members:

      DYNAMIC

      KINEMATIC
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    DYNAMIC: sapien.core.pysapien.ArticulationType # value = <ArticulationType.DYNAMIC: 0>
    KINEMATIC: sapien.core.pysapien.ArticulationType # value = <ArticulationType.KINEMATIC: 1>
    __members__: dict # value = {'DYNAMIC': <ArticulationType.DYNAMIC: 0>, 'KINEMATIC': <ArticulationType.KINEMATIC: 1>}
    pass
class CollisionGeometry():
    pass
class CameraSpec():
    def get_model_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_projection_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def lookAt(self, direction: numpy.ndarray[numpy.float32], up: numpy.ndarray[numpy.float32]) -> None: ...
    def set_position(self, position: numpy.ndarray[numpy.float32]) -> None: ...
    def set_rotation(self, rotation: numpy.ndarray[numpy.float32]) -> None: ...
    @property
    def aspect(self) -> float:
        """
        :type: float
        """
    @aspect.setter
    def aspect(self, arg0: float) -> None:
        pass
    @property
    def far(self) -> float:
        """
        :type: float
        """
    @far.setter
    def far(self, arg0: float) -> None:
        pass
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg0: str) -> None:
        pass
    @property
    def near(self) -> float:
        """
        :type: float
        """
    @near.setter
    def near(self, arg0: float) -> None:
        pass
    @property
    def position(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def rotation(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class CapsuleGeometry(CollisionGeometry):
    @property
    def half_length(self) -> float:
        """
        :type: float
        """
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class BoxGeometry(CollisionGeometry):
    @property
    def half_lengths(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class CollisionShape():
    @property
    def box_geometry(self) -> BoxGeometry:
        """
        :type: BoxGeometry
        """
    @property
    def capsule_geometry(self) -> CapsuleGeometry:
        """
        :type: CapsuleGeometry
        """
    @property
    def convex_mesh_geometry(self) -> ConvexMeshGeometry:
        """
        :type: ConvexMeshGeometry
        """
    @property
    def plane_geometry(self) -> PlaneGeometry:
        """
        :type: PlaneGeometry
        """
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def sphere_geometry(self) -> SphereGeometry:
        """
        :type: SphereGeometry
        """
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class Contact():
    def __repr__(self) -> str: ...
    @property
    def actor1(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def actor2(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def ends(self) -> bool:
        """
        :type: bool
        """
    @property
    def persists(self) -> bool:
        """
        :type: bool
        """
    @property
    def points(self) -> typing.List[ContactPoint]:
        """
        :type: typing.List[ContactPoint]
        """
    @property
    def starts(self) -> bool:
        """
        :type: bool
        """
    pass
class ContactPoint():
    @property
    def impulse(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def normal(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def position(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def separation(self) -> float:
        """
        :type: float
        """
    pass
class ConvexMeshGeometry(CollisionGeometry):
    @property
    def indices(self) -> numpy.ndarray[numpy.uint32]:
        """
        :type: numpy.ndarray[numpy.uint32]
        """
    @property
    def rotation(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def vertices(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class Drive():
    def destroy(self) -> None: ...
    def lock_motion(self, tx: bool, ty: bool, tz: bool, rx: bool, ry: bool, rz: bool) -> None: ...
    def set_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_target(self, pose: Pose) -> None: ...
    def set_target_velocity(self, linear: numpy.ndarray[numpy.float32], angular: numpy.ndarray[numpy.float32]) -> None: ...
    def set_x_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_y_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_z_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    pass
class Engine():
    def __init__(self, n_thread: int = 0, tolerance_length: float = 0.10000000149011612, tolerance_speed: float = 0.20000000298023224) -> None: ...
    def create_physical_material(self, static_friction: float, dynamic_friction: float, restitution: float) -> PhysicalMaterial: ...
    def create_scene(self, config: SceneConfig = ...) -> Scene: ...
    def get_renderer(self) -> IPxrRenderer: ...
    def set_log_level(self, level: str) -> None: ...
    def set_renderer(self, renderer: IPxrRenderer) -> None: ...
    pass
class ISensor():
    def get_pose(self) -> Pose: ...
    def set_initial_pose(self, pose: Pose) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    pass
class IPxrRenderer():
    def create_material(self) -> RenderMaterial: ...
    pass
class ICamera(ISensor):
    def get_albedo_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_color_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_depth(self) -> numpy.ndarray[numpy.float32]: ...
    def get_far(self) -> float: ...
    def get_fovy(self) -> float: ...
    def get_height(self) -> int: ...
    def get_name(self) -> str: ...
    def get_near(self) -> float: ...
    def get_normal_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_obj_segmentation(self) -> numpy.ndarray[numpy.int32]: ...
    def get_segmentation(self) -> numpy.ndarray[numpy.int32]: ...
    def get_width(self) -> int: ...
    def take_picture(self) -> None: ...
    pass
class Input():
    def get_key_down(self, arg0: int) -> int: ...
    def get_key_mods(self, arg0: int) -> int: ...
    def get_key_state(self, arg0: int) -> int: ...
    pass
class JointBase():
    def __repr__ (self) -> str: ...
    def get_child_link(self) -> LinkBase: ...
    def get_dof(self) -> int: ...
    def get_limits(self) -> numpy.ndarray[numpy.float32]: ...
    def get_name(self) -> str: ...
    def get_parent_link(self) -> LinkBase: ...
    def get_pose_in_child_frame(self) -> Pose: ...
    def get_pose_in_parent_frame(self) -> Pose: ...
    def set_limits(self, limits: numpy.ndarray[numpy.float32]) -> None: ...
    def set_name(self, name: str) -> None: ...
    @property
    def articulation(self) -> ArticulationBase:
        """
        :type: ArticulationBase
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def type(self) -> ArticulationJointType:
        """
        :type: ArticulationJointType
        """
    pass
class Joint(JointBase):
    def get_global_pose(self) -> Pose: ...
    def set_drive_property(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38) -> None: ...
    def set_drive_target(self, target: float) -> None: ...
    def set_drive_velocity_target(self, velocity: float) -> None: ...
    def set_friction(self, friction: float) -> None: ...
    @property
    def damping(self) -> float:
        """
        :type: float
        """
    @property
    def force_limit(self) -> float:
        """
        :type: float
        """
    @property
    def friction(self) -> float:
        """
        :type: float
        """
    @property
    def stiffness(self) -> float:
        """
        :type: float
        """
    pass
class JointRecord():
    @property
    def child_pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def damping(self) -> float:
        """
        :type: float
        """
    @property
    def friction(self) -> float:
        """
        :type: float
        """
    @property
    def joint_type(self) -> str:
        """
        :type: str
        """
    @property
    def limits(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def parent_pose(self) -> Pose:
        """
        :type: Pose
        """
    pass
class KinematicArticulation(ArticulationDrivable, ArticulationBase):
    pass
class KinematicJoint(JointBase):
    pass
class KinematicJointFixed(KinematicJoint, JointBase):
    pass
class KinematicJointSingleDof(KinematicJoint, JointBase):
    pass
class KinematicJointRevolute(KinematicJointSingleDof, KinematicJoint, JointBase):
    pass
class KinematicJointPrismatic(KinematicJointSingleDof, KinematicJoint, JointBase):
    pass
class LinkBase(ActorDynamicBase, ActorBase):
    def get_articulation(self) -> ArticulationBase: ...
    def get_index(self) -> int: ...
    pass
class Link(LinkBase, ActorDynamicBase, ActorBase):
    def get_articulation(self) -> Articulation: ...
    pass
class KinematicLink(LinkBase, ActorDynamicBase, ActorBase):
    def get_articulation(self) -> KinematicArticulation: ...
    pass
class LinkBuilder(ActorBuilder):
    def get_index(self) -> int: ...
    def get_joints(self) -> JointRecord: ...
    def get_name(self) -> str: ...
    def get_parent(self) -> int: ...
    def set_joint_name(self, arg0: str) -> None: ...
    def set_joint_properties(self, joint_type: ArticulationJointType, limits: numpy.ndarray[numpy.float32], parent_pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), child_pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), friction: float = 0, damping: float = 0) -> None: ...
    def set_name(self, arg0: str) -> None: ...
    def set_parent(self, arg0: int) -> None: ...
    pass
class OptifuserCamera(ICamera, ISensor):
    def get_camera_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_custom_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_model_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_projection_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def set_mode_orthographic(self, scaling: float = 1.0) -> None: ...
    def set_mode_perspective(self, fovy: float = 0.6108652353286743) -> None: ...
    pass
class OptifuserConfig():
    def __init__(self) -> None: ...
    @property
    def shadow_frustum_size(self) -> float:
        """
        :type: float
        """
    @shadow_frustum_size.setter
    def shadow_frustum_size(self, arg0: float) -> None:
        pass
    @property
    def shadow_map_size(self) -> int:
        """
        :type: int
        """
    @shadow_map_size.setter
    def shadow_map_size(self, arg0: int) -> None:
        pass
    @property
    def use_ao(self) -> bool:
        """
        :type: bool
        """
    @use_ao.setter
    def use_ao(self, arg0: bool) -> None:
        pass
    @property
    def use_shadow(self) -> bool:
        """
        :type: bool
        """
    @use_shadow.setter
    def use_shadow(self, arg0: bool) -> None:
        pass
    pass
class OptifuserController():
    def __init__(self, renderer: OptifuserRenderer) -> None: ...
    def focus(self, actor: ActorBase) -> None: ...
    def get_camera_pose(self) -> Pose: ...
    def get_selected_actor(self) -> ActorBase: ...
    def hide_window(self) -> None: ...
    def render(self) -> None: ...
    def set_camera_position(self, x: float, y: float, z: float) -> None: ...
    def set_camera_rotation(self, yaw: float, pitch: float) -> None: ...
    def set_current_scene(self, scene: Scene) -> None: ...
    def show_window(self) -> None: ...
    @property
    def input(self) -> Input:
        """
        :type: Input
        """
    @property
    def should_quit(self) -> bool:
        """
        :type: bool
        """
    pass
class RenderMaterial():
    def set_base_color(self, rgba: numpy.ndarray[numpy.float32]) -> None: ...
    def set_metallic(self, metallic: float) -> None: ...
    def set_roughness(self, roughness: float) -> None: ...
    def set_specular(self, specular: float) -> None: ...
    pass
class OptifuserRenderer(IPxrRenderer):
    def __init__(self, glsl_dir: str = '', glsl_version: str = '', config: OptifuserConfig = ...) -> None: ...
    def enable_global_axes(self, enable: bool = True) -> None: ...
    @staticmethod
    def set_default_shader_config(glsl_dir: str, glsl_version: str) -> None: ...
    def set_log_level(self, level: str) -> None: ...
    pass
class PhysicalMaterial():
    def get_dynamic_friction(self) -> float: ...
    def get_restitution(self) -> float: ...
    def get_static_friction(self) -> float: ...
    def set_dynamic_friction(self, coef: float) -> None: ...
    def set_restitution(self, coef: float) -> None: ...
    def set_static_friction(self, coef: float) -> None: ...
    pass
class PinocchioModel():
    def compute_coriolis_matrix(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], qvel: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def compute_forward_dynamics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], qvel: numpy.ndarray[numpy.float64, _Shape[m, 1]], qf: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: ...
    def compute_forward_kinematics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: ...
    def compute_full_jacobian(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: ...
    def compute_generalized_mass_matrix(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def compute_inverse_dynamics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], qvel: numpy.ndarray[numpy.float64, _Shape[m, 1]], qacc: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: ...
    def compute_inverse_kinematics(self, link_index: int, pose: Pose, initial_qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]] = array([], dtype=float64), eps: float = 0.0001, max_iterations: int = 1000, dt: float = 0.1, damp: float = 1e-06) -> typing.Tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], bool, numpy.ndarray[numpy.float64, _Shape[6, 1]]]: ...
    def compute_single_link_local_jacobian(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], link_index: int) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: ...
    def get_link_jacobian(self, link_index: int, local: bool = False) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: ...
    def get_link_pose(self, link_index: int) -> Pose: ...
    pass
class PlaneGeometry(CollisionGeometry):
    pass
class Pose():
    def __init__(self, p: numpy.ndarray[numpy.float32] = array([0., 0., 0.], dtype=float32), q: numpy.ndarray[numpy.float32] = array([1., 0., 0., 0.], dtype=float32)) -> None: ...
    def __mul__(self, arg0: Pose) -> Pose: ...
    def __repr__(self) -> str: ...
    @staticmethod
    def from_transformation_matrix(mat44: numpy.ndarray[numpy.float32]) -> Pose: ...
    def inv(self) -> Pose: ...
    def set_p(self, p: numpy.ndarray[numpy.float32]) -> None: ...
    def set_q(self, q: numpy.ndarray[numpy.float32]) -> None: ...
    def set_rotation(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    def to_transformation_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: ...
    def transform(self, arg0: Pose) -> Pose: ...
    @property
    def p(self) -> numpy.ndarray[numpy.float32, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[3, 1]]
        """
    @property
    def q(self) -> numpy.ndarray[numpy.float32, _Shape[4, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[4, 1]]
        """
    pass
class RenderBody():
    def get_name(self) -> str: ...
    def get_render_shapes(self) -> typing.List[RenderShape]: ...
    def get_segmentation_id(self) -> int: ...
    def get_unique_id(self) -> int: ...
    def set_custom_data(self, custom_data: typing.List[float]) -> None: ...
    def set_name(self, name: str) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    def set_unique_id(self, id: int) -> None: ...
    def set_visibility(self, visibility: float) -> None: ...
    pass
class RenderGeometry():
    @property
    def bitangents(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def indices(self) -> numpy.ndarray[numpy.uint32]:
        """
        :type: numpy.ndarray[numpy.uint32]
        """
    @property
    def normals(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def tangents(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def uvs(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def vertices(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class OptifuserMaterial(RenderMaterial):
    pass
class RenderScene():
    def add_mesh_from_file(self, mesh_file: str, scale: numpy.ndarray[numpy.float32]) -> RenderBody: ...
    def add_primitive_mesh(self, type: str, scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None) -> RenderBody: ...
    def remove_mesh(self, mesh: RenderBody) -> None: ...
    pass
class RenderShape():
    @property
    def mesh(self) -> RenderGeometry:
        """
        :type: RenderGeometry
        """
    @property
    def obj_id(self) -> int:
        """
        :type: int
        """
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class Scene():
    def add_directional_light(self, direction: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32]) -> None: ...
    def add_ground(self, altitude: float, render: bool = True, material: PhysicalMaterial = None, render_material: RenderMaterial = None) -> ActorStatic: ...
    def add_mounted_camera(self, name: str, actor: ActorBase, pose: Pose, width: int, height: int, fovx: float, fovy: float, near: float, far: float) -> ICamera: ...
    def add_point_light(self, position: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32]) -> None: ...
    def create_actor_builder(self) -> ActorBuilder: ...
    def create_articulation_builder(self) -> ArticulationBuilder: ...
    def create_drive(self, actor1: ActorBase, pose1: Pose, actor2: ActorBase, pose2: Pose) -> Drive: ...
    def create_urdf_loader(self) -> URDFLoader: ...
    def find_actor_by_id(self, id: int) -> ActorBase: ...
    def find_articulation_link_by_link_id(self, id: int) -> LinkBase: ...
    def find_mounted_camera(self, name: str, actor: ActorBase = None) -> ICamera: ...
    def generate_unique_render_id(self) -> int: ...
    def get_all_actors(self) -> typing.List[ActorBase]: ...
    def get_all_articulations(self) -> typing.List[ArticulationBase]: ...
    def get_contacts(self) -> typing.List[Contact]: ...
    def get_mounted_actors(self) -> typing.List[ActorBase]: ...
    def get_mounted_cameras(self) -> typing.List[ICamera]: ...
    def get_render_scene(self) -> RenderScene: ...
    def get_timestep(self) -> float: ...
    def pack(self) -> typing.Dict[str, typing.Dict[int, typing.List[float]]]: ...
    def remove_actor(self, actor: ActorBase) -> None: ...
    def remove_articulation(self, articulation: Articulation) -> None: ...
    def remove_kinematic_articulation(self, kinematic_articulation: KinematicArticulation) -> None: ...
    def remove_mounted_camera(self, camera: ICamera) -> None: ...
    def set_ambient_light(self, clor: numpy.ndarray[numpy.float32]) -> None: ...
    def set_shadow_light(self, direction: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32]) -> None: ...
    def set_timestep(self, second: float) -> None: ...
    def step(self) -> None: ...
    def step_async(self) -> None: ...
    def step_wait(self) -> None: ...
    def unpack(self, data: typing.Dict[str, typing.Dict[int, typing.List[float]]]) -> None: ...
    def update_render(self) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def render_id_to_visual_name(self) -> typing.Dict[int, str]:
        """
        :type: typing.Dict[int, str]
        """
    @property
    def timestep(self) -> float:
        """
        :type: float
        """
    @timestep.setter
    def timestep(self, arg1: float) -> None:
        pass
    pass
class SceneConfig():
    def __init__(self) -> None: ...
    @property
    def bounce_threshold(self) -> float:
        """
        :type: float
        """
    @bounce_threshold.setter
    def bounce_threshold(self, arg0: float) -> None:
        pass
    @property
    def contact_offset(self) -> float:
        """
        :type: float
        """
    @contact_offset.setter
    def contact_offset(self, arg0: float) -> None:
        pass
    @property
    def default_dynamic_friction(self) -> float:
        """
        :type: float
        """
    @default_dynamic_friction.setter
    def default_dynamic_friction(self, arg0: float) -> None:
        pass
    @property
    def default_restitution(self) -> float:
        """
        :type: float
        """
    @default_restitution.setter
    def default_restitution(self, arg0: float) -> None:
        pass
    @property
    def default_static_friction(self) -> float:
        """
        :type: float
        """
    @default_static_friction.setter
    def default_static_friction(self, arg0: float) -> None:
        pass
    @property
    def enable_adaptive_force(self) -> bool:
        """
        :type: bool
        """
    @enable_adaptive_force.setter
    def enable_adaptive_force(self, arg0: bool) -> None:
        pass
    @property
    def enable_ccd(self) -> bool:
        """
        :type: bool
        """
    @enable_ccd.setter
    def enable_ccd(self, arg0: bool) -> None:
        pass
    @property
    def enable_enhanced_determinism(self) -> bool:
        """
        :type: bool
        """
    @enable_enhanced_determinism.setter
    def enable_enhanced_determinism(self, arg0: bool) -> None:
        pass
    @property
    def enable_friction_every_iteration(self) -> bool:
        """
        :type: bool
        """
    @enable_friction_every_iteration.setter
    def enable_friction_every_iteration(self, arg0: bool) -> None:
        pass
    @property
    def enable_pcm(self) -> bool:
        """
        :type: bool
        """
    @enable_pcm.setter
    def enable_pcm(self, arg0: bool) -> None:
        pass
    @property
    def enable_tgs(self) -> bool:
        """
        :type: bool
        """
    @enable_tgs.setter
    def enable_tgs(self, arg0: bool) -> None:
        pass
    @property
    def gravity(self) -> numpy.ndarray[numpy.float32, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[3, 1]]
        """
    @gravity.setter
    def gravity(self, arg0: numpy.ndarray[numpy.float32, _Shape[3, 1]]) -> None:
        pass
    @property
    def sleep_threshold(self) -> float:
        """
        :type: float
        """
    @sleep_threshold.setter
    def sleep_threshold(self, arg0: float) -> None:
        pass
    @property
    def solver_iterations(self) -> int:
        """
        :type: int
        """
    @solver_iterations.setter
    def solver_iterations(self, arg0: int) -> None:
        pass
    @property
    def solver_velocity_iterations(self) -> int:
        """
        :type: int
        """
    @solver_velocity_iterations.setter
    def solver_velocity_iterations(self, arg0: int) -> None:
        pass
    pass
class ShapeRecord():
    @property
    def density(self) -> float:
        """
        :type: float
        """
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def material(self) -> PhysicalMaterial:
        """
        :type: PhysicalMaterial
        """
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class SolverType():
    """
    Members:

      PGS

      TGS
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    PGS: sapien.core.pysapien.SolverType # value = <SolverType.PGS: 0>
    TGS: sapien.core.pysapien.SolverType # value = <SolverType.TGS: 1>
    __members__: dict # value = {'PGS': <SolverType.PGS: 0>, 'TGS': <SolverType.TGS: 1>}
    pass
class SphereGeometry(CollisionGeometry):
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class Subscription():
    def unsubscribe(self) -> None: ...
    pass
class Trigger():
    def __repr__(self) -> str: ...
    @property
    def actor_other(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def actor_trigger(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def ends(self) -> bool:
        """
        :type: bool
        """
    @property
    def starts(self) -> bool:
        """
        :type: bool
        """
    pass
class URDFLoader():
    def __init__(self, scene: Scene) -> None: ...
    def load(self, filename: str, config: dict = {}) -> Articulation: ...
    def load_file_as_articulation_builder(self, filename: str, config: dict = {}) -> ArticulationBuilder: ...
    def load_from_string(self, urdf_string: str, srdf_string: str, config: dict = {}) -> Articulation: ...
    def load_kinematic(self, filename: str, config: dict = {}) -> KinematicArticulation: ...
    @property
    def collision_is_visual(self) -> bool:
        """
        :type: bool
        """
    @collision_is_visual.setter
    def collision_is_visual(self, arg0: bool) -> None:
        pass
    @property
    def default_density(self) -> None:
        """
        :type: None
        """
    @default_density.setter
    def default_density(self) -> None:
        pass
    @property
    def fix_root_link(self) -> bool:
        """
        :type: bool
        """
    @fix_root_link.setter
    def fix_root_link(self, arg0: bool) -> None:
        pass
    @property
    def load_multiple_shapes_from_file(self) -> bool:
        """
        :type: bool
        """
    @load_multiple_shapes_from_file.setter
    def load_multiple_shapes_from_file(self, arg0: bool) -> None:
        pass
    @property
    def scale(self) -> float:
        """
        :type: float
        """
    @scale.setter
    def scale(self, arg0: float) -> None:
        pass
    pass
class VisualRecord():
    @property
    def density(self) -> str:
        """
        :type: str
        """
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def length(self) -> float:
        """
        :type: float
        """
    @property
    def material(self) -> RenderMaterial:
        """
        :type: RenderMaterial
        """
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class VulkanCamera(ICamera, ISensor):
    def get_camera_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_float_texture(self, texture_name: str) -> numpy.ndarray[numpy.float32]: ...
    def get_model_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_projection_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_uint32_texture(self, texture_name: str) -> numpy.ndarray[numpy.uint32]: ...
    def set_mode_orthographic(self, scale: float) -> None: ...
    def set_mode_perspective(self, fovy: float) -> None: ...
    pass
class VulkanMaterial(RenderMaterial):
    pass
class VulkanRenderer(IPxrRenderer):
    def __init__(self, offscreen_only: bool = False, max_num_materials: int = 5000, max_num_textures: int = 5000, default_mipmap_levels: int = 1) -> None: ...
    def create_window(self, width: int = 800, height: int = 600, shader_dir: str = '') -> VulkanWindow: ...
    def set_log_level(self, level: str) -> None: ...
    @staticmethod
    def set_shader_dir(shader_dir: str) -> None: ...
    @property
    def _internal_context(self) -> renderer.Context:
        """
        :type: renderer.Context
        """
    pass
class VulkanScene(RenderScene):
    def add_shadow_directional_light(self, direction: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32], position: numpy.ndarray[numpy.float32] = array([0., 0., 0.], dtype=float32), scale: float = 10.0, near: float = -10.0, far: float = 10.0) -> None: ...
    def add_shadow_point_light(self, position: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32], near: float = 0.1, far: float = 10) -> None: ...
    @property
    def _internal_scene(self) -> renderer.Scene:
        """
        :type: renderer.Scene
        """
    pass
class VulkanWindow():
    def download_float_target(self, name: str) -> numpy.ndarray[numpy.float32]: ...
    def download_float_target_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.float32]: ...
    def download_uint32_target(self, name: str) -> numpy.ndarray[numpy.uint32]: ...
    def download_uint32_target_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.uint32]: ...
    def download_uint8_target(self, name: str) -> numpy.ndarray[numpy.uint8]: ...
    def download_uint8_target_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.uint8]: ...
    def get_camera_position(self) -> numpy.ndarray[numpy.float32]: ...
    def get_camera_rotation(self) -> numpy.ndarray[numpy.float32]: ...
    def hide(self) -> None: ...
    def key_down(self, key: str) -> bool: ...
    def key_press(self, key: str) -> bool: ...
    def mouse_click(self, key: int) -> bool: ...
    def mouse_down(self, key: int) -> bool: ...
    def render(self, target_name: str, ui_windows: typing.List[renderer.UIWindow] = []) -> None: ...
    def resize(self, width: int, height: int) -> None: ...
    def set_camera_parameters(self, near: float, far: float, fovy: float) -> None: ...
    def set_camera_position(self, position: numpy.ndarray[numpy.float32]) -> None: ...
    def set_camera_rotation(self, quat: numpy.ndarray[numpy.float32]) -> None: ...
    def set_scene(self, scene: Scene) -> None: ...
    def show(self) -> None: ...
    @property
    def alt(self) -> bool:
        """
        :type: bool
        """
    @property
    def ctrl(self) -> bool:
        """
        :type: bool
        """
    @property
    def mouse_delta(self) -> typing.List[float[2]]:
        """
        :type: typing.List[float[2]]
        """
    @property
    def mouse_position(self) -> typing.List[float[2]]:
        """
        :type: typing.List[float[2]]
        """
    @property
    def mouse_wheel_delta(self) -> typing.List[float[2]]:
        """
        :type: typing.List[float[2]]
        """
    @property
    def shift(self) -> bool:
        """
        :type: bool
        """
    @property
    def super(self) -> bool:
        """
        :type: bool
        """
    @property
    def target_names(self) -> typing.List[str]:
        """
        :type: typing.List[str]
        """
    pass
DYNAMIC: sapien.core.pysapien.ArticulationType # value = <ArticulationType.DYNAMIC: 0>
FIX: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.FIX: 3>
KINEMATIC: sapien.core.pysapien.ArticulationType # value = <ArticulationType.KINEMATIC: 1>
KINEMATIC_LINK: sapien.core.pysapien.ActorType # value = <ActorType.KINEMATIC_LINK: 4>
LINK: sapien.core.pysapien.ActorType # value = <ActorType.LINK: 3>
PGS: sapien.core.pysapien.SolverType # value = <SolverType.PGS: 0>
PRISMATIC: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.PRISMATIC: 0>
REVOLUTE: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.REVOLUTE: 1>
SPHERICAL: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.SPHERICAL: 2>
STATIC: sapien.core.pysapien.ActorType # value = <ActorType.STATIC: 0>
TGS: sapien.core.pysapien.SolverType # value = <SolverType.TGS: 1>
UNDEFINED: sapien.core.pysapien.ArticulationJointType # value = <ArticulationJointType.UNDEFINED: 4>
