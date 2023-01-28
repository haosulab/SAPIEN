"""SAPIEN core module"""
from __future__ import annotations
import sapien.core.pysapien
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "ActiveLightEntity",
    "Actor",
    "ActorBase",
    "ActorBuilder",
    "ActorDynamicBase",
    "ActorStatic",
    "Articulation",
    "ArticulationBase",
    "ArticulationBuilder",
    "ArticulationDrivable",
    "AwaitableDLList",
    "AwaitableVoid",
    "BoxGeometry",
    "CameraEntity",
    "CapsuleGeometry",
    "CollisionGeometry",
    "CollisionShape",
    "Constraint",
    "Contact",
    "ContactPoint",
    "ConvexMeshGeometry",
    "DirectionalLightEntity",
    "Drive",
    "Engine",
    "Entity",
    "Gear",
    "IPxrRenderer",
    "Joint",
    "JointBase",
    "JointRecord",
    "KinematicArticulation",
    "KinematicJoint",
    "KinematicJointFixed",
    "KinematicJointPrismatic",
    "KinematicJointRevolute",
    "KinematicJointSingleDof",
    "KinematicLink",
    "KuafuConfig",
    "KuafuRenderer",
    "LightEntity",
    "Link",
    "LinkBase",
    "LinkBuilder",
    "NonconvexMeshGeometry",
    "ParticleEntity",
    "PhysicalMaterial",
    "PinocchioModel",
    "PlaneGeometry",
    "PointLightEntity",
    "Pose",
    "ProfilerBlock",
    "RenderBody",
    "RenderClient",
    "RenderConfig",
    "RenderMaterial",
    "RenderMesh",
    "RenderParticleBody",
    "RenderScene",
    "RenderServer",
    "RenderServerBuffer",
    "RenderShape",
    "RenderTexture",
    "SapienRenderer",
    "Scene",
    "SceneConfig",
    "SceneMultistepCallback",
    "ShapeRecord",
    "SphereGeometry",
    "SpotLightEntity",
    "Subscription",
    "Trigger",
    "URDFLoader",
    "VisualRecord",
    "VulkanParticleBody",
    "VulkanRenderMesh",
    "VulkanRigidbody",
    "VulkanScene",
    "VulkanWindow",
    "add_profiler_event",
    "dlpack",
    "get_global_render_config",
    "renderer",
    "simsense"
]


class Entity():
    def get_name(self) -> str: ...
    def get_pose(self) -> Pose: ...
    def set_name(self, name: str) -> None: ...
    @property
    def _ptr(self) -> capsule:
        """
        :type: capsule
        """
    @property
    def classname(self) -> None:
        """
        :type: None
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
    pass
class ActorBase(Entity):
    def __repr__(self) -> str: ...
    def get_builder(self) -> ActorBuilder: ...
    def get_collision_shapes(self) -> typing.List[CollisionShape]: ...
    def get_collision_visual_bodies(self) -> typing.List[RenderBody]: ...
    def get_id(self) -> int: ...
    def get_scene(self) -> Scene: ...
    def get_visual_bodies(self) -> typing.List[RenderBody]: ...
    def hide_visual(self) -> None: ...
    def is_hiding_visual(self) -> bool: ...
    def on_contact(self, func: typing.Callable[[ActorBase, ActorBase, Contact], None]) -> None: ...
    def on_step(self, func: typing.Callable[[ActorBase, float], None]) -> None: ...
    def on_trigger(self, func: typing.Callable[[ActorBase, ActorBase, Trigger], None]) -> None: ...
    def render_collision(self, render: bool = True) -> None: ...
    def unhide_visual(self) -> None: ...
    @property
    def id(self) -> int:
        """
        :type: int
        """
    @property
    def type(self) -> str:
        """
        One of "static", "kinematic", "dynamic", "link", "kinematic_link"

        :type: str
        """
    pass
class ActorDynamicBase(ActorBase, Entity):
    def add_force_at_point(self, force: numpy.ndarray[numpy.float32], point: numpy.ndarray[numpy.float32]) -> None: ...
    def add_force_torque(self, force: numpy.ndarray[numpy.float32], torque: numpy.ndarray[numpy.float32]) -> None: ...
    def get_angular_velocity(self) -> numpy.ndarray[numpy.float32]: ...
    def get_cmass_local_pose(self) -> Pose: ...
    def get_inertia(self) -> numpy.ndarray[numpy.float32]: ...
    def get_mass(self) -> float: ...
    def get_velocity(self) -> numpy.ndarray[numpy.float32]: ...
    def set_ccd(self, enable: bool) -> None: ...
    def set_damping(self, linear: float, angular: float) -> None: ...
    @property
    def angular_velocity(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def ccd(self) -> bool:
        """
        :type: bool
        """
    @ccd.setter
    def ccd(self, arg1: bool) -> None:
        pass
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
    def add_box_collision(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), half_size: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> ActorBuilder: ...
    @typing.overload
    def add_box_visual(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), half_size: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), color: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), name: str = '') -> ActorBuilder: ...
    @typing.overload
    def add_box_visual(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), half_size: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None, name: str = '') -> ActorBuilder: ...
    def add_capsule_collision(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, half_length: float = 1, material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> ActorBuilder: 
        """
        Add a capsule collision shape. The height is along the x-axis.
        """
    @typing.overload
    def add_capsule_visual(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, half_length: float = 1, color: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), name: str = '') -> ActorBuilder: 
        """
        Add a capsule visual shape. The height is along the x-axis.

        Add a capsule visual shape. The height is along the x-axis.
        """
    @typing.overload
    def add_capsule_visual(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, half_length: float = 1, material: RenderMaterial = None, name: str = '') -> ActorBuilder: ...
    def add_collision_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> ActorBuilder: 
        """
        Add a collision shape from file (see assimp for supported formats).
        If the shape in the file is not convex, it will be converted by the PhysX backend.
        """
    def add_multiple_collisions_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> ActorBuilder: 
        """
        Add multiple collisions shapes from files. Also see @add_collision_from_file.
        Different from @add_collision_from_file, all connected components in the file will be converted to be convex.
        """
    def add_nonconvex_collision_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: PhysicalMaterial = None, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> ActorBuilder: 
        """
        Add a nonconvex collision shape from a file. If it is not a trigger, then it is only valid for static and kinematic actors.
        """
    def add_sphere_collision(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, material: PhysicalMaterial = None, density: float = 1000, patch_radius: float = 0.0, min_patch_radius: float = 0.0, is_trigger: bool = False) -> ActorBuilder: ...
    @typing.overload
    def add_sphere_visual(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, color: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), name: str = '') -> ActorBuilder: ...
    @typing.overload
    def add_sphere_visual(self, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), radius: float = 1, material: RenderMaterial = None, name: str = '') -> None: ...
    def add_visual_from_file(self, filename: str, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None, name: str = '') -> ActorBuilder: ...
    def add_visual_from_mesh(self, mesh: RenderMesh, pose: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None, name: str = '') -> ActorBuilder: ...
    def build(self, name: str = '') -> Actor: ...
    def build_kinematic(self, name: str = '') -> Actor: ...
    def build_static(self, name: str = '') -> ActorStatic: ...
    def get_collisions(self) -> typing.List[ShapeRecord]: ...
    def get_visuals(self) -> typing.List[VisualRecord]: ...
    def remove_all_collisions(self) -> ActorBuilder: ...
    def remove_all_visuals(self) -> ActorBuilder: ...
    def remove_collision_at(self, index: int) -> ActorBuilder: ...
    def remove_visual_at(self, index: int) -> ActorBuilder: ...
    def reset_collision_groups(self) -> ActorBuilder: ...
    def set_collision_groups(self, group0: int, group1: int, group2: int, group3: int) -> ActorBuilder: 
        """
        see CollisionShape.set_collision_groups
        """
    def set_mass_and_inertia(self, mass: float, inertia_pose: Pose, inertia: numpy.ndarray[numpy.float32]) -> ActorBuilder: 
        """
        Set the mass and inertia.

        Args:
          mass: the (scalar) mass of the actor
          inertia_pose: 
            the position is the center of mass;
            the rotation (quaternion) is the principle axis of inertia, relative to actor frame
          inertia: principle moments of inertia (a 3D vector)

        References:
          https://en.wikipedia.org/wiki/Moment_of_inertia#Principal_axes
        """
    def set_scene(self, arg0: Scene) -> ActorBuilder: ...
    pass
class Actor(ActorDynamicBase, ActorBase, Entity):
    def lock_motion(self, x: bool = True, y: bool = True, z: bool = True, rx: bool = True, ry: bool = True, rz: bool = True) -> None: ...
    def pack(self) -> typing.List[float]: ...
    def set_angular_velocity(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    def set_solver_iterations(self, position: int, velocity: int = 1) -> None: ...
    def set_velocity(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    def unpack(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ActorStatic(ActorBase, Entity):
    def pack(self) -> typing.List[float]: ...
    def set_pose(self, pose: Pose) -> None: ...
    def unpack(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ArticulationBase(Entity):
    def create_pinocchio_model(self) -> PinocchioModel: 
        """
        Create the kinematic and dynamic model of this articulation implemented by the Pinocchio library. Allowing computing forward/inverse kinematics/dynamics.
        """
    def export_urdf(self, cache_dir: str = '') -> str: ...
    def get_builder(self) -> ArticulationBuilder: ...
    def get_joints(self) -> typing.List[JointBase]: ...
    def get_links(self) -> typing.List[LinkBase]: ...
    def get_qacc(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qf(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qlimits(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qpos(self) -> numpy.ndarray[numpy.float32]: ...
    def get_qvel(self) -> numpy.ndarray[numpy.float32]: ...
    def get_root_pose(self) -> Pose: ...
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
    def type(self) -> str:
        """
        :type: str
        """
    pass
class ArticulationDrivable(ArticulationBase, Entity):
    def get_drive_target(self) -> numpy.ndarray[numpy.float32]: ...
    def set_drive_target(self, drive_target: numpy.ndarray[numpy.float32]) -> None: ...
    pass
class ArticulationBuilder():
    def build(self, fix_root_link: bool = False) -> Articulation: ...
    def build_kinematic(self) -> KinematicArticulation: ...
    def create_link_builder(self, parent: LinkBuilder = None) -> LinkBuilder: ...
    def get_link_builders(self) -> typing.List[LinkBuilder]: ...
    def get_scene(self) -> Scene: ...
    def set_scene(self, scene: Scene) -> None: ...
    pass
class Articulation(ArticulationDrivable, ArticulationBase, Entity):
    def compute_adjoint_matrix(self, source_link_ik: int, target_link_id: int) -> numpy.ndarray[numpy.float32, _Shape[6, 6]]: ...
    def compute_cartesian_diff_ik(self, world_velocity: numpy.ndarray[numpy.float32, _Shape[6, 1]], commanded_link_id: int, active_joint_ids: typing.List[int] = []) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def compute_forward_dynamics(self, arg0: numpy.ndarray[numpy.float32]) -> numpy.ndarray[numpy.float32]: ...
    def compute_generalized_external_force(self, forces: numpy.ndarray[numpy.float32, _Shape[m, 3]], torques: numpy.ndarray[numpy.float32, _Shape[m, 3]]) -> numpy.ndarray[numpy.float32]: ...
    def compute_inverse_dynamics(self, arg0: numpy.ndarray[numpy.float32]) -> numpy.ndarray[numpy.float32]: ...
    def compute_manipulator_inertia_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[m, n]]: ...
    def compute_passive_force(self, gravity: bool = True, coriolis_and_centrifugal: bool = True, external: bool = True) -> numpy.ndarray[numpy.float32]: ...
    def compute_spatial_twist_jacobian(self) -> numpy.ndarray[numpy.float32, _Shape[m, n]]: ...
    def compute_transformation_matrix(self, source_link_id: int, target_link_id: int) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: ...
    def compute_twist_diff_ik(self, spatial_twist: numpy.ndarray[numpy.float32, _Shape[6, 1]], commanded_link_id: int, active_joint_ids: typing.List[int] = []) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def compute_world_cartesian_jacobian(self) -> numpy.ndarray[numpy.float32, _Shape[m, n]]: ...
    def get_active_joints(self) -> typing.List[Joint]: ...
    def get_drive_velocity_target(self) -> numpy.ndarray[numpy.float32]: ...
    def pack(self) -> typing.List[float]: ...
    def set_drive_velocity_target(self, drive_velocity_target: numpy.ndarray[numpy.float32]) -> None: ...
    def set_root_angular_velocity(self, vel: numpy.ndarray[numpy.float32]) -> None: ...
    def set_root_velocity(self, vel: numpy.ndarray[numpy.float32]) -> None: ...
    def unpack(self, arg0: numpy.ndarray[numpy.float32]) -> None: ...
    @property
    def fixed(self) -> bool:
        """
        :type: bool
        """
    pass
class AwaitableDLList():
    def ready(self) -> bool: ...
    def wait(self) -> typing.List[capsule]: ...
    pass
class AwaitableVoid():
    def ready(self) -> bool: ...
    def wait(self) -> None: ...
    pass
class CollisionGeometry():
    pass
class CameraEntity(Entity):
    def get_albedo_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_camera_matrix(self) -> numpy.ndarray[numpy.float32]: 
        """
        Get 4x4 intrinsic camera matrix in OpenCV format.
        """
    def get_color_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_dl_tensor(self, texture_name: str) -> capsule: 
        """
        Get raw GPU memory for a render target in the dl format. It can be wrapped into PyTorch or Tensorflow using their API
        """
    def get_extrinsic_matrix(self) -> numpy.ndarray[numpy.float32]: 
        """
        Get 4x4 extrinsic camera matrix in OpenCV format.
        """
    def get_float_texture(self, texture_name: str) -> numpy.ndarray[numpy.float32]: ...
    def get_intrinsic_matrix(self) -> numpy.ndarray[numpy.float32]: 
        """
        Get 3x3 intrinsic camera matrix in OpenCV format.
        """
    def get_model_matrix(self) -> numpy.ndarray[numpy.float32]: 
        """
        Get model matrix (inverse of extrinsic matrix) used in rendering (Y up, Z back)
        """
    def get_normal_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_position_rgba(self) -> numpy.ndarray[numpy.float32]: ...
    def get_projection_matrix(self) -> numpy.ndarray[numpy.float32]: 
        """
        Get projection matrix in used in rendering (right-handed NDC with [-1,1] XY and [0,1] Z)
        """
    def get_texture(self, texture_name: str) -> numpy.ndarray: ...
    def get_uint32_texture(self, texture_name: str) -> numpy.ndarray[numpy.uint32]: ...
    def get_uint8_texture(self, texture_name: str) -> numpy.ndarray[numpy.uint8]: ...
    def get_visual_actor_segmentation(self) -> numpy.ndarray[numpy.uint32]: ...
    def set_focal_lengths(self, fx: float, fy: float) -> None: ...
    def set_fovx(self, fov: float, compute_y: bool = True) -> None: ...
    def set_fovy(self, fov: float, compute_x: bool = True) -> None: ...
    def set_local_pose(self, pose: Pose) -> None: ...
    def set_parent(self, parent: ActorBase, keep_pose: bool) -> None: ...
    def set_perspective_parameters(self, near: float, far: float, fx: float, fy: float, cx: float, cy: float, skew: float) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    def set_principal_point(self, cx: float, cy: float) -> None: ...
    def take_picture(self) -> None: ...
    def take_picture_and_get_dl_tensors_async(self, names: typing.List[str]) -> AwaitableDLList: ...
    @property
    def cx(self) -> float:
        """
        :type: float
        """
    @property
    def cy(self) -> float:
        """
        :type: float
        """
    @property
    def far(self) -> float:
        """
        :type: float
        """
    @far.setter
    def far(self, arg1: float) -> None:
        pass
    @property
    def fovx(self) -> float:
        """
        :type: float
        """
    @property
    def fovy(self) -> float:
        """
        :type: float
        """
    @property
    def fx(self) -> float:
        """
        :type: float
        """
    @property
    def fy(self) -> float:
        """
        :type: float
        """
    @property
    def height(self) -> int:
        """
        :type: int
        """
    @property
    def local_pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def near(self) -> float:
        """
        :type: float
        """
    @near.setter
    def near(self, arg1: float) -> None:
        pass
    @property
    def parent(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @parent.setter
    def parent(self, arg1: ActorBase, arg2: bool) -> None:
        pass
    @property
    def skew(self) -> float:
        """
        :type: float
        """
    @skew.setter
    def skew(self, arg1: float) -> None:
        pass
    @property
    def width(self) -> int:
        """
        :type: int
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
    def get_collision_groups(self) -> typing.List[int[4]]: ...
    def get_local_pose(self) -> Pose: ...
    def get_physical_material(self) -> PhysicalMaterial: ...
    def set_collision_groups(self, group0: int, group1: int, group2: int, group3: int) -> None: 
        """
        collision groups determine the collision behavior of objects. Let A.gx denote the collision group x of collision shape A. Collision shape A and B will collide iff the following condition holds:

        ((A.g0 & B.g1) or (A.g1 & B.g0)) and (not ((A.g2 & B.g2) and ((A.g3 & 0xffff) == (B.g3 & 0xffff))))

        Here is some explanation: g2 is the "ignore group" and g3 is the "id group". The only the lower 16 bits of the id group is used since the upper 16 bits are reserved for other purposes in the future. When 2 collision shapes have the same ID (g3), then if any of their g2 bits match, their collisions are definitely ignored.

        If after testing g2 and g3, the objects may collide, g0 and g1 come into play. g0 is the "contact type group" and g1 is the "contact affinity group". Collision shapes collide only when a bit in the contact type of the first shape matches a bit in the contact affinity of the second shape.
        """
    def set_local_pose(self, pose: Pose) -> None: ...
    def set_physical_material(self, material: PhysicalMaterial) -> None: ...
    @property
    def actor(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def contact_offset(self) -> float:
        """
        :type: float
        """
    @contact_offset.setter
    def contact_offset(self, arg1: float) -> None:
        pass
    @property
    def geometry(self) -> CollisionGeometry:
        """
        :type: CollisionGeometry
        """
    @property
    def is_trigger(self) -> bool:
        """
        :type: bool
        """
    @is_trigger.setter
    def is_trigger(self, arg1: bool) -> None:
        pass
    @property
    def min_patch_radius(self) -> float:
        """
        :type: float
        """
    @min_patch_radius.setter
    def min_patch_radius(self, arg1: float) -> None:
        pass
    @property
    def patch_radius(self) -> float:
        """
        :type: float
        """
    @patch_radius.setter
    def patch_radius(self, arg1: float) -> None:
        pass
    @property
    def rest_offset(self) -> float:
        """
        :type: float
        """
    @rest_offset.setter
    def rest_offset(self, arg1: float) -> None:
        pass
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class Constraint():
    pass
class Contact():
    def __repr__(self) -> str: ...
    @property
    def actor0(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def actor1(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @property
    def collision_shape0(self) -> CollisionShape:
        """
        :type: CollisionShape
        """
    @property
    def collision_shape1(self) -> CollisionShape:
        """
        :type: CollisionShape
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
class LightEntity(Entity):
    def set_color(self, color: numpy.ndarray[numpy.float32]) -> None: ...
    def set_direction(self, direction: numpy.ndarray[numpy.float32]) -> None: ...
    def set_local_pose(self, pose: Pose) -> None: ...
    def set_parent(self, parent: ActorBase, keep_pose: bool) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    def set_position(self, position: numpy.ndarray[numpy.float32]) -> None: ...
    @property
    def color(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def direction(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def local_pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def parent(self) -> ActorBase:
        """
        :type: ActorBase
        """
    @parent.setter
    def parent(self, arg1: ActorBase, arg2: bool) -> None:
        pass
    @property
    def pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def position(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def shadow(self) -> bool:
        """
        :type: bool
        """
    @shadow.setter
    def shadow(self, arg1: bool) -> None:
        pass
    pass
class Drive(Constraint):
    def free_motion(self, tx: bool, ty: bool, tz: bool, rx: bool, ry: bool, rz: bool) -> None: ...
    def lock_motion(self, tx: bool, ty: bool, tz: bool, rx: bool, ry: bool, rz: bool) -> None: ...
    def set_distance_limit(self, distance: float) -> None: ...
    def set_slerp_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_target(self, pose: Pose) -> None: ...
    def set_target_velocity(self, linear: numpy.ndarray[numpy.float32], angular: numpy.ndarray[numpy.float32]) -> None: ...
    def set_x_limit(self, low: float, high: float) -> None: ...
    def set_x_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_x_twist_limit(self, low: float, high: float) -> None: ...
    def set_x_twist_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_y_limit(self, low: float, high: float) -> None: ...
    def set_y_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_yz_cone_limit(self, y: float, z: float) -> None: ...
    def set_yz_pyramid_limit(self, ylow: float, yhigh: float, zlow: float, zhigh: float) -> None: ...
    def set_yz_swing_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    def set_z_limit(self, low: float, high: float) -> None: ...
    def set_z_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, is_acceleration: bool = True) -> None: ...
    pass
class Engine():
    def __init__(self, thread_count: int = 0, tolerance_length: float = 0.10000000149011612, tolerance_speed: float = 0.20000000298023224) -> None: ...
    def create_physical_material(self, static_friction: float, dynamic_friction: float, restitution: float) -> PhysicalMaterial: ...
    def create_scene(self, config: SceneConfig = SceneConfig()) -> Scene: ...
    def get_renderer(self) -> IPxrRenderer: ...
    def set_log_level(self, level: str) -> None: ...
    def set_renderer(self, renderer: IPxrRenderer) -> None: ...
    @property
    def renderer(self) -> IPxrRenderer:
        """
        :type: IPxrRenderer
        """
    @renderer.setter
    def renderer(self, arg1: IPxrRenderer) -> None:
        pass
    pass
class ActiveLightEntity(LightEntity, Entity):
    def set_fov(self, arg0: float) -> None: ...
    def set_shadow_parameters(self, near: float, far: float) -> None: ...
    @property
    def fov(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_far(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_near(self) -> float:
        """
        :type: float
        """
    pass
class Gear():
    @property
    def ratio(self) -> float:
        """
        :type: float
        """
    @ratio.setter
    def ratio(self, arg1: float) -> None:
        pass
    pass
class IPxrRenderer():
    def create_material(self) -> RenderMaterial: ...
    def create_mesh(self, vertices: numpy.ndarray[numpy.float32, _Shape[m, 3]], indices: numpy.ndarray[numpy.uint32, _Shape[m, 3]]) -> RenderMesh: ...
    def create_texture_from_array(self, array: numpy.ndarray[numpy.uint8], mipmap_levels: int = 1, filter_mode: str = 'linear', address_mode: str = 'repeat') -> RenderTexture: ...
    def create_texture_from_file(self, filename: str, mipmap_levels: int = 1, filter_mode: str = 'linear', address_mode: str = 'repeat') -> RenderTexture: ...
    pass
class JointBase():
    def __repr__ (self) -> str: ...
    def get_child_link(self) -> LinkBase: ...
    def get_dof(self) -> int: ...
    def get_limits(self) -> numpy.ndarray[numpy.float32]: ...
    def get_name(self) -> str: ...
    def get_parent_link(self) -> LinkBase: ...
    def get_pose_in_child(self) -> Pose: ...
    def get_pose_in_parent(self) -> Pose: ...
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
    def type(self) -> str:
        """
        :type: str
        """
    pass
class Joint(JointBase):
    def get_drive_target(self) -> float: ...
    def get_drive_velocity_target(self) -> float: ...
    def get_global_pose(self) -> Pose: ...
    def set_drive_property(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: str = 'force') -> None: ...
    def set_drive_target(self, target: float) -> None: ...
    def set_drive_velocity_target(self, velocity: float) -> None: ...
    def set_friction(self, friction: float) -> None: ...
    @property
    def damping(self) -> float:
        """
        :type: float
        """
    @property
    def drive_mode(self) -> str:
        """
        :type: str
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
    def pose_in_child(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def pose_in_parent(self) -> Pose:
        """
        :type: Pose
        """
    pass
class KinematicArticulation(ArticulationDrivable, ArticulationBase, Entity):
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
class LinkBase(ActorDynamicBase, ActorBase, Entity):
    def get_articulation(self) -> ArticulationBase: ...
    def get_index(self) -> int: ...
    pass
class KuafuConfig():
    def __init__(self) -> None: ...
    @property
    def accumulate_frames(self) -> bool:
        """
        :type: bool
        """
    @accumulate_frames.setter
    def accumulate_frames(self, arg0: bool) -> None:
        pass
    @property
    def assets_path(self) -> str:
        """
        :type: str
        """
    @assets_path.setter
    def assets_path(self, arg0: str) -> None:
        pass
    @property
    def max_bounces(self) -> int:
        """
        :type: int
        """
    @max_bounces.setter
    def max_bounces(self, arg0: int) -> None:
        pass
    @property
    def max_geometries(self) -> int:
        """
        :type: int
        """
    @max_geometries.setter
    def max_geometries(self, arg0: int) -> None:
        pass
    @property
    def max_geometry_instances(self) -> int:
        """
        :type: int
        """
    @max_geometry_instances.setter
    def max_geometry_instances(self, arg0: int) -> None:
        pass
    @property
    def max_materials(self) -> int:
        """
        :type: int
        """
    @max_materials.setter
    def max_materials(self, arg0: int) -> None:
        pass
    @property
    def max_textures(self) -> int:
        """
        :type: int
        """
    @max_textures.setter
    def max_textures(self, arg0: int) -> None:
        pass
    @property
    def spp(self) -> int:
        """
        :type: int
        """
    @spp.setter
    def spp(self, arg0: int) -> None:
        pass
    @property
    def use_denoiser(self) -> bool:
        """
        :type: bool
        """
    @use_denoiser.setter
    def use_denoiser(self, arg0: bool) -> None:
        pass
    @property
    def use_viewer(self) -> bool:
        """
        :type: bool
        """
    @use_viewer.setter
    def use_viewer(self, arg0: bool) -> None:
        pass
    @property
    def viewer_height(self) -> int:
        """
        :type: int
        """
    @viewer_height.setter
    def viewer_height(self, arg0: int) -> None:
        pass
    @property
    def viewer_width(self) -> int:
        """
        :type: int
        """
    @viewer_width.setter
    def viewer_width(self, arg0: int) -> None:
        pass
    pass
class KuafuRenderer(IPxrRenderer):
    def __init__(self, config: KuafuConfig = ...) -> None: ...
    @staticmethod
    def _set_default_assets_path(assets_path: str) -> None: ...
    @staticmethod
    def set_log_level(level: str) -> None: ...
    @property
    def is_running(self) -> bool:
        """
        :type: bool
        """
    pass
class DirectionalLightEntity(LightEntity, Entity):
    def set_shadow_parameters(self, half_size: float, near: float, far: float) -> None: ...
    @property
    def shadow_far(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_half_size(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_near(self) -> float:
        """
        :type: float
        """
    pass
class Link(LinkBase, ActorDynamicBase, ActorBase, Entity):
    def get_articulation(self) -> Articulation: ...
    pass
class KinematicLink(LinkBase, ActorDynamicBase, ActorBase, Entity):
    def get_articulation(self) -> KinematicArticulation: ...
    pass
class LinkBuilder(ActorBuilder):
    def get_index(self) -> int: ...
    def get_joint(self) -> JointRecord: ...
    def get_name(self) -> str: ...
    def get_parent(self) -> int: ...
    def set_joint_name(self, arg0: str) -> None: ...
    def set_joint_properties(self, joint_type: str, limits: numpy.ndarray[numpy.float32], pose_in_parent: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), pose_in_child: Pose = Pose([0, 0, 0], [1, 0, 0, 0]), friction: float = 0, damping: float = 0) -> None: 
        """
        Set the properties of the joint.

        Args:
          joint_type: ["revolute", "prismatic", "fixed"]
          limits: [[min1, max1], ...], the length is the DoF
          pose_in_parent: joint pose in parent frame. 
            The x-axis is the rotation axis for revolute, or the translation axis for prismatic.
          pose_in_child: joint pose in child frame. See also @pose_in_parent.
          friction: joint friction
          damping: joint damping
        """
    def set_name(self, arg0: str) -> None: ...
    def set_parent(self, arg0: int) -> None: ...
    pass
class NonconvexMeshGeometry(CollisionGeometry):
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
class ParticleEntity(Entity):
    @property
    def visual_body(self) -> RenderParticleBody:
        """
        :type: RenderParticleBody
        """
    pass
class PhysicalMaterial():
    def get_dynamic_friction(self) -> float: ...
    def get_restitution(self) -> float: ...
    def get_static_friction(self) -> float: ...
    def set_dynamic_friction(self, coef: float) -> None: ...
    def set_restitution(self, coef: float) -> None: ...
    def set_static_friction(self, coef: float) -> None: ...
    @property
    def dynamic_friction(self) -> float:
        """
        :type: float
        """
    @property
    def restitution(self) -> float:
        """
        :type: float
        """
    @property
    def static_friction(self) -> float:
        """
        :type: float
        """
    pass
class PinocchioModel():
    def compute_coriolis_matrix(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], qvel: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def compute_forward_dynamics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], qvel: numpy.ndarray[numpy.float64, _Shape[m, 1]], qf: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: ...
    def compute_forward_kinematics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: 
        """
        Compute and cache forward kinematics. After computation, use get_link_pose to retrieve the computed pose for a specific link.
        """
    def compute_full_jacobian(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: 
        """
        Compute and cache Jacobian for all links
        """
    def compute_generalized_mass_matrix(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, n]]: ...
    def compute_inverse_dynamics(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], qvel: numpy.ndarray[numpy.float64, _Shape[m, 1]], qacc: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]: ...
    def compute_inverse_kinematics(self, link_index: int, pose: Pose, initial_qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]] = array([], dtype=float64), active_qmask: numpy.ndarray[numpy.int32, _Shape[m, 1]] = array([], dtype=int32), eps: float = 0.0001, max_iterations: int = 1000, dt: float = 0.1, damp: float = 1e-06) -> typing.Tuple[numpy.ndarray[numpy.float64, _Shape[m, 1]], bool, numpy.ndarray[numpy.float64, _Shape[6, 1]]]: 
        """
        Compute inverse kinematics with CLIK algorithm.
        Details see https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
        Args:
            link_index: index of the link
            pose: target pose of the link in articulation base frame
            initial_qpos: initial qpos to start CLIK
            active_qmask: dof sized integer array, 1 to indicate active joints and 0 for inactive joints, default to all 1s
            max_iterations: number of iterations steps
            dt: iteration step "speed"
            damp: iteration step "damping"
        Returns:
            result: qpos from IK
            success: whether IK is successful
            error: se3 norm error
        """
    def compute_single_link_local_jacobian(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]], link_index: int) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: 
        """
        Compute the link(body) Jacobian for a single link. It is faster than compute_full_jacobian followed by get_link_jacobian
        """
    def get_link_jacobian(self, link_index: int, local: bool = False) -> numpy.ndarray[numpy.float64, _Shape[6, n]]: 
        """
        Given link index, get the Jacobian. Must be called after compute_full_jacobian.

        Args:
          link_index: index of the link
          local: True for world(spatial) frame; False for link(body) frame
        """
    def get_link_pose(self, link_index: int) -> Pose: 
        """
        Given link index, get link pose (in articulation base frame) from forward kinematics. Must be called after compute_forward_kinematics.
        """
    pass
class PlaneGeometry(CollisionGeometry):
    pass
class PointLightEntity(LightEntity, Entity):
    def set_shadow_parameters(self, near: float, far: float) -> None: ...
    @property
    def shadow_far(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_near(self) -> float:
        """
        :type: float
        """
    pass
class Pose():
    def __getstate__(self) -> tuple: ...
    def __init__(self, p: numpy.ndarray[numpy.float32] = array([0., 0., 0.], dtype=float32), q: numpy.ndarray[numpy.float32] = array([1., 0., 0., 0.], dtype=float32)) -> None: ...
    def __mul__(self, arg0: Pose) -> Pose: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, arg0: tuple) -> None: ...
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
class ProfilerBlock():
    def __enter__(self) -> None: ...
    def __exit__(self, arg0: object, arg1: object, arg2: object) -> None: ...
    def __init__(self, name: str) -> None: ...
    pass
class RenderBody():
    def get_actor_id(self) -> int: ...
    def get_name(self) -> str: ...
    def get_render_shapes(self) -> typing.List[RenderShape]: ...
    def get_visual_id(self) -> int: ...
    def set_custom_data(self, custom_data: typing.List[float]) -> None: ...
    def set_name(self, name: str) -> None: ...
    def set_pose(self, pose: Pose) -> None: ...
    def set_visibility(self, visibility: float) -> None: ...
    def set_visible(self, is_visible: bool) -> None: ...
    def set_visual_id(self, id: int) -> None: ...
    @property
    def actor_id(self) -> int:
        """
        :type: int
        """
    @property
    def half_length(self) -> float:
        """
        :type: float
        """
    @property
    def half_lengths(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def local_pose(self) -> Pose:
        """
        :type: Pose
        """
    @property
    def name(self) -> str:
        """
        :type: str
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
    @property
    def shade_flat(self) -> bool:
        """
        :type: bool
        """
    @shade_flat.setter
    def shade_flat(self, arg1: bool) -> None:
        pass
    @property
    def type(self) -> str:
        """
        :type: str
        """
    @property
    def visual_id(self) -> int:
        """
        :type: int
        """
    pass
class RenderClient(IPxrRenderer):
    def __init__(self, address: str, process_index: int) -> None: ...
    pass
class RenderConfig():
    def get_render_target_format(self, name: str) -> str: ...
    def has_render_target_format(self, name: str) -> bool: ...
    def set_render_target_format(self, name: str, format: str) -> None: ...
    def unset_render_target_format(self, name: str) -> None: ...
    @property
    def camera_shader_dir(self) -> str:
        """
        :type: str
        """
    @camera_shader_dir.setter
    def camera_shader_dir(self, arg0: str) -> None:
        pass
    @property
    def rt_path_depth(self) -> int:
        """
        :type: int
        """
    @rt_path_depth.setter
    def rt_path_depth(self, arg0: int) -> None:
        pass
    @property
    def rt_samples_per_pixel(self) -> int:
        """
        :type: int
        """
    @rt_samples_per_pixel.setter
    def rt_samples_per_pixel(self, arg0: int) -> None:
        pass
    @property
    def rt_use_denoiser(self) -> bool:
        """
        :type: bool
        """
    @rt_use_denoiser.setter
    def rt_use_denoiser(self, arg0: bool) -> None:
        pass
    @property
    def viewer_shader_dir(self) -> str:
        """
        :type: str
        """
    @viewer_shader_dir.setter
    def viewer_shader_dir(self, arg0: str) -> None:
        pass
    pass
class RenderMaterial():
    def set_base_color(self, rgba: numpy.ndarray[numpy.float32]) -> None: ...
    def set_diffuse_texture(self, texture: RenderTexture) -> None: ...
    def set_diffuse_texture_from_file(self, path: str) -> None: ...
    def set_emission(self, rgbs: numpy.ndarray[numpy.float32]) -> None: ...
    def set_emission_texture(self, texture: RenderTexture) -> None: ...
    def set_emission_texture_from_file(self, path: str) -> None: ...
    def set_ior(self, ior: float) -> None: ...
    def set_metallic(self, metallic: float) -> None: ...
    def set_metallic_texture(self, texture: RenderTexture) -> None: ...
    def set_metallic_texture_from_file(self, path: str) -> None: ...
    def set_normal_texture(self, texture: RenderTexture) -> None: ...
    def set_normal_texture_from_file(self, path: str) -> None: ...
    def set_roughness(self, roughness: float) -> None: ...
    def set_roughness_texture(self, texture: RenderTexture) -> None: ...
    def set_roughness_texture_from_file(self, path: str) -> None: ...
    def set_specular(self, specular: float) -> None: ...
    def set_transmission(self, transmission: float) -> None: ...
    def set_transmission_texture(self, texture: RenderTexture) -> None: ...
    def set_transmission_texture_from_file(self, path: str) -> None: ...
    @property
    def base_color(self) -> typing.List[float[4]]:
        """
        :type: typing.List[float[4]]
        """
    @base_color.setter
    def base_color(self, arg1: numpy.ndarray[numpy.float32]) -> None:
        pass
    @property
    def diffuse_texture(self) -> RenderTexture:
        """
        :type: RenderTexture
        """
    @diffuse_texture.setter
    def diffuse_texture(self, arg1: RenderTexture) -> None:
        pass
    @property
    def diffuse_texture_filename(self) -> str:
        """
        :type: str
        """
    @diffuse_texture_filename.setter
    def diffuse_texture_filename(self, arg1: str) -> None:
        pass
    @property
    def emission(self) -> typing.List[float[4]]:
        """
        :type: typing.List[float[4]]
        """
    @emission.setter
    def emission(self, arg1: numpy.ndarray[numpy.float32]) -> None:
        pass
    @property
    def emission_texture(self) -> RenderTexture:
        """
        :type: RenderTexture
        """
    @emission_texture.setter
    def emission_texture(self, arg1: RenderTexture) -> None:
        pass
    @property
    def emission_texture_filename(self) -> str:
        """
        :type: str
        """
    @emission_texture_filename.setter
    def emission_texture_filename(self, arg1: str) -> None:
        pass
    @property
    def ior(self) -> float:
        """
        :type: float
        """
    @ior.setter
    def ior(self, arg1: float) -> None:
        pass
    @property
    def metallic(self) -> float:
        """
        :type: float
        """
    @metallic.setter
    def metallic(self, arg1: float) -> None:
        pass
    @property
    def metallic_texture(self) -> RenderTexture:
        """
        :type: RenderTexture
        """
    @metallic_texture.setter
    def metallic_texture(self, arg1: RenderTexture) -> None:
        pass
    @property
    def metallic_texture_filename(self) -> str:
        """
        :type: str
        """
    @metallic_texture_filename.setter
    def metallic_texture_filename(self, arg1: str) -> None:
        pass
    @property
    def normal_texture(self) -> RenderTexture:
        """
        :type: RenderTexture
        """
    @normal_texture.setter
    def normal_texture(self, arg1: RenderTexture) -> None:
        pass
    @property
    def normal_texture_filename(self) -> str:
        """
        :type: str
        """
    @normal_texture_filename.setter
    def normal_texture_filename(self, arg1: str) -> None:
        pass
    @property
    def roughness(self) -> float:
        """
        :type: float
        """
    @roughness.setter
    def roughness(self, arg1: float) -> None:
        pass
    @property
    def roughness_texture(self) -> RenderTexture:
        """
        :type: RenderTexture
        """
    @roughness_texture.setter
    def roughness_texture(self, arg1: RenderTexture) -> None:
        pass
    @property
    def roughness_texture_filename(self) -> str:
        """
        :type: str
        """
    @roughness_texture_filename.setter
    def roughness_texture_filename(self, arg1: str) -> None:
        pass
    @property
    def specular(self) -> float:
        """
        :type: float
        """
    @specular.setter
    def specular(self, arg1: float) -> None:
        pass
    @property
    def transmission(self) -> float:
        """
        :type: float
        """
    @transmission.setter
    def transmission(self, arg1: float) -> None:
        pass
    @property
    def transmission_texture(self) -> RenderTexture:
        """
        :type: RenderTexture
        """
    @transmission_texture.setter
    def transmission_texture(self, arg1: RenderTexture) -> None:
        pass
    @property
    def transmission_texture_filename(self) -> str:
        """
        :type: str
        """
    @transmission_texture_filename.setter
    def transmission_texture_filename(self, arg1: str) -> None:
        pass
    pass
class RenderMesh():
    def set_indices(self, arg0: numpy.ndarray[numpy.uint32, _Shape[m, 3]]) -> None: ...
    def set_normals(self, arg0: numpy.ndarray[numpy.float32, _Shape[m, 3]]) -> None: ...
    def set_uvs(self, arg0: numpy.ndarray[numpy.float32, _Shape[m, 2]]) -> None: ...
    def set_vertices(self, arg0: numpy.ndarray[numpy.float32, _Shape[m, 3]]) -> None: ...
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
class RenderParticleBody():
    def set_attribute(self, name: str, value: numpy.ndarray[numpy.float32, _Shape[m, n]]) -> None: ...
    def set_rendered_point_count(self, n: int) -> None: ...
    def set_shading_mode(self, mode: int) -> None: ...
    def set_visibility(self, visibility: float) -> None: ...
    @property
    def rendered_point_count(self) -> int:
        """
        :type: int
        """
    @rendered_point_count.setter
    def rendered_point_count(self, arg1: int) -> None:
        pass
    pass
class RenderScene():
    def add_mesh_from_file(self, mesh_file: str, scale: numpy.ndarray[numpy.float32]) -> RenderBody: ...
    def add_primitive_mesh(self, type: str, scale: numpy.ndarray[numpy.float32] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None) -> RenderBody: ...
    def remove_mesh(self, mesh: RenderBody) -> None: ...
    @property
    def ambient_light(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    pass
class RenderServer():
    def __init__(self, max_num_materials: int = 5000, max_num_textures: int = 5000, default_mipmap_levels: int = 1, device: str = '', do_not_load_texture: bool = False) -> None: ...
    @staticmethod
    def _set_shader_dir(shader_dir: str) -> None: ...
    def auto_allocate_buffers(self, render_targets: typing.List[str]) -> typing.List[RenderServerBuffer]: ...
    def start(self, address: str) -> None: ...
    def stop(self) -> None: ...
    def summary(self) -> str: ...
    def wait_all(self, timeout: int = 18446744073709551615) -> bool: ...
    def wait_scenes(self, scenes: typing.List[int], timeout: int = 18446744073709551615) -> bool: ...
    pass
class RenderServerBuffer():
    @property
    def __cuda_array_interface__(self) -> dict:
        """
        :type: dict
        """
    @property
    def nbytes(self) -> int:
        """
        :type: int
        """
    @property
    def pointer(self) -> int:
        """
        :type: int
        """
    @property
    def shape(self) -> tuple:
        """
        :type: tuple
        """
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class RenderShape():
    def set_material(self, material: RenderMaterial) -> None: ...
    @property
    def material(self) -> RenderMaterial:
        """
        :type: RenderMaterial
        """
    @property
    def mesh(self) -> RenderMesh:
        """
        :type: RenderMesh
        """
    pass
class RenderTexture():
    @property
    def address_mode(self) -> str:
        """
        :type: str
        """
    @property
    def channels(self) -> int:
        """
        :type: int
        """
    @property
    def dtype(self) -> detail::accessor<detail::accessor_policies::str_attr>:
        """
        :type: detail::accessor<detail::accessor_policies::str_attr>
        """
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def filter_mode(self) -> str:
        """
        :type: str
        """
    @property
    def height(self) -> int:
        """
        :type: int
        """
    @property
    def mipmap_levels(self) -> int:
        """
        :type: int
        """
    @property
    def width(self) -> int:
        """
        :type: int
        """
    pass
class SapienRenderer(IPxrRenderer):
    def __init__(self, offscreen_only: bool = False, max_num_materials: int = 5000, max_num_textures: int = 5000, default_mipmap_levels: int = 1, device: str = '', culling: str = 'back', do_not_load_texture: bool = False) -> None: 
        """
        Create the Vulkan-based renderer (rasterization and ray tracing)

        Args:
          offscreen_only: tell the renderer the user does not need to present onto a screen. The renderer will not try to select a GPU with present abilities.
          max_num_materials: tell the maximum number of materials that will exist at the same time. Increase this number if descriptor pool is out of memory.
          max_num_textures: specify the maximum number of textures that will exist at the same time. Increase this number if descriptor pool is out of memory.
          default_mipmap_levels: set the mip map levels for loaded textures.
          device: One the the following:
            'cuda:x' where x is a decimal number, the renderer tries to render using this cuda device. Present request is ignored
            'cuda', the renderer tries to render using a cuda-visible device. If present is requested, it will be prioritized
            'pci:x', where x is a hexadecimal number, the renderer picks the device with given PCI bus number
            '', if present is requested, first try to find cuda+present, next present only, and then turn off present. If present is turned off, first try to find cuda, next any graphics device.
        """
    def _release_gpu_memory_unsafe(self) -> None: 
        """
        A very unsafe way to release cached gpu (but not CPU) resources. It MUST be called when no rendering is running, and all cameras and windows become invalid after calling this function.
        """
    def clear_cached_resources(self) -> None: ...
    def create_ktx_environment_map(self, px: str, nx: str, py: str, ny: str, pz: str, nz: str, out: str) -> None: ...
    def create_window(self, width: int = 800, height: int = 600, shader_dir: str = '') -> VulkanWindow: ...
    @staticmethod
    def set_log_level(level: str) -> None: ...
    @property
    def _internal_context(self) -> renderer.Context:
        """
        :type: renderer.Context
        """
    pass
class Scene():
    def _update_render_and_take_pictures(self, arg0: typing.List[CameraEntity]) -> None: ...
    def add_active_light(self, pose: Pose, color: numpy.ndarray[numpy.float32], fov: float, tex_path: str, near: float = 0.10000000149011612, far: float = 10.0, shadow_map_size: int = 2048) -> ActiveLightEntity: ...
    def add_camera(self, name: str, width: int, height: int, fovy: float, near: float, far: float) -> CameraEntity: ...
    def add_directional_light(self, direction: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32], shadow: bool = False, position: numpy.ndarray[numpy.float32] = array([0., 0., 0.], dtype=float32), scale: float = 10.0, near: float = -10.0, far: float = 10.0, shadow_map_size: int = 2048) -> DirectionalLightEntity: ...
    def add_ground(self, altitude: float, render: bool = True, material: PhysicalMaterial = None, render_material: RenderMaterial = None, render_half_size: numpy.ndarray[numpy.float32] = array([10., 10.], dtype=float32)) -> None: ...
    @typing.overload
    def add_mounted_camera(self, name: str, actor: ActorBase, pose: Pose, width: int, height: int, fovx: float, fovy: float, near: float, far: float) -> CameraEntity: ...
    @typing.overload
    def add_mounted_camera(self, name: str, actor: ActorBase, pose: Pose, width: int, height: int, fovy: float, near: float, far: float) -> CameraEntity: ...
    def add_particle_entity(self, positions: numpy.ndarray[numpy.float32, _Shape[m, 3]]) -> ParticleEntity: ...
    def add_point_light(self, position: numpy.ndarray[numpy.float32], color: numpy.ndarray[numpy.float32], shadow: bool = False, near: float = 0.1, far: float = 10, shadow_map_size: int = 2048) -> PointLightEntity: ...
    def add_spot_light(self, position: numpy.ndarray[numpy.float32], direction: numpy.ndarray[numpy.float32], inner_fov: float, outer_fov: float, color: numpy.ndarray[numpy.float32], shadow: bool = False, near: float = 0.10000000149011612, far: float = 10.0, shadow_map_size: int = 2048) -> SpotLightEntity: ...
    def create_actor_builder(self) -> ActorBuilder: ...
    def create_articulation_builder(self) -> ArticulationBuilder: ...
    def create_drive(self, actor1: ActorBase, pose1: Pose, actor2: ActorBase, pose2: Pose) -> Drive: ...
    def create_gear(self, actor1: ActorDynamicBase, pose1: Pose, actor2: ActorDynamicBase, pose2: Pose) -> Gear: ...
    def create_physical_material(self, static_friction: float, dynamic_friction: float, restitution: float) -> PhysicalMaterial: ...
    def create_urdf_loader(self) -> URDFLoader: ...
    def find_actor_by_id(self, id: int) -> ActorBase: ...
    def find_articulation_link_by_link_id(self, id: int) -> LinkBase: ...
    def generate_unique_render_id(self) -> int: ...
    def get_all_actors(self) -> typing.List[ActorBase]: ...
    def get_all_articulations(self) -> typing.List[ArticulationBase]: ...
    def get_all_lights(self) -> typing.List[LightEntity]: ...
    def get_cameras(self) -> typing.List[CameraEntity]: ...
    def get_contacts(self) -> typing.List[Contact]: ...
    def get_mounted_cameras(self) -> typing.List[CameraEntity]: ...
    def get_renderer_scene(self) -> RenderScene: ...
    def get_timestep(self) -> float: ...
    def multistep_async(self, arg0: int, arg1: capsule) -> AwaitableVoid: ...
    def pack(self) -> typing.Dict[str, typing.Dict[int, typing.List[float]]]: ...
    def remove_actor(self, actor: ActorBase) -> None: ...
    def remove_articulation(self, articulation: Articulation) -> None: ...
    def remove_camera(self, camera: CameraEntity) -> None: ...
    def remove_drive(self, drive: Constraint) -> None: ...
    def remove_kinematic_articulation(self, kinematic_articulation: KinematicArticulation) -> None: ...
    def remove_light(self, light: LightEntity) -> None: ...
    def remove_particle_entity(self, entity: ParticleEntity) -> None: ...
    def set_ambient_light(self, color: numpy.ndarray[numpy.float32]) -> None: ...
    def set_environment_map(self, filename: str) -> None: ...
    def set_environment_map_from_files(self, px: str, nx: str, py: str, ny: str, pz: str, nz: str) -> None: ...
    def set_timestep(self, second: float) -> None: ...
    def step(self) -> None: ...
    def step_async(self) -> AwaitableVoid: ...
    def unpack(self, data: typing.Dict[str, typing.Dict[int, typing.List[float]]]) -> None: ...
    def update_render(self) -> None: ...
    def update_render_async(self) -> AwaitableVoid: ...
    @property
    def _ptr(self) -> capsule:
        """
        :type: capsule
        """
    @property
    def ambient_light(self) -> numpy.ndarray[numpy.float32]:
        """
        :type: numpy.ndarray[numpy.float32]
        """
    @property
    def default_physical_material(self) -> PhysicalMaterial:
        """
        :type: PhysicalMaterial
        """
    @default_physical_material.setter
    def default_physical_material(self, arg1: PhysicalMaterial) -> None:
        pass
    @property
    def engine(self) -> Engine:
        """
        :type: Engine
        """
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
    def renderer_scene(self) -> RenderScene:
        """
        :type: RenderScene
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
    def __repr__(self) -> str: ...
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
    def disable_collision_visual(self) -> bool:
        """
        :type: bool
        """
    @disable_collision_visual.setter
    def disable_collision_visual(self, arg0: bool) -> None:
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
class SceneMultistepCallback():
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
    def length(self) -> float:
        """
        :type: float
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
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class SphereGeometry(CollisionGeometry):
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class SpotLightEntity(LightEntity, Entity):
    def set_fov(self, arg0: float) -> None: ...
    def set_shadow_parameters(self, near: float, far: float) -> None: ...
    @property
    def fov(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_far(self) -> float:
        """
        :type: float
        """
    @property
    def shadow_near(self) -> float:
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
    def load(self, filename: str, config: dict = {}) -> Articulation: 
        """
        Load articulation from URDF.
        Gazebo cameras are also loaded.

        Args:
          filename: path to URDF
          config: a dict to specify any on-the-fly modification of articulation
            It follows the following schema (the inner parameter overrides the outer one):
            - material: PhysicalMaterial
            - density: float
            - link: dict[str, dict]
              - ${link_name}: dict
                - material: PhysicalMaterial
                - density: float
                - patch_radius: float
                - min_patch_radius: float
                - collision: dict[int, dict]
                  - ${collision_index}: dict
                    - material: PhysicalMaterial
                    - density: float
                    - patch_radius: float
                    - min_patch_radius: float
        """
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
    def fix_root_link(self) -> bool:
        """
        :type: bool
        """
    @fix_root_link.setter
    def fix_root_link(self, arg0: bool) -> None:
        pass
    @property
    def load_multiple_collisions_from_file(self) -> bool:
        """
        :type: bool
        """
    @load_multiple_collisions_from_file.setter
    def load_multiple_collisions_from_file(self, arg0: bool) -> None:
        pass
    @property
    def package_dir(self) -> str:
        """
        :type: str
        """
    @package_dir.setter
    def package_dir(self, arg0: str) -> None:
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
    @property
    def type(self) -> str:
        """
        :type: str
        """
    pass
class VulkanParticleBody(RenderParticleBody):
    @property
    def dl_vertices(self) -> capsule:
        """
        :type: capsule
        """
    pass
class VulkanRenderMesh(RenderMesh):
    @property
    def dl_vertices(self) -> capsule:
        """
        :type: capsule
        """
    pass
class VulkanRigidbody(RenderBody):
    @property
    def _internal_objects(self) -> typing.List[renderer.Object]:
        """
        :type: typing.List[renderer.Object]
        """
    pass
class VulkanScene(RenderScene):
    @property
    def _internal_scene(self) -> renderer.Scene:
        """
        :type: renderer.Scene
        """
    pass
class VulkanWindow():
    def get_camera_position(self) -> numpy.ndarray[numpy.float32]: ...
    def get_camera_projection_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_camera_rotation(self) -> numpy.ndarray[numpy.float32]: ...
    def get_float_texture(self, name: str) -> numpy.ndarray[numpy.float32]: ...
    def get_float_texture_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.float32]: ...
    def get_target_size(self, name: str) -> typing.List[int[2]]: ...
    def get_uint32_texture(self, name: str) -> numpy.ndarray[numpy.uint32]: ...
    def get_uint32_texture_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.uint32]: ...
    def hide(self) -> None: ...
    def key_down(self, key: str) -> bool: ...
    def key_press(self, key: str) -> bool: ...
    def mouse_click(self, key: int) -> bool: ...
    def mouse_down(self, key: int) -> bool: ...
    def render(self, target_name: str, ui_windows: typing.List[renderer.UIWindow] = []) -> None: ...
    def resize(self, width: int, height: int) -> None: ...
    def set_camera_parameters(self, near: float, far: float, fovy: float) -> None: ...
    def set_camera_position(self, position: numpy.ndarray[numpy.float32]) -> None: ...
    @typing.overload
    def set_camera_property(self, key: str, value: float) -> None: ...
    @typing.overload
    def set_camera_property(self, key: str, value: int) -> None: ...
    def set_camera_rotation(self, quat: numpy.ndarray[numpy.float32]) -> None: ...
    def set_drop_callback(self, callback: typing.Callable[[typing.List[str]], None]) -> None: ...
    def set_intrinsic_parameters(self, near: float, far: float, fx: float, fy: float, cx: float, cy: float, skew: float) -> None: ...
    def set_scene(self, scene: Scene) -> None: ...
    def set_shader_dir(self, shader_dir: str) -> None: ...
    def show(self) -> None: ...
    def unset_drop_callback(self) -> None: ...
    @property
    def _internal_renderer(self) -> renderer.Renderer:
        """
        :type: renderer.Renderer
        """
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
    def cursor(self) -> bool:
        """
        :type: bool
        """
    @cursor.setter
    def cursor(self, arg1: bool) -> None:
        pass
    @property
    def display_target_names(self) -> typing.List[str]:
        """
        Names for available display targets that can be displayed in the render function

        :type: typing.List[str]
        """
    @property
    def far(self) -> float:
        """
        :type: float
        """
    @property
    def fovy(self) -> float:
        """
        :type: float
        """
    @property
    def fps(self) -> float:
        """
        :type: float
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
    def near(self) -> float:
        """
        :type: float
        """
    @property
    def shift(self) -> bool:
        """
        :type: bool
        """
    @property
    def should_close(self) -> bool:
        """
        :type: bool
        """
    @property
    def size(self) -> typing.List[int[2]]:
        """
        :type: typing.List[int[2]]
        """
    @property
    def super(self) -> bool:
        """
        :type: bool
        """
    pass
def add_profiler_event(name: str) -> None:
    pass
def get_global_render_config() -> RenderConfig:
    pass
