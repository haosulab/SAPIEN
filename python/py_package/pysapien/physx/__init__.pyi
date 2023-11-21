from __future__ import annotations
import sapien.pysapien.physx
import typing
import numpy
import sapien.pysapien
_Shape = typing.Tuple[int, ...]
_T = typing.TypeVar("T")

__all__ = [
    "PhysxArticulation",
    "PhysxArticulationJoint",
    "PhysxArticulationLinkComponent",
    "PhysxBaseComponent",
    "PhysxCollisionShape",
    "PhysxCollisionShapeBox",
    "PhysxCollisionShapeCapsule",
    "PhysxCollisionShapeConvexMesh",
    "PhysxCollisionShapeCylinder",
    "PhysxCollisionShapePlane",
    "PhysxCollisionShapeSphere",
    "PhysxCollisionShapeTriangleMesh",
    "PhysxContact",
    "PhysxContactPoint",
    "PhysxDistanceJointComponent",
    "PhysxDriveComponent",
    "PhysxEngine",
    "PhysxGearComponent",
    "PhysxJointComponent",
    "PhysxMaterial",
    "PhysxRayHit",
    "PhysxRigidBaseComponent",
    "PhysxRigidBodyComponent",
    "PhysxRigidDynamicComponent",
    "PhysxRigidStaticComponent",
    "PhysxSceneConfig",
    "PhysxSystem",
    "get_default_material",
    "set_default_material"
]


class PhysxArticulation():
    def compute_passive_force(self, gravity: bool = True, coriolis_and_centrifugal: bool = True) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def create_fixed_tendon(self, link_chain: list[PhysxArticulationLinkComponent], coefficients: list[float], recip_coefficients: list[float], rest_length: float = 0, offset: float = 0, stiffness: float = 0, damping: float = 0, low: float = -3.4028234663852886e+38, high: float = 3.4028234663852886e+38, limit_stiffness: float = 0) -> None: ...
    def find_joint_by_name(self, arg0: str) -> PhysxArticulationJoint: ...
    def find_link_by_name(self, arg0: str) -> PhysxArticulationLinkComponent: ...
    def get_active_joints(self) -> list[PhysxArticulationJoint]: ...
    def get_dof(self) -> int: ...
    def get_joints(self) -> list[PhysxArticulationJoint]: ...
    def get_links(self) -> list[PhysxArticulationLinkComponent]: ...
    def get_name(self) -> str: ...
    def get_pose(self) -> sapien.pysapien.Pose: ...
    def get_qacc(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_qf(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_qlimit(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]: 
        """
        same as get_qlimit
        """
    def get_qlimits(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]: ...
    def get_qpos(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_qvel(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_root(self) -> PhysxArticulationLinkComponent: ...
    def get_root_angular_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_root_pose(self) -> sapien.pysapien.Pose: ...
    def get_root_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def set_name(self, arg0: str) -> None: ...
    def set_pose(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_qacc(self, qacc: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def set_qf(self, qf: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def set_qpos(self, qpos: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def set_qvel(self, qvel: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def set_root_angular_velocity(self, velocity: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_root_pose(self, pose: sapien.pysapien.Pose) -> None: ...
    def set_root_velocity(self, velocity: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    @property
    def active_joints(self) -> list[PhysxArticulationJoint]:
        """
        :type: list[PhysxArticulationJoint]
        """
    @property
    def dof(self) -> int:
        """
        :type: int
        """
    @property
    def joints(self) -> list[PhysxArticulationJoint]:
        """
        :type: list[PhysxArticulationJoint]
        """
    @property
    def links(self) -> list[PhysxArticulationLinkComponent]:
        """
        :type: list[PhysxArticulationLinkComponent]
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
    def pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @pose.setter
    def pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def qacc(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @qacc.setter
    def qacc(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
    @property
    def qf(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @qf.setter
    def qf(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
    @property
    def qlimit(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 2]]
        """
    @property
    def qlimits(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 2]]
        """
    @property
    def qpos(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @qpos.setter
    def qpos(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
    @property
    def qvel(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @qvel.setter
    def qvel(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
    @property
    def root(self) -> PhysxArticulationLinkComponent:
        """
        :type: PhysxArticulationLinkComponent
        """
    @property
    def root_angular_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @root_angular_velocity.setter
    def root_angular_velocity(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def root_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @root_pose.setter
    def root_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def root_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @root_velocity.setter
    def root_velocity(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    pass
class PhysxArticulationJoint():
    def get_armature(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_child_link(self) -> PhysxArticulationLinkComponent: ...
    def get_damping(self) -> float: ...
    def get_dof(self) -> int: ...
    def get_drive_mode(self) -> typing.Literal['force', 'acceleration']: ...
    def get_drive_target(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_drive_velocity_target(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def get_force_limit(self) -> float: ...
    def get_friction(self) -> float: ...
    def get_global_pose(self) -> sapien.pysapien.Pose: ...
    def get_limit(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]: 
        """
        same as get_limits
        """
    def get_limits(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]: ...
    def get_name(self) -> str: ...
    def get_parent_link(self) -> PhysxArticulationLinkComponent: ...
    def get_pose_in_child(self) -> sapien.pysapien.Pose: ...
    def get_pose_in_parent(self) -> sapien.pysapien.Pose: ...
    def get_stiffness(self) -> float: ...
    def get_type(self) -> typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']: ...
    def set_armature(self, armature: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def set_drive_properties(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_property(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: 
        """
        same as set_drive_properties
        """
    @typing.overload
    def set_drive_target(self, target: float) -> None: ...
    @typing.overload
    def set_drive_target(self, target: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    @typing.overload
    def set_drive_velocity_target(self, velocity: float) -> None: ...
    @typing.overload
    def set_drive_velocity_target(self, velocity: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def set_friction(self, friction: float) -> None: ...
    def set_limit(self, limit: numpy.ndarray[numpy.float32, _Shape[m, 2]]) -> None: 
        """
        same as set_limits
        """
    def set_limits(self, limit: numpy.ndarray[numpy.float32, _Shape[m, 2]]) -> None: ...
    def set_name(self, name: str) -> None: ...
    def set_pose_in_child(self, pose: sapien.pysapien.Pose) -> None: ...
    def set_pose_in_parent(self, pose: sapien.pysapien.Pose) -> None: ...
    def set_type(self, type: typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']) -> None: ...
    @property
    def armature(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @armature.setter
    def armature(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
    @property
    def child_link(self) -> PhysxArticulationLinkComponent:
        """
        :type: PhysxArticulationLinkComponent
        """
    @property
    def damping(self) -> float:
        """
        :type: float
        """
    @property
    def dof(self) -> int:
        """
        :type: int
        """
    @property
    def drive_mode(self) -> typing.Literal['force', 'acceleration']:
        """
        :type: typing.Literal['force', 'acceleration']
        """
    @property
    def drive_target(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @drive_target.setter
    def drive_target(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
    @property
    def drive_velocity_target(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 1]]
        """
    @drive_velocity_target.setter
    def drive_velocity_target(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None:
        pass
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
    @friction.setter
    def friction(self, arg1: float) -> None:
        pass
    @property
    def global_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @property
    def limit(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 2]]
        """
    @limit.setter
    def limit(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 2]]) -> None:
        pass
    @property
    def limits(self) -> numpy.ndarray[numpy.float32, _Shape[m, 2]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 2]]
        """
    @limits.setter
    def limits(self, arg1: numpy.ndarray[numpy.float32, _Shape[m, 2]]) -> None:
        pass
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def parent_link(self) -> PhysxArticulationLinkComponent:
        """
        :type: PhysxArticulationLinkComponent
        """
    @property
    def pose_in_child(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @pose_in_child.setter
    def pose_in_child(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def pose_in_parent(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @pose_in_parent.setter
    def pose_in_parent(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def stiffness(self) -> float:
        """
        :type: float
        """
    @property
    def type(self) -> typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']:
        """
        :type: typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']
        """
    @type.setter
    def type(self, arg1: typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']) -> None:
        pass
    pass
class PhysxBaseComponent(sapien.pysapien.Component):
    pass
class PhysxRigidBaseComponent(PhysxBaseComponent, sapien.pysapien.Component):
    def attach(self, collision_shape: PhysxCollisionShape) -> PhysxRigidBaseComponent: ...
    def compute_global_aabb_tight(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[2, 3]]: ...
    def get_collision_shapes(self) -> list[PhysxCollisionShape]: ...
    def get_global_aabb_fast(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[2, 3]]: ...
    @property
    def collision_shapes(self) -> list[PhysxCollisionShape]:
        """
        :type: list[PhysxCollisionShape]
        """
    pass
class PhysxCollisionShape():
    def get_collision_groups(self) -> list[int]: ...
    def get_contact_offset(self) -> float: ...
    def get_density(self) -> float: ...
    def get_local_pose(self) -> sapien.pysapien.Pose: ...
    def get_min_patch_radius(self) -> float: ...
    def get_patch_radius(self) -> float: ...
    def get_physical_material(self) -> PhysxMaterial: ...
    def get_rest_offset(self) -> float: ...
    def set_collision_groups(self, groups: list[int]) -> None: 
        """
        collision groups determine the collision behavior of objects. Let A.gx denote the collision group x of collision shape A. Collision shape A and B will collide iff the following condition holds:

        ((A.g0 & B.g1) or (A.g1 & B.g0)) and (not ((A.g2 & B.g2) and ((A.g3 & 0xffff) == (B.g3 & 0xffff))))

        Here is some explanation: g2 is the "ignore group" and g3 is the "id group". The only the lower 16 bits of the id group is used since the upper 16 bits are reserved for other purposes in the future. When 2 collision shapes have the same ID (g3), then if any of their g2 bits match, their collisions are definitely ignored.

        If after testing g2 and g3, the objects may collide, g0 and g1 come into play. g0 is the "contact type group" and g1 is the "contact affinity group". Collision shapes collide only when a bit in the contact type of the first shape matches a bit in the contact affinity of the second shape.
        """
    def set_contact_offset(self, offset: float) -> None: ...
    def set_density(self, arg0: float) -> None: ...
    def set_local_pose(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_min_patch_radius(self, radius: float) -> None: ...
    def set_patch_radius(self, radius: float) -> None: ...
    def set_physical_material(self, material: PhysxMaterial) -> None: ...
    def set_rest_offset(self, offset: float) -> None: ...
    @property
    def collision_groups(self) -> list[int]:
        """
        :type: list[int]
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
    def density(self) -> float:
        """
        :type: float
        """
    @density.setter
    def density(self, arg1: float) -> None:
        pass
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
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
    def physical_material(self) -> PhysxMaterial:
        """
        :type: PhysxMaterial
        """
    @physical_material.setter
    def physical_material(self, arg1: PhysxMaterial) -> None:
        pass
    @property
    def rest_offset(self) -> float:
        """
        :type: float
        """
    @rest_offset.setter
    def rest_offset(self, arg1: float) -> None:
        pass
    pass
class PhysxCollisionShapeBox(PhysxCollisionShape):
    def __init__(self, half_size: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: PhysxMaterial) -> None: ...
    def get_half_size(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    @property
    def half_size(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class PhysxCollisionShapeCapsule(PhysxCollisionShape):
    def __init__(self, radius: float, half_length: float, material: PhysxMaterial) -> None: ...
    def get_half_length(self) -> float: ...
    def get_radius(self) -> float: ...
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
class PhysxCollisionShapeConvexMesh(PhysxCollisionShape):
    def __init__(self, filename: str, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: PhysxMaterial) -> None: ...
    def get_scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_triangles(self) -> numpy.ndarray[numpy.uint32, _Shape[m, 3]]: ...
    def get_vertices(self) -> numpy.ndarray[numpy.float32, _Shape[m, 3]]: ...
    @staticmethod
    def load_multiple(filename: str, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: PhysxMaterial) -> list[PhysxCollisionShapeConvexMesh]: ...
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def triangles(self) -> numpy.ndarray[numpy.uint32, _Shape[m, 3]]:
        """
        :type: numpy.ndarray[numpy.uint32, _Shape[m, 3]]
        """
    @property
    def vertices(self) -> numpy.ndarray[numpy.float32, _Shape[m, 3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 3]]
        """
    pass
class PhysxCollisionShapeCylinder(PhysxCollisionShape):
    def __init__(self, radius: float, half_length: float, material: PhysxMaterial) -> None: ...
    def get_half_length(self) -> float: ...
    def get_radius(self) -> float: ...
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
class PhysxCollisionShapePlane(PhysxCollisionShape):
    def __init__(self, material: PhysxMaterial) -> None: ...
    pass
class PhysxCollisionShapeSphere(PhysxCollisionShape):
    def __init__(self, radius: float, material: PhysxMaterial) -> None: ...
    def get_radius(self) -> float: ...
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class PhysxCollisionShapeTriangleMesh(PhysxCollisionShape):
    def __init__(self, filename: str, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: PhysxMaterial) -> None: ...
    def get_scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_triangles(self) -> numpy.ndarray[numpy.uint32, _Shape[m, 3]]: ...
    def get_vertices(self) -> numpy.ndarray[numpy.float32, _Shape[m, 3]]: ...
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def triangles(self) -> numpy.ndarray[numpy.uint32, _Shape[m, 3]]:
        """
        :type: numpy.ndarray[numpy.uint32, _Shape[m, 3]]
        """
    @property
    def vertices(self) -> numpy.ndarray[numpy.float32, _Shape[m, 3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 3]]
        """
    pass
class PhysxContact():
    def __repr__(self) -> str: ...
    @property
    def components(self) -> typing.Annotated[list[PhysxRigidBaseComponent], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[list[PhysxRigidBaseComponent], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @property
    def points(self) -> list[PhysxContactPoint]:
        """
        :type: list[PhysxContactPoint]
        """
    @property
    def shapes(self) -> typing.Annotated[list[PhysxCollisionShape], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[list[PhysxCollisionShape], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    pass
class PhysxContactPoint():
    @property
    def impulse(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def normal(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def position(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def separation(self) -> float:
        """
        :type: float
        """
    pass
class PhysxJointComponent(PhysxBaseComponent, sapien.pysapien.Component):
    def get_parent(self) -> PhysxRigidBaseComponent: ...
    def get_pose_in_child(self) -> sapien.pysapien.Pose: ...
    def get_pose_in_parent(self) -> sapien.pysapien.Pose: ...
    def get_relative_pose(self) -> sapien.pysapien.Pose: ...
    def set_inv_inertia_scales(self, scale0: float, scale1: float) -> None: ...
    def set_inv_mass_scales(self, scale0: float, scale1: float) -> None: ...
    def set_parent(self, parent: PhysxRigidBaseComponent) -> None: ...
    def set_pose_in_child(self, pose: sapien.pysapien.Pose) -> None: ...
    def set_pose_in_parent(self, pose: sapien.pysapien.Pose) -> None: ...
    @property
    def parent(self) -> PhysxRigidBaseComponent:
        """
        :type: PhysxRigidBaseComponent
        """
    @parent.setter
    def parent(self, arg1: PhysxRigidBaseComponent) -> None:
        pass
    @property
    def pose_in_child(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @pose_in_child.setter
    def pose_in_child(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def pose_in_parent(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @pose_in_parent.setter
    def pose_in_parent(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def relative_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    pass
class PhysxDriveComponent(PhysxJointComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def __init__(self, body: PhysxRigidBodyComponent) -> None: ...
    def get_drive_property_slerp(self) -> tuple[float, float, float, typing.Literal['force', 'acceleration']]: ...
    def get_drive_property_swing(self) -> tuple[float, float, float, typing.Literal['force', 'acceleration']]: ...
    def get_drive_property_twist(self) -> tuple[float, float, float, typing.Literal['force', 'acceleration']]: ...
    def get_drive_property_x(self) -> tuple[float, float, float, typing.Literal['force', 'acceleration']]: ...
    def get_drive_property_y(self) -> tuple[float, float, float, typing.Literal['force', 'acceleration']]: ...
    def get_drive_property_z(self) -> tuple[float, float, float, typing.Literal['force', 'acceleration']]: ...
    def get_drive_target(self) -> sapien.pysapien.Pose: ...
    def get_drive_velocity_target(self) -> tuple[numpy.ndarray[numpy.float32, _Shape, _Shape[3]], numpy.ndarray[numpy.float32, _Shape, _Shape[3]]]: ...
    def get_limit_cone(self) -> tuple[float, float, float, float]: ...
    def get_limit_pyramid(self) -> tuple[float, float, float, float, float, float]: ...
    def get_limit_twist(self) -> tuple[float, float, float, float]: ...
    def get_limit_x(self) -> tuple[float, float, float, float]: ...
    def get_limit_y(self) -> tuple[float, float, float, float]: ...
    def get_limit_z(self) -> tuple[float, float, float, float]: ...
    def set_drive_property_slerp(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_property_swing(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_property_twist(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_property_x(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_property_y(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_property_z(self, stiffness: float, damping: float, force_limit: float = 3.4028234663852886e+38, mode: typing.Literal['force', 'acceleration'] = 'force') -> None: ...
    def set_drive_target(self, target: sapien.pysapien.Pose) -> None: ...
    def set_drive_velocity_target(self, linear: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], angular: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_limit_cone(self, angle_y: float, angle_z: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    def set_limit_pyramid(self, low_y: float, high_y: float, low_z: float, high_z: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    def set_limit_twist(self, low: float, high: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    def set_limit_x(self, low: float, high: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    def set_limit_y(self, low: float, high: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    def set_limit_z(self, low: float, high: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    @property
    def drive_target(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @drive_target.setter
    def drive_target(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    pass
class PhysxEngine():
    def __init__(self, tolerance_length: float, tolerance_speed: float) -> None: ...
    pass
class PhysxGearComponent(PhysxJointComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def __init__(self, body: PhysxRigidBodyComponent) -> None: ...
    def enable_hinges(self) -> None: ...
    def get_gear_ratio(self) -> float: ...
    def set_gear_ratio(self, ratio: float) -> None: ...
    @property
    def gear_ratio(self) -> float:
        """
        :type: float
        """
    @gear_ratio.setter
    def gear_ratio(self, arg1: float) -> None:
        pass
    @property
    def is_hinges_enabled(self) -> bool:
        """
        :type: bool
        """
    pass
class PhysxDistanceJointComponent(PhysxJointComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def __init__(self, body: PhysxRigidBodyComponent) -> None: ...
    def get_distance(self) -> float: ...
    def set_limit(self, low: float, high: float, stiffness: float = 0.0, damping: float = 0.0) -> None: ...
    @property
    def distance(self) -> float:
        """
        :type: float
        """
    pass
class PhysxMaterial():
    def __init__(self, static_friction: float, dynamic_friction: float, restitution: float) -> None: ...
    def get_dynamic_friction(self) -> float: ...
    def get_restitution(self) -> float: ...
    def get_static_friction(self) -> float: ...
    def set_dynamic_friction(self, arg0: float) -> None: ...
    def set_restitution(self, arg0: float) -> None: ...
    def set_static_friction(self, arg0: float) -> None: ...
    @property
    def dynamic_friction(self) -> float:
        """
        :type: float
        """
    @dynamic_friction.setter
    def dynamic_friction(self, arg1: float) -> None:
        pass
    @property
    def restitution(self) -> float:
        """
        :type: float
        """
    @restitution.setter
    def restitution(self, arg1: float) -> None:
        pass
    @property
    def static_friction(self) -> float:
        """
        :type: float
        """
    @static_friction.setter
    def static_friction(self, arg1: float) -> None:
        pass
    pass
class PhysxRayHit():
    def __repr__(self) -> str: ...
    @property
    def component(self) -> PhysxRigidBaseComponent:
        """
        :type: PhysxRigidBaseComponent
        """
    @property
    def distance(self) -> float:
        """
        :type: float
        """
    @property
    def normal(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def position(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def shape(self) -> PhysxCollisionShape:
        """
        :type: PhysxCollisionShape
        """
    pass
class PhysxRigidBodyComponent(PhysxRigidBaseComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def add_force_at_point(self, force: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], point: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], mode: typing.Literal['force', 'acceleration', 'velocity_change', 'impulse'] = 'force') -> None: ...
    def add_force_torque(self, force: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], torque: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], mode: typing.Literal['force', 'acceleration', 'velocity_change', 'impulse'] = 'force') -> None: ...
    def get_angular_damping(self) -> float: ...
    def get_angular_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_auto_compute_mass(self) -> bool: ...
    def get_cmass_local_pose(self) -> sapien.pysapien.Pose: ...
    def get_disable_gravity(self) -> bool: ...
    def get_inertia(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_linear_damping(self) -> float: ...
    def get_linear_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_mass(self) -> float: ...
    def get_max_contact_impulse(self) -> float: ...
    def get_max_depenetraion_velocity(self) -> float: ...
    def set_angular_damping(self, damping: float) -> None: ...
    def set_cmass_local_pose(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_disable_gravity(self, arg0: bool) -> None: ...
    def set_inertia(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_linear_damping(self, damping: float) -> None: ...
    def set_mass(self, arg0: float) -> None: ...
    def set_max_contact_impulse(self, impulse: float) -> None: ...
    def set_max_depenetraion_velocity(self, velocity: float) -> None: ...
    @property
    def angular_damping(self) -> float:
        """
        :type: float
        """
    @angular_damping.setter
    def angular_damping(self, arg1: float) -> None:
        pass
    @property
    def angular_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def auto_compute_mass(self) -> bool:
        """
        :type: bool
        """
    @property
    def cmass_local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @cmass_local_pose.setter
    def cmass_local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def disable_gravity(self) -> bool:
        """
        :type: bool
        """
    @disable_gravity.setter
    def disable_gravity(self, arg1: bool) -> None:
        pass
    @property
    def inertia(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @inertia.setter
    def inertia(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def linear_damping(self) -> float:
        """
        :type: float
        """
    @linear_damping.setter
    def linear_damping(self, arg1: float) -> None:
        pass
    @property
    def linear_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @property
    def mass(self) -> float:
        """
        :type: float
        """
    @mass.setter
    def mass(self, arg1: float) -> None:
        pass
    @property
    def max_contact_impulse(self) -> float:
        """
        :type: float
        """
    @max_contact_impulse.setter
    def max_contact_impulse(self, arg1: float) -> None:
        pass
    @property
    def max_depenetraion_velocity(self) -> float:
        """
        :type: float
        """
    @max_depenetraion_velocity.setter
    def max_depenetraion_velocity(self, arg1: float) -> None:
        pass
    pass
class PhysxArticulationLinkComponent(PhysxRigidBodyComponent, PhysxRigidBaseComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def __init__(self, parent: PhysxArticulationLinkComponent = None) -> None: ...
    def get_articulation(self) -> PhysxArticulation: ...
    def get_children(self) -> list[PhysxArticulationLinkComponent]: ...
    def get_index(self) -> int: ...
    def get_joint(self) -> PhysxArticulationJoint: ...
    def get_parent(self) -> PhysxArticulationLinkComponent: ...
    def put_to_sleep(self) -> None: ...
    def set_parent(self, parent: PhysxArticulationLinkComponent) -> None: ...
    def wake_up(self) -> None: ...
    @property
    def articulation(self) -> PhysxArticulation:
        """
        :type: PhysxArticulation
        """
    @property
    def children(self) -> list[PhysxArticulationLinkComponent]:
        """
        :type: list[PhysxArticulationLinkComponent]
        """
    @property
    def index(self) -> int:
        """
        :type: int
        """
    @property
    def is_root(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint(self) -> PhysxArticulationJoint:
        """
        :type: PhysxArticulationJoint
        """
    @property
    def parent(self) -> PhysxArticulationLinkComponent:
        """
        :type: PhysxArticulationLinkComponent
        """
    @property
    def sleeping(self) -> bool:
        """
        :type: bool
        """
    pass
class PhysxRigidDynamicComponent(PhysxRigidBodyComponent, PhysxRigidBaseComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def get_angular_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_kinematic(self) -> bool: ...
    def get_kinematic_target(self) -> sapien.pysapien.Pose: ...
    def get_linear_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_locked_motion_axes(self) -> list[bool]: ...
    def put_to_sleep(self) -> None: ...
    def set_angular_velocity(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_kinematic(self, arg0: bool) -> None: ...
    def set_kinematic_target(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_linear_velocity(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_locked_motion_axes(self, axes: list[bool]) -> None: 
        """
        set some motion axes of the dynamic rigid body to be locked
        Args:
            axes: list of 6 true/false values indicating whether which  of the 6 DOFs of the body is locked.
                  The order is linear X, Y, Z followed by angular X, Y, Z.

        Example:
            set_locked_motion_axes([True, False, False, False, True, False]) allows the object to move along the X axis and rotate about the Y axis
        """
    def wake_up(self) -> None: ...
    @property
    def angular_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @angular_velocity.setter
    def angular_velocity(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def is_sleeping(self) -> bool:
        """
        :type: bool
        """
    @property
    def kinematic(self) -> bool:
        """
        :type: bool
        """
    @kinematic.setter
    def kinematic(self, arg1: bool) -> None:
        pass
    @property
    def kinematic_target(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @kinematic_target.setter
    def kinematic_target(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def linear_velocity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @linear_velocity.setter
    def linear_velocity(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def locked_motion_axes(self) -> list[bool]:
        """
        :type: list[bool]
        """
    pass
class PhysxRigidStaticComponent(PhysxRigidBaseComponent, PhysxBaseComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    pass
class PhysxSceneConfig():
    def __getstate__(self) -> tuple: ...
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, arg0: tuple) -> None: ...
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
    def gravity(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @gravity.setter
    def gravity(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
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
class PhysxSystem(sapien.pysapien.System):
    def __init__(self, config: PhysxSceneConfig = SceneConfig()) -> None: ...
    def get_articulation_link_components(self) -> list[PhysxArticulationLinkComponent]: ...
    def get_config(self) -> PhysxSceneConfig: ...
    def get_contacts(self) -> list[PhysxContact]: ...
    def get_rigid_dynamic_components(self) -> list[PhysxRigidDynamicComponent]: ...
    def get_rigid_static_components(self) -> list[PhysxRigidStaticComponent]: ...
    def get_timestep(self) -> float: ...
    def pack(self) -> bytes: ...
    def raycast(self, position: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], direction: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], distance: float) -> PhysxRayHit: 
        """
        Casts a ray and returns the closest hit. Returns None if no hit
        """
    def set_timestep(self, arg0: float) -> None: ...
    def unpack(self, data: bytes) -> None: ...
    @property
    def articulation_link_components(self) -> list[PhysxArticulationLinkComponent]:
        """
        :type: list[PhysxArticulationLinkComponent]
        """
    @property
    def config(self) -> PhysxSceneConfig:
        """
        :type: PhysxSceneConfig
        """
    @property
    def rigid_dynamic_components(self) -> list[PhysxRigidDynamicComponent]:
        """
        :type: list[PhysxRigidDynamicComponent]
        """
    @property
    def rigid_static_components(self) -> list[PhysxRigidStaticComponent]:
        """
        :type: list[PhysxRigidStaticComponent]
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
def get_default_material() -> PhysxMaterial:
    pass
def set_default_material(static_friction: float, dynamic_friction: float, restitution: float) -> None:
    pass
