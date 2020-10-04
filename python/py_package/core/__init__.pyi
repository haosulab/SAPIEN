import sapien.core
import typing
from sapien.core.pysapien import Actor
from sapien.core.pysapien import ActorBase
from sapien.core.pysapien import ActorBuilder
from sapien.core.pysapien import ActorDynamicBase
from sapien.core.pysapien import ActorStatic
from sapien.core.pysapien import ActorType
from sapien.core.pysapien import Articulation
from sapien.core.pysapien import ArticulationBase
from sapien.core.pysapien import ArticulationBuilder
from sapien.core.pysapien import ArticulationDrivable
from sapien.core.pysapien import ArticulationJointType
from sapien.core.pysapien import ArticulationType
from sapien.core.pysapien import BoxGeometry
from sapien.core.pysapien import CameraSpec
from sapien.core.pysapien import CapsuleGeometry
from sapien.core.pysapien import CollisionGeometry
from sapien.core.pysapien import CollisionShape
from sapien.core.pysapien import Contact
from sapien.core.pysapien import ContactPoint
from sapien.core.pysapien import ConvexMeshGeometry
from sapien.core.pysapien import Drive
from sapien.core.pysapien import Engine
from sapien.core.pysapien import ICamera
from sapien.core.pysapien import IPxrRenderer
from sapien.core.pysapien import IPxrScene
from sapien.core.pysapien import ISensor
from sapien.core.pysapien import Input
from sapien.core.pysapien import Joint
from sapien.core.pysapien import JointBase
from sapien.core.pysapien import JointRecord
from sapien.core.pysapien import KinematicArticulation
from sapien.core.pysapien import KinematicJoint
from sapien.core.pysapien import KinematicJointFixed
from sapien.core.pysapien import KinematicJointPrismatic
from sapien.core.pysapien import KinematicJointRevolute
from sapien.core.pysapien import KinematicJointSingleDof
from sapien.core.pysapien import KinematicLink
from sapien.core.pysapien import Link
from sapien.core.pysapien import LinkBase
from sapien.core.pysapien import LinkBuilder
from sapien.core.pysapien import OptifuserCamera
from sapien.core.pysapien import OptifuserConfig
from sapien.core.pysapien import OptifuserController
from sapien.core.pysapien import OptifuserRenderer
from sapien.core.pysapien import PinocchioModel
from sapien.core.pysapien import PlaneGeometry
from sapien.core.pysapien import Pose
from sapien.core.pysapien import PxMaterial
from sapien.core.pysapien import PxrMaterial
from sapien.core.pysapien import RenderBody
from sapien.core.pysapien import RenderGeometry
from sapien.core.pysapien import RenderShape
from sapien.core.pysapien import Scene
from sapien.core.pysapien import SceneConfig
from sapien.core.pysapien import ShapeRecord
from sapien.core.pysapien import SolverType
from sapien.core.pysapien import SphereGeometry
from sapien.core.pysapien import URDFLoader
from sapien.core.pysapien import VisualRecord
from sapien.core.pysapien import VulkanCamera
from sapien.core.pysapien import VulkanController
from sapien.core.pysapien import VulkanRenderer
import os
import pkg_resources
import sapien.core.pysapien
import sys

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
    "GL_SHADER_ROOT",
    "ICamera",
    "IPxrRenderer",
    "IPxrScene",
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
    "OptifuserRenderer",
    "PGS",
    "PRISMATIC",
    "PTX_ROOT",
    "PinocchioModel",
    "PlaneGeometry",
    "Pose",
    "PxMaterial",
    "PxrMaterial",
    "REVOLUTE",
    "RenderBody",
    "RenderGeometry",
    "RenderShape",
    "SPHERICAL",
    "SPV_ROOT",
    "STATIC",
    "Scene",
    "SceneConfig",
    "ShapeRecord",
    "SolverType",
    "SphereGeometry",
    "TGS",
    "UNDEFINED",
    "URDFLoader",
    "VisualRecord",
    "VulkanCamera",
    "VulkanController",
    "VulkanRenderer",
    "enable_default_gl3",
    "enable_default_gl4",
    "os",
    "pkg_resources",
    "pysapien",
    "sys"
]


DYNAMIC: sapien.core.pysapien.ArticulationType # value = ArticulationType.DYNAMIC
FIX: sapien.core.pysapien.ArticulationJointType # value = ArticulationJointType.FIX
GL_SHADER_ROOT = ''
KINEMATIC: sapien.core.pysapien.ArticulationType # value = ArticulationType.KINEMATIC
KINEMATIC_LINK: sapien.core.pysapien.ActorType # value = ActorType.KINEMATIC_LINK
LINK: sapien.core.pysapien.ActorType # value = ActorType.LINK
PGS: sapien.core.pysapien.SolverType # value = SolverType.PGS
PRISMATIC: sapien.core.pysapien.ArticulationJointType # value = ArticulationJointType.PRISMATIC
PTX_ROOT = ''
REVOLUTE: sapien.core.pysapien.ArticulationJointType # value = ArticulationJointType.REVOLUTE
SPHERICAL: sapien.core.pysapien.ArticulationJointType # value = ArticulationJointType.SPHERICAL
SPV_ROOT = ''
STATIC: sapien.core.pysapien.ActorType # value = ActorType.STATIC
TGS: sapien.core.pysapien.SolverType # value = SolverType.TGS
UNDEFINED: sapien.core.pysapien.ArticulationJointType # value = ArticulationJointType.UNDEFINED
__GL_VERSION_DICT = {3: '130', 4: '410'}
