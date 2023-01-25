from __future__ import annotations
import sapien.core
import typing
from sapien.core.pysapien import ActiveLightEntity
from sapien.core.pysapien import Actor
from sapien.core.pysapien import ActorBase
from sapien.core.pysapien import ActorBuilder
from sapien.core.pysapien import ActorDynamicBase
from sapien.core.pysapien import ActorStatic
from sapien.core.pysapien import Articulation
from sapien.core.pysapien import ArticulationBase
from sapien.core.pysapien import ArticulationBuilder
from sapien.core.pysapien import ArticulationDrivable
from sapien.core.pysapien import AwaitableDLList
from sapien.core.pysapien import AwaitableVoid
from sapien.core.pysapien import BoxGeometry
from sapien.core.pysapien import CameraEntity
from sapien.core.pysapien import CapsuleGeometry
from sapien.core.pysapien import CollisionGeometry
from sapien.core.pysapien import CollisionShape
from sapien.core.pysapien import Constraint
from sapien.core.pysapien import Contact
from sapien.core.pysapien import ContactPoint
from sapien.core.pysapien import ConvexMeshGeometry
from sapien.core.pysapien import DirectionalLightEntity
from sapien.core.pysapien import Drive
from sapien.core.pysapien import Engine
from sapien.core.pysapien import Entity
from sapien.core.pysapien import Gear
from sapien.core.pysapien import IPxrRenderer
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
from sapien.core.pysapien import KuafuConfig
from sapien.core.pysapien import LightEntity
from sapien.core.pysapien import Link
from sapien.core.pysapien import LinkBase
from sapien.core.pysapien import LinkBuilder
from sapien.core.pysapien import NonconvexMeshGeometry
from sapien.core.pysapien import ParticleEntity
from sapien.core.pysapien import PhysicalMaterial
from sapien.core.pysapien import PinocchioModel
from sapien.core.pysapien import PlaneGeometry
from sapien.core.pysapien import PointLightEntity
from sapien.core.pysapien import Pose
from sapien.core.pysapien import ProfilerBlock
from sapien.core.pysapien import RenderBody
from sapien.core.pysapien import RenderClient
from sapien.core.pysapien import RenderConfig
from sapien.core.pysapien import RenderMaterial
from sapien.core.pysapien import RenderMesh
from sapien.core.pysapien import RenderParticleBody
from sapien.core.pysapien import RenderScene
from sapien.core.pysapien import RenderServer
from sapien.core.pysapien import RenderServerBuffer
from sapien.core.pysapien import RenderShape
from sapien.core.pysapien import RenderTexture
from sapien.core.pysapien import SapienRenderer
from sapien.core.pysapien import Scene
from sapien.core.pysapien import SceneConfig
from sapien.core.pysapien import SceneMultistepCallback
from sapien.core.pysapien import ShapeRecord
from sapien.core.pysapien import SphereGeometry
from sapien.core.pysapien import SpotLightEntity
from sapien.core.pysapien import Subscription
from sapien.core.pysapien import Trigger
from sapien.core.pysapien import URDFLoader
from sapien.core.pysapien import VisualRecord
from sapien.core.pysapien import VulkanParticleBody
from sapien.core.pysapien import VulkanRenderMesh
from sapien.core.pysapien import VulkanRigidbody
from sapien.core.pysapien import VulkanScene
from sapien.core.pysapien import VulkanWindow
from sapien.core.pysapien import KuafuRenderer as _KuafuRenderer
import os
import pkg_resources
import sapien.core.pysapien
import sapien.core.pysapien.dlpack
import sapien.core.pysapien.renderer
import sapien.core.pysapien.simsense
import sapien.core.renderer_config
import sys

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
    "List",
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
    "Union",
    "VisualRecord",
    "VulkanParticleBody",
    "VulkanRenderMesh",
    "VulkanRenderer",
    "VulkanRigidbody",
    "VulkanScene",
    "VulkanWindow",
    "add_profiler_event",
    "dlpack",
    "ensure_icd",
    "get_global_render_config",
    "os",
    "pkg_resources",
    "pysapien",
    "render_config",
    "renderer",
    "renderer_config",
    "simsense",
    "sys",
    "warn"
]


class KuafuRenderer(pysapien.IPxrRenderer):
    pass
class VulkanRenderer(pysapien.SapienRenderer, pysapien.IPxrRenderer):
    pass
def add_profiler_event(name: str) -> None:
    pass
def get_global_render_config() -> pysapien.RenderConfig:
    pass
List: typing._SpecialGenericAlias # value = typing.List
Union: typing._SpecialForm # value = typing.Union
render_config: sapien.core.renderer_config._RenderConfig
