from ..pysapien.physx import PhysxSystem, PhysxGpuSystem, PhysxCpuSystem, PhysxMaterial
from .. import pysapien as sapien
from ..pysapien.render import RenderSystem
from warnings import warn
from .scene import Scene
from ..pysapien.physx import PhysxSceneConfig as SceneConfig


class Engine:
    def __init__(self, **args):
        warn(
            "Engine is deprecated. use sapien.Scene() directly.",
            DeprecationWarning,
            stacklevel=2,
        )

    def set_renderer(self, renderer):
        self.renderer = renderer

    def create_physical_material(self, static_friction, dynamic_friction, restitution):
        return PhysxMaterial(static_friction, dynamic_friction, restitution)

    def create_scene(self, config=SceneConfig()):
        sapien.physx.set_scene_config(config)
        return Scene()
