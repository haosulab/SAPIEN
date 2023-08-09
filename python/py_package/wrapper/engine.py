from ..pysapien.physx import PhysxSystem, PhysxMaterial
from ..pysapien.render import RenderSystem
import warnings
from .scene import Scene
from ..pysapien.physx import PhysxSceneConfig as SceneConfig


class Engine:
    def __init__(self, **args):
        pass

    def set_renderer(self, renderer):
        self.renderer = renderer

    def create_physical_material(self, static_friction, dynamic_friction, restitution):
        return PhysxMaterial(static_friction, dynamic_friction, restitution)

    def create_mesh_geometry(self, vertices, indices, scale=[1, 1, 1]):
        # TODO
        pass

    def create_scene(self, config=SceneConfig()):
        return Scene([PhysxSystem(config), RenderSystem()])
