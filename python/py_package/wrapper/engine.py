from ..pysapien.physx import PhysxSystem, PhysxGpuSystem, PhysxCpuSystem, PhysxMaterial
from .. import pysapien as sapien
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
        if sapien.physx.is_gpu_enabled():
            physx_system = Scene._GpuSystem
            if physx_system is None:
                Scene._GpuSystem = physx_system = sapien.physx.PhysxGpuSystem(config)
            else:
                physx_system = sapien.physx.PhysxCpuSystem(config)

        return Scene([physx_system, RenderSystem()])
