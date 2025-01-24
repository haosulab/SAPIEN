#
# Copyright 2025 Hillbot Inc.
# Copyright 2020-2024 UCSD SU Lab
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
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
