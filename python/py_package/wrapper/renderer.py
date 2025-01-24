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
from ..pysapien.render import SapienRenderer as _SapienRenderer
from ..pysapien.render import RenderMaterial
from warnings import warn


class SapienRenderer(_SapienRenderer):
    def __init__(self, **args):
        warn("SapienRenderer is no no longer needed.", DeprecationWarning, stacklevel=2)
        super().__init__()

    def create_material(self):
        return RenderMaterial()


def set_diffuse_texture(self, texture):
    warn(
        "diffuse_texture is renamed to base_color_texture",
        DeprecationWarning,
        stacklevel=2,
    )
    self.set_base_color_texture(texture)


def get_diffuse_texture(self):
    warn(
        "diffuse_texture is renamed to base_color_texture",
        DeprecationWarning,
        stacklevel=2,
    )
    self.get_base_color_texture()


RenderMaterial.set_diffuse_texture = set_diffuse_texture
RenderMaterial.get_diffuse_texture = get_diffuse_texture
RenderMaterial.diffuse_texture = property(get_diffuse_texture, set_diffuse_texture)
