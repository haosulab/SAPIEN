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
