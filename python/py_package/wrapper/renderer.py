from ..pysapien.render import SapienRenderer as _SapienRenderer
from ..pysapien.render import RenderMaterial


class SapienRenderer(_SapienRenderer):
    def create_material(self):
        return RenderMaterial()
