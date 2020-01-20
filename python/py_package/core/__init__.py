from .pysapien import *
import pkg_resources
import os

GL_SHADER_ROOT = pkg_resources.resource_filename("sapien", "glsl_shader")
__GL_VERSION_DICT = {3: "130", 4: "450"}


def __enable_gl(num: int):
    assert num in [3, 4]
    __GL_VERSION = num
    _GL_SHADER_PATH = os.path.join(GL_SHADER_ROOT, __GL_VERSION_DICT[__GL_VERSION])
    OptifuserRenderer.set_default_shader_config(_GL_SHADER_PATH, __GL_VERSION_DICT[__GL_VERSION])


def enable_default_3():
    __enable_gl(3)


def enable_default_gl4():
    __enable_gl(4)


enable_default_3()
