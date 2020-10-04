from .pysapien import *
import pkg_resources
import os
import sys

GL_SHADER_ROOT = pkg_resources.resource_filename("sapien", "glsl_shader")
PTX_ROOT = pkg_resources.resource_filename("sapien", 'ptx')
__GL_VERSION_DICT = {3: "130", 4: "410"}
SPV_ROOT = pkg_resources.resource_filename("sapien", 'spv')


def __enable_ptx():
    assert os.path.exists(PTX_ROOT)
    OptifuserRenderer.set_optix_config(PTX_ROOT)


def __enable_gl(num: int):
    assert num in [3, 4]
    __GL_VERSION = num
    _GL_SHADER_PATH = os.path.join(GL_SHADER_ROOT, __GL_VERSION_DICT[__GL_VERSION])
    OptifuserRenderer.set_default_shader_config(_GL_SHADER_PATH, __GL_VERSION_DICT[__GL_VERSION])
    print("Using default glsl path {}".format(_GL_SHADER_PATH))


def __enable_vulkan():
    assert os.path.exists(SPV_ROOT)
    VulkanRenderer.set_shader_dir(SPV_ROOT)


def enable_default_gl3():
    __enable_gl(3)


def enable_default_gl4():
    __enable_gl(4)


if sys.platform.startswith("darwin"):
    enable_default_gl4()
else:
    enable_default_gl3()

try:
    __enable_ptx()
    sys.stderr.write('ray tracing enabled\n')
except:
    pass


try:
    __enable_vulkan()
    sys.stderr.write('Vulkan enabled\n')
except:
    pass
