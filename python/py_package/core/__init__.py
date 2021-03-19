from .pysapien import *
from .pysapien import renderer
import pkg_resources
import os
import sys

GL_SHADER_ROOT = pkg_resources.resource_filename("sapien", "glsl_shader")
PTX_ROOT = pkg_resources.resource_filename("sapien", "ptx")
__GL_VERSION_DICT = {3: "130", 4: "410"}
VULKAN_SHADER_ROOT = pkg_resources.resource_filename("sapien", "vulkan_shader/full")
VULKAN_ICD_ROOT = pkg_resources.resource_filename("sapien", "vulkan_icd")


def __enable_ptx():
    assert os.path.exists(PTX_ROOT)
    OptifuserRenderer.set_optix_config(PTX_ROOT)


def __enable_gl(num: int):
    assert num in [3, 4]
    __GL_VERSION = num
    _GL_SHADER_PATH = os.path.join(GL_SHADER_ROOT, __GL_VERSION_DICT[__GL_VERSION])
    OptifuserRenderer.set_default_shader_config(
        _GL_SHADER_PATH, __GL_VERSION_DICT[__GL_VERSION]
    )


def ensure_icd():
    icd_filenames = os.environ.get("VK_ICD_FILENAMES")
    if icd_filenames is None:
        icd_filenames = ""
    icd_filenames = "{0}/nvidia_icd.json:{0}/radeon_icd.x86_64.json:{0}/intel_icd.x86_64.json:{1}".format(
        VULKAN_ICD_ROOT, icd_filenames
    )
    os.environ["VK_ICD_FILENAMES"] = icd_filenames


def __enable_vulkan():
    assert os.path.exists(VULKAN_SHADER_ROOT)
    VulkanRenderer.set_shader_dir(VULKAN_SHADER_ROOT)
    ensure_icd()


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
except:
    pass


__enable_vulkan()
