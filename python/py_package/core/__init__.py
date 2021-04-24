from .pysapien import *
from .pysapien import renderer
import pkg_resources
import os
import sys


def __enable_ptx():
    __PTX_ROOT = pkg_resources.resource_filename("sapien", "ptx")
    assert os.path.exists(__PTX_ROOT)
    OptifuserRenderer.set_optix_config(__PTX_ROOT)


def __enable_gl(num: int):
    assert num in [3, 4]
    __GL_VERSION = num
    __GL_SHADER_ROOT = pkg_resources.resource_filename("sapien", "glsl_shader")
    _GL_SHADER_PATH = os.path.join(__GL_SHADER_ROOT, "130")
    OptifuserRenderer.set_default_shader_config(_GL_SHADER_PATH, "130")


def ensure_icd():
    __VULKAN_ICD_ROOT = pkg_resources.resource_filename("sapien", "vulkan_icd")
    icd_filenames = os.environ.get("VK_ICD_FILENAMES")

    # if VK_ICD_FILENAMES is not provided, we try to provide it
    if not icd_filenames:
        icd_filenames = "{0}/intel_icd.x86_64.json:{0}/nvidia_icd.json:{0}/radeon_icd.x86_64.json:{0}/MoltenVK_icd.json:{1}".format(
            __VULKAN_ICD_ROOT, icd_filenames
        )
        os.environ["VK_ICD_FILENAMES"] = icd_filenames


def __enable_vulkan():
    __VULKAN_VIEWER_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/default_viewer"
    )
    __VULKAN_CAMERA_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/default_camera"
    )
    assert os.path.exists(__VULKAN_VIEWER_SHADER_ROOT)
    assert os.path.exists(__VULKAN_CAMERA_SHADER_ROOT)
    VulkanRenderer._set_viewer_shader_dir(__VULKAN_VIEWER_SHADER_ROOT)
    VulkanRenderer._set_camera_shader_dir(__VULKAN_CAMERA_SHADER_ROOT)
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


def __set_viewer_shader_dir(shader_dir):
    if os.path.exists(shader_dir):
        VulkanRenderer._set_viewer_shader_dir(shader_dir)
        return
    shader_dir = pkg_resources.resource_filename("sapien", "vulkan_shader/{}".format(shader_dir))
    if os.path.exists(shader_dir):
        VulkanRenderer._set_viewer_shader_dir(shader_dir)
        return
    raise FileNotFoundError(shader_dir)


def __set_camera_shader_dir(shader_dir):
    if os.path.exists(shader_dir):
        VulkanRenderer._set_camera_shader_dir(shader_dir)
        return
    shader_dir = pkg_resources.resource_filename("sapien", "vulkan_shader/{}".format(shader_dir))
    if os.path.exists(shader_dir):
        VulkanRenderer._set_camera_shader_dir(shader_dir)
        return
    raise FileNotFoundError(shader_dir)


VulkanRenderer.set_viewer_shader_dir = __set_viewer_shader_dir
VulkanRenderer.set_camera_shader_dir = __set_camera_shader_dir
