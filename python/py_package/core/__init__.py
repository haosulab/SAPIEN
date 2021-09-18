from .pysapien import *
from .pysapien import renderer
import pkg_resources
import os
import sys


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
        "sapien", "vulkan_shader/ibl"
    )
    __VULKAN_CAMERA_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/ibl"
    )
    __KUAFU_ASSETS_ROOT = pkg_resources.resource_filename(
        "sapien", "kuafu_assets"
    )
    assert os.path.exists(__VULKAN_VIEWER_SHADER_ROOT)
    assert os.path.exists(__VULKAN_CAMERA_SHADER_ROOT)
    assert os.path.exists(__KUAFU_ASSETS_ROOT)
    VulkanRenderer._set_viewer_shader_dir(__VULKAN_VIEWER_SHADER_ROOT)
    VulkanRenderer._set_camera_shader_dir(__VULKAN_CAMERA_SHADER_ROOT)
    KuafuRenderer._set_default_assets_path(__KUAFU_ASSETS_ROOT)
    ensure_icd()


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
Entity.classname = property(lambda e: e.__class__.__name__)
