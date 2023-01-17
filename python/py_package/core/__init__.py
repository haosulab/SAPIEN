from warnings import warn

from .pysapien import *
from .pysapien import renderer
from .renderer_config import *

try:
    from .pysapien import dlpack
except ImportError:
    pass

import os
import sys
from typing import List

import pkg_resources


def ensure_icd():
    icd_filenames = os.environ.get("VK_ICD_FILENAMES")

    # if VK_ICD_FILENAMES is not provided, we try to provide it
    if not icd_filenames:
        icd_dir = "/usr/share/vulkan/icd.d"
        if os.path.isdir(icd_dir):
            files = os.listdir("/usr/share/vulkan/icd.d")
            os.environ["VK_ICD_FILENAMES"] = ":".join(
                [os.path.join(icd_dir, f) for f in files]
            )


# _global_render_config = get_global_render_config()


# def _set_viewer_shader_dir(shader_dir):
#     if os.path.exists(shader_dir):
#         _global_render_config.viewer_shader_dir = shader_dir
#         return
#     pkg_shader_dir = pkg_resources.resource_filename(
#         "sapien", "vulkan_shader/{}".format(shader_dir)
#     )
#     if os.path.exists(pkg_shader_dir):
#         _global_render_config.viewer_shader_dir = pkg_shader_dir
#         return
#     raise FileNotFoundError(shader_dir)


# def _set_camera_shader_dir(shader_dir):
#     if os.path.exists(shader_dir):
#         _global_render_config.camera_shader_dir = shader_dir
#         return
#     pkg_shader_dir = pkg_resources.resource_filename(
#         "sapien", "vulkan_shader/{}".format(shader_dir)
#     )
#     if os.path.exists(pkg_shader_dir):
#         _global_render_config.camera_shader_dir = pkg_shader_dir
#         return
#     raise FileNotFoundError(shader_dir)


# def _set_viewer_shader_dir_deprecated(shader_dir):
#     warn(
#         "This method is deprecated. Use context manager CameraShaderDir instead",
#         DeprecationWarning,
#         stacklevel=2,
#     )
#     _set_viewer_shader_dir(shader_dir)


# def _set_camera_shader_dir_deprecated(shader_dir):
#     warn(
#         "This method is deprecated. Use context manager ViewerShaderDir instead",
#         DeprecationWarning,
#         stacklevel=2,
#     )
#     _set_camera_shader_dir(shader_dir)


def __enable_vulkan():
    __VULKAN_VIEWER_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/ibl"
    )
    __VULKAN_CAMERA_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/ibl"
    )
    __KUAFU_ASSETS_ROOT = pkg_resources.resource_filename("sapien", "kuafu_assets")
    assert os.path.exists(__VULKAN_VIEWER_SHADER_ROOT)
    assert os.path.exists(__VULKAN_CAMERA_SHADER_ROOT)
    assert os.path.exists(__KUAFU_ASSETS_ROOT)
    RenderServer._set_shader_dir(__VULKAN_CAMERA_SHADER_ROOT)

    get_global_render_config().viewer_shader_dir = __VULKAN_VIEWER_SHADER_ROOT
    get_global_render_config().camera_shader_dir = __VULKAN_CAMERA_SHADER_ROOT
    # VulkanRenderer._set_viewer_shader_dir(__VULKAN_VIEWER_SHADER_ROOT)
    # VulkanRenderer._set_camera_shader_dir(__VULKAN_CAMERA_SHADER_ROOT)

    KuafuRenderer._set_default_assets_path(__KUAFU_ASSETS_ROOT)
    ensure_icd()


__enable_vulkan()


Entity.classname = property(lambda e: e.__class__.__name__)


def _auto_allocate_torch_tensors(self: RenderServer, render_targets: List[str]):
    import torch

    buffers = self.auto_allocate_buffers(render_targets)
    tensors = [torch.as_tensor(x, device="cuda") for x in buffers]

    for b, t in zip(buffers, tensors):
        assert b.__cuda_array_interface__["data"][0] == t.data_ptr()

    return tensors


RenderServer.auto_allocate_torch_tensors = _auto_allocate_torch_tensors
