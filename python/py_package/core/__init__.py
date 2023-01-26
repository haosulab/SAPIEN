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


class VulkanRenderer(SapienRenderer):
    def __init__(self, *args, **kwargs):
        warn("VulkanRenderer is renamed SapienRenderer now")
        super().__init__(*args, **kwargs)


# it is okay to not have Kuafu
try:
    from .pysapien import KuafuRenderer as _KuafuRenderer

    class KuafuRenderer(_KuafuRenderer):
        def __init__(self, config: KuafuConfig):
            warn(
                """Kuafu renderer is deprecated in favor of `VulkanRenderer`, which supports both rasterization and ray tracing. To migrate,
    set `sapien.core.render_config.viewer_shader_dir` or `sapien.core.render_config.camera_shader_dir` to `"rt"`. Replace the following,
    kuafu_config.spp -> sapien.core.render_config.rt_samples_per_pixel
    kuafu_config.max_bounces -> sapien.core.render_config.rt_max_path_depth
    kuafu_config.use_denoiser -> sapien.core.render_config.rt_use_denoiser
    `kuafu_config.max_materials` and `kuafu_config.max_textures` are now input parameters to `VulkanRenderer`.
    There is currently no hard limit on max geometries.
    `kuafu_config.accumulate_frames` is no longer needed. Frames are automatically accumulated if scene.update_render is not called.
    In addition, `sapien.core.render_config` can be modified at any time and it takes effect for the cameras created after the modification.

                """
            )
            super().__init__(config)

    def __enable_kuafu():
        __KUAFU_ASSETS_ROOT = pkg_resources.resource_filename("sapien", "kuafu_assets")
        assert os.path.exists(__KUAFU_ASSETS_ROOT)
        _KuafuRenderer._set_default_assets_path(__KUAFU_ASSETS_ROOT)

    __enable_kuafu()

except ImportError:
    pass


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


def __enable_vulkan():
    __VULKAN_VIEWER_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/ibl"
    )
    __VULKAN_CAMERA_SHADER_ROOT = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/ibl"
    )
    assert os.path.exists(__VULKAN_VIEWER_SHADER_ROOT)
    assert os.path.exists(__VULKAN_CAMERA_SHADER_ROOT)
    RenderServer._set_shader_dir(__VULKAN_CAMERA_SHADER_ROOT)

    get_global_render_config().viewer_shader_dir = __VULKAN_VIEWER_SHADER_ROOT
    get_global_render_config().camera_shader_dir = __VULKAN_CAMERA_SHADER_ROOT

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
