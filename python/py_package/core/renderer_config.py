import os
from typing import Union

import pkg_resources
from warnings import warn

from .pysapien import SapienRenderer, get_global_render_config

_global_render_config = get_global_render_config()


def _set_viewer_shader_dir(shader_dir):
    if os.path.exists(shader_dir):
        _global_render_config.viewer_shader_dir = shader_dir
        return
    pkg_shader_dir = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/{}".format(shader_dir)
    )
    if os.path.exists(pkg_shader_dir):
        _global_render_config.viewer_shader_dir = pkg_shader_dir
        return
    raise FileNotFoundError(shader_dir)


def _set_camera_shader_dir(shader_dir):
    if os.path.exists(shader_dir):
        _global_render_config.camera_shader_dir = shader_dir
        return
    pkg_shader_dir = pkg_resources.resource_filename(
        "sapien", "vulkan_shader/{}".format(shader_dir)
    )
    if os.path.exists(pkg_shader_dir):
        _global_render_config.camera_shader_dir = pkg_shader_dir
        return
    raise FileNotFoundError(shader_dir)


def _set_viewer_shader_dir_deprecated(shader_dir):
    warn(
        "This method is deprecated. Use sapien.core.render_config.viewer_shader_dir instead",
        DeprecationWarning,
        stacklevel=2,
    )
    _set_viewer_shader_dir(shader_dir)


def _set_camera_shader_dir_deprecated(shader_dir):
    warn(
        "This method is deprecated. Use sapien.core.render_config.camera_shader_dir instead",
        DeprecationWarning,
        stacklevel=2,
    )
    _set_camera_shader_dir(shader_dir)


SapienRenderer.set_viewer_shader_dir = _set_viewer_shader_dir_deprecated
SapienRenderer.set_camera_shader_dir = _set_camera_shader_dir_deprecated


class _RenderConfig:
    def __init__(self):
        self.config = get_global_render_config()

    @property
    def camera_shader_dir(self):
        return self.config.camera_shader_dir

    @camera_shader_dir.setter
    def camera_shader_dir(self, dirname: str):
        _set_camera_shader_dir(dirname)

    @property
    def viewer_shader_dir(self):
        return self.config.viewer_shader_dir

    @viewer_shader_dir.setter
    def viewer_shader_dir(self, dirname: str):
        _set_viewer_shader_dir(dirname)

    @property
    def rt_samples_per_pixel(self):
        return self.config.rt_samples_per_pixel

    @rt_samples_per_pixel.setter
    def rt_samples_per_pixel(self, value: int):
        self.config.rt_samples_per_pixel = value

    @property
    def rt_max_path_depth(self):
        return self.config.rt_path_depth

    @rt_max_path_depth.setter
    def rt_max_path_depth(self, depth: int):
        self.config.rt_path_depth = depth

    @property
    def rt_use_denoiser(self):
        return self.config.rt_use_denoiser

    @rt_use_denoiser.setter
    def rt_use_denoiser(self, enable: bool):
        self.config.rt_use_denoiser = enable

    def get_render_target_format(self, name: str) -> Union[str, None]:
        if not self.config.has_render_target_format(name):
            return None
        return self.config.get_render_target_format(name)

    def set_render_target_format(self, name: str, format: str):
        self.config.set_render_target_format(name, format)

    def unset_render_target_format(self, name: str):
        self.config.unset_render_target_format(name)


render_config = _RenderConfig()
