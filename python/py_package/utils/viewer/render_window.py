from pathlib import Path

import numpy as np
import pkg_resources
import sapien
from sapien import internal_renderer as R

from .plugin import Plugin


class RenderOptionsWindow(Plugin):
    def __init__(self):
        self.reset()

    def init(self, viewer):
        super().init(viewer)
        self._setup_shader_dir()

    @property
    def window(self):
        return self.viewer.window

    @property
    def shader_name_list(self):
        return [d.name for d in self.shader_list]

    def set_shader(self, index):
        self.shader_dir = str(self.shader_list[index])
        self.shader_type = str(self.shader_types[index])
        self.window.set_shader_dir(self.shader_dir)
        self.viewer.render_target = "Color"
        self.shader_index = index
        print(self.shader_index)

    def reset(self):
        self.ui_window = None
        self.shader_dir = None
        self.shader_index = None
        self.shader_list = None
        self.shader_type = None
        self.shader_types = None

    @property
    def is_rt(self):
        return getattr(self, "shader_type", None) == "rt"

    @property
    def denoiser_index(self):
        if self.viewer.window.denoiser == "oidn":
            return 1
        if self.viewer.window.denoiser == "optix":
            return 2
        return 0

    @denoiser_index.setter
    def denoiser_index(self, index):
        if index == 1:
            self.viewer.window.denoiser = "oidn"
        elif index == 2:
            self.viewer.window.denoiser = "optix"
        else:
            self.viewer.window.denoiser = "none"

    @property
    def spp(self):
        return self.window.get_camera_property_int("spp")

    @spp.setter
    def spp(self, v):
        if v < 1:
            v = 1
        if v > 1024:
            v = 1024
            print("spp seems too large")

        return self.window.set_camera_property("spp", int(v))

    @property
    def ray_depth(self):
        return self.window.get_camera_property_int("maxDepth")

    @ray_depth.setter
    def ray_depth(self, v):
        if v < 1:
            v = 1
        if v > 16:
            v = 16
            print("ray depth seems too large")

        return self.window.set_camera_property("maxDepth", int(v))

    @property
    def aperture(self):
        return self.window.get_camera_property_float("aperture")

    @aperture.setter
    def aperture(self, v):
        return self.window.set_camera_property("aperture", float(v))

    @property
    def focal_plane(self):
        return self.window.get_camera_property_float("focusPlane")

    @focal_plane.setter
    def focal_plane(self, v):
        return self.window.set_camera_property("focusPlane", float(v))

    @property
    def exposure(self):
        return self.window.get_camera_property_float("exposure")

    @exposure.setter
    def exposure(self, v):
        return self.window.set_camera_property("exposure", float(v))

    @property
    def tone_index(self):
        try:
            return self.viewer.window.get_camera_property_int("toneMapper")
        except Exception:
            self.viewer.window.set_camera_property("toneMapper", 0)

        return self.viewer.window.get_camera_property_int("toneMapper")

    @tone_index.setter
    def tone_index(self, index):
        index = min(max(0, index), 2)
        self.viewer.window.set_camera_property("toneMapper", index)

    def _setup_shader_dir(self):
        default_shader_dir = Path(self.viewer.shader_dir)

        default_type = "rast"
        for x in default_shader_dir.iterdir():
            if str(x).endswith("camera.rgen"):
                default_type = "rt"
                break

        self.shader_list = []
        self.shader_types = []

        try:
            all_shader_dir = Path(
                pkg_resources.resource_filename("sapien", "vulkan_shader")
            )

            for f in all_shader_dir.iterdir():
                if f.is_dir():
                    if any("gbuffer.frag" in x.name for x in f.iterdir()):
                        self.shader_list.append(f)
                        self.shader_types.append("rast")
                    if any("camera.rgen" in x.name for x in f.iterdir()):
                        self.shader_list.append(f)
                        self.shader_types.append("rt")

        except Exception:
            pass

        self.shader_type = default_type
        self.shader_dir = str(default_shader_dir)
        if default_shader_dir not in self.shader_list:
            self.shader_list = [default_shader_dir] + self.shader_list
            self.shader_types = [default_type] + self.shader_types

        self.shader_index = self.shader_list.index(default_shader_dir)

    def build(self):
        if self.viewer.render_scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.ui_window = R.UIWindow().Label("Render").Pos(10, 10).Size(400, 400)

            self.ui_window.append(
                R.UIOptions()
                .Label("Shader")
                .Style("select")
                .BindItems(self, "shader_name_list")
                .BindIndex(self, "shader_index")
                .Callback(lambda p: self.set_shader(p.index)),
                R.UIConditional()
                .Bind(self, "is_rt")
                .append(
                    R.UIOptions()
                    .Style("select")
                    .Label("Denoiser")
                    .Items(["none", "oidn", "optix"])
                    .BindIndex(self, "denoiser_index"),
                    R.UIInputInt().Label("spp").Bind(self, "spp"),
                    R.UIInputInt().Label("Ray Depth").Bind(self, "ray_depth"),
                    R.UISliderFloat()
                    .Label("Focal Plane")
                    .Min(0.01)
                    .Max(10)
                    .Bind(self, "focal_plane"),
                    R.UISliderFloat()
                    .Label("Aperture")
                    .Min(0)
                    .Max(0.1)
                    .Bind(self, "aperture"),
                    R.UISliderFloat()
                    .Label("Exposure")
                    .Min(0)
                    .Max(30)
                    .Bind(self, "exposure"),
                    R.UIOptions()
                    .Style("select")
                    .Label("Color Management")
                    .Items(["Gamma", "sRGB", "Filmic"])
                    .BindIndex(self, "tone_index"),
                ),
            )

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []

    def close(self):
        self.reset()
