import os

import numpy as np
import sapien.core as sapien
from sapien.core import (
    ActorBase,
    ArticulationBase,
    CameraEntity,
    DirectionalLightEntity,
    Entity,
    Joint,
    LightEntity,
    LinkBase,
    PointLightEntity,
    Pose,
    SapienRenderer,
    Scene,
    SpotLightEntity,
    VulkanWindow,
    render_config,
)
from sapien.core import renderer as R

from .actor_window import ActorWindow
from .articulation_window import ArticulationWindow
from .control_window import ControlWindow
from .imgui_ini import imgui_ini
from .keyframe_window import KeyframeWindow
from .plugin import Plugin
from .scene_window import SceneWindow
from .transform_window import TransformWindow


class Viewer:
    def __init__(
        self,
        renderer: SapienRenderer,
        shader_dir="",
        resolutions=(1920, 1080),
        plugins=[
            ControlWindow(),
            SceneWindow(),
            ActorWindow(),
            ArticulationWindow(),
            TransformWindow(),
            KeyframeWindow(),
        ],
    ):
        if not os.path.exists("imgui.ini"):
            with open("imgui.ini", "w") as f:
                f.write(imgui_ini)

        self.renderer = renderer
        self.renderer_context = renderer._internal_context
        if not shader_dir:
            self.shader_dir = render_config.viewer_shader_dir
        resolution = np.array(resolutions).flatten()[:2]

        self.scene = None
        self.window = self.renderer.create_window(*resolution, self.shader_dir)
        self.window.set_focus_callback(self.focus_change)
        self.paused = False
        self.render_target = "Color"

        self.plugins = plugins
        self.init_plugins(plugins)

        self._selected_entity_visibility = 0.5

    def focus_change(self, focused):
        for plugin in self.plugins:
            plugin.notify_window_focus_change(focused)

    def init_plugins(self, plugins):
        for plugin in plugins:
            assert isinstance(plugin, Plugin)
            plugin.init(self)

    def set_scene(self, scene: Scene):
        if self.scene is not None:
            self.clear_scene()

        self.selected_entity = None

        self.scene = scene
        self.window.set_scene(scene)

        for plugin in self.plugins:
            plugin.notify_scene_change()

    def clear_scene(self):
        for plugin in self.plugins:
            plugin.clear_scene()
        pass

    @property
    def closed(self):
        return self.window is None

    def close(self):
        self.window = None

    def set_camera_pose(self, pose):
        self.window.set_camera_pose(pose)
        self.notify_render_update()

    def notify_render_update(self):
        """notify the viewer that the camera is moved"""
        self.render_updated = True

    def reset_notifications(self):
        self.render_updated = False

    def render(self):
        if self.window.should_close:
            for plugin in self.plugins:
                plugin.close()
            self.close()

        if self.closed:
            return

        while True:
            if not self.paused or self.render_updated:
                self.scene.update_render()
            self.reset_notifications()

            for plugin in self.plugins:
                plugin.before_render()

            ui_windows = []
            for plugin in self.plugins:
                ui_windows += plugin.get_ui_windows()

            self.window.render(self.render_target, ui_windows)

            for plugin in self.plugins:
                plugin.after_render()

            if not self.paused:
                break

    def select_entity(self, entity: Entity):
        if self.selected_entity == entity:
            return

        # reset previous selected entity
        if self.selected_entity is not None:
            if isinstance(self.selected_entity, sapien.ActorBase):
                self.selected_entity.set_visibility(1)

        self.selected_entity = entity

        # update selected entity
        if self.selected_entity is not None:
            if isinstance(self.selected_entity, sapien.ActorBase):
                self.selected_entity.set_visibility(self.selected_entity_visibility)

        for plugin in self.plugins:
            plugin.notify_selected_entity_change()

    @property
    def selected_entity_visibility(self):
        return self._selected_entity_visibility

    @selected_entity_visibility.setter
    def selected_entity_visibility(self, v):
        self.notify_render_update()
        self._selected_entity_visibility = v

        # update selected entity
        if self.selected_entity is not None:
            if isinstance(self.selected_entity, sapien.ActorBase):
                self.selected_entity.set_visibility(self.selected_entity_visibility)

    @property
    def resolution(self):
        return self.window.size

    @resolution.setter
    def resolution(self, res):
        self.window.resize(res[0], res[1])
