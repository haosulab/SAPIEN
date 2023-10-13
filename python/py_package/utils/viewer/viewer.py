import os

import numpy as np
import sapien
from sapien import internal_renderer as R
from sapien.render import get_viewer_shader_dir
from sapien.render import SapienRenderer, RenderWindow, RenderSystem
from sapien import Scene, Entity

from .entity_window import EntityWindow

from .articulation_window import ArticulationWindow
from .control_window import ControlWindow
from .render_window import RenderOptionsWindow

from .imgui_ini import imgui_ini

# from .keyframe_window import KeyframeWindow
from .plugin import Plugin

from .scene_window import SceneWindow
from .transform_window import TransformWindow


class Viewer:
    def __init__(
        self,
        renderer: SapienRenderer = None,
        shader_dir="",
        resolutions=(1920, 1080),
        plugins=[
            ControlWindow(),
            SceneWindow(),
            EntityWindow(),
            ArticulationWindow(),
            TransformWindow(),
            RenderOptionsWindow()
            # KeyframeWindow(),
        ],
    ):
        if not os.path.exists("imgui.ini"):
            with open("imgui.ini", "w") as f:
                f.write(imgui_ini)

        if renderer is None:
            renderer = SapienRenderer()

        self.renderer = renderer

        self.renderer_context = renderer._internal_context

        if not shader_dir:
            self.shader_dir = get_viewer_shader_dir()
        else:
            self.shader_dir = shader_dir

        resolution = np.array(resolutions).flatten()[:2]

        self.scene = None
        self.system = None

        self.window = RenderWindow(*resolution, self.shader_dir)
        self.window.set_focus_callback(self.focus_change)
        self.window.set_drop_callback(self.drop)

        self.paused = False
        self.render_target = "Color"

        self.plugins = plugins
        self.init_plugins(plugins)

        self._selected_entity_visibility = 0.5

    def drop(self, files):
        if not self.scene:
            return

        builder = self.scene.create_actor_builder()
        for f in files:
            builder.add_visual_from_file(f)
        builder.build_kinematic("dropped file")

    def focus_change(self, focused):
        for plugin in self.plugins:
            plugin.notify_window_focus_change(focused)

    def init_plugins(self, plugins):
        for plugin in plugins:
            assert isinstance(plugin, Plugin)
            plugin.init(self)

    def set_scene(self, scene: Scene):
        if self.scene is not None:
            camera_pose = self.window.get_camera_pose()
            self.clear_scene()
        else:
            camera_pose = sapien.Pose([-2, 0, 0.5])

        self.selected_entity = None

        self.scene = scene
        self.system = scene.render_system
        self.window.set_scene(scene)

        self.window.set_camera_parameters(0.1, 1000, np.pi / 2)
        self.set_camera_pose(camera_pose)

        for plugin in self.plugins:
            plugin.notify_scene_change()

    def clear_scene(self):
        for plugin in self.plugins:
            plugin.clear_scene()

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
            if self.window.should_close:
                break

            if not self.paused or self.render_updated:
                self.system.step()
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
            for c in self.selected_entity.components:
                if isinstance(c, sapien.render.RenderBodyComponent):
                    c.visibility = 1

        self.selected_entity = entity

        # update selected entity
        if self.selected_entity is not None:
            for c in self.selected_entity.components:
                if isinstance(c, sapien.render.RenderBodyComponent):
                    c.visibility = self.selected_entity_visibility

        for plugin in self.plugins:
            plugin.notify_selected_entity_change()

    @property
    def selected_entity_visibility(self):
        return self._selected_entity_visibility

    @selected_entity_visibility.setter
    def selected_entity_visibility(self, v):
        self.notify_render_update()
        self._selected_entity_visibility = v

        if self.selected_entity is not None:
            for c in self.selected_entity.components:
                if isinstance(c, sapien.render.RenderBodyComponent):
                    c.visibility = self.selected_entity_visibility

    @property
    def resolution(self):
        return self.window.size

    @resolution.setter
    def resolution(self, res):
        self.window.resize(res[0], res[1])