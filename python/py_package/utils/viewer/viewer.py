import os
from pathlib import Path

import numpy as np
import sapien
from sapien import Entity, Scene
from sapien import internal_renderer as R
from sapien.render import (
    RenderSystem,
    RenderWindow,
    SapienRenderer,
    get_viewer_shader_dir,
)

from .articulation_window import ArticulationWindow
from .contact_window import ContactWindow
from .control_window import ControlWindow
from .entity_window import EntityWindow
from .imgui_ini import imgui_ini
from .path_window import PathWindow
from .plugin import Plugin
from .render_window import RenderOptionsWindow
from .scene_window import SceneWindow
from .setting_window import SettingWindow
from .transform_window import TransformWindow

# from .keyframe_window import KeyframeWindow


class Viewer:
    def __init__(
        self,
        renderer: SapienRenderer = None,
        shader_dir="",
        resolutions=(1920, 1080),
        plugins=[
            PathWindow(),
            ContactWindow(),
            SettingWindow(),
            TransformWindow(),
            RenderOptionsWindow(),
            ControlWindow(),
            SceneWindow(),
            EntityWindow(),
            ArticulationWindow(),
            # KeyframeWindow(),
        ],
    ):
        filename = sapien.render.get_imgui_ini_filename()
        if not filename:
            filename = "imgui.ini"

        if not os.path.exists(filename):
            Path(filename).parent.mkdir(parents=True, exist_ok=True)

            with open(filename, "w") as f:
                f.write(imgui_ini)

        if renderer is None:
            renderer = SapienRenderer()

        self.renderer_context = renderer._internal_context

        if not shader_dir:
            self.shader_dir = get_viewer_shader_dir()
        else:
            self.shader_dir = shader_dir

        resolution = np.array(resolutions).flatten()[:2]

        self.scenes = []

        self.window = RenderWindow(*resolution, self.shader_dir)
        self.window.set_focus_callback(self.focus_change)
        self.window.set_drop_callback(self.drop)

        self.paused = False
        self.render_target = "Color"

        self.plugins = plugins
        self.init_plugins(plugins)

        self._selected_entity_visibility = 0.5

    @property
    def render_scene(self):
        return self.window._internal_scene

    @property
    def scene(self) -> sapien.Scene:
        if len(self.scenes) == 1:
            return self.scenes[0]
        return None

    @property
    def cameras(self):
        if self.scene:
            return self.scene.render_system.cameras
        return []

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

    def set_scenes(self, scenes, offsets=None):
        if offsets is None:
            side = int(np.ceil(len(scenes) ** 0.5))
            idx = np.arange(len(scenes))
            offsets = np.stack([idx // side, idx % side, np.zeros_like(idx)], axis=1)

        if self.scenes:
            camera_pose = self.window.get_camera_pose()
            self.clear_scene()
        else:
            camera_pose = sapien.Pose([-2, 0, 0.5])

        self.selected_entity = None
        self.scenes = scenes

        if len(scenes) == 0:
            self.window.set_scene(None)
        elif len(scenes) == 1:
            self.window.set_scene(scenes[0])
            self.scene_offset = {scenes[0]: np.array([0, 0, 0])}
        else:
            self.window.set_scenes(scenes, offsets)
            self.scene_offset = dict((s, o) for s, o in zip(scenes, offsets))

        self.window.set_camera_parameters(0.1, 1000, np.pi / 2)
        self.set_camera_pose(camera_pose)

        for plugin in self.plugins:
            plugin.notify_scene_change()

    def get_entity_viewer_pose(self, entity):
        return sapien.Pose(self.scene_offset[entity.scene]) * entity.pose

    def set_scene(self, scene: Scene):
        self.set_scenes([scene])
        # if self.scene is not None:
        #     camera_pose = self.window.get_camera_pose()
        #     self.clear_scene()
        # else:
        #     camera_pose = sapien.Pose([-2, 0, 0.5])

        # self.selected_entity = None

        # self.scene = scene
        # # self.system = self.scene.render_system
        # self.window.set_scene(scene)

        # self.window.set_camera_parameters(0.1, 1000, np.pi / 2)
        # self.set_camera_pose(camera_pose)

        # for plugin in self.plugins:
        #     plugin.notify_scene_change()

    def clear_scene(self):
        for plugin in self.plugins:
            plugin.clear_scene()

    @property
    def closed(self):
        return self.window is None

    def close(self):
        for plugin in self.plugins:
            plugin.close()

        self.selected_entity = None
        self.scenes = []
        # self.system = None
        self.window = None
        self.plugins = []
        self.renderer_context = None

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
            self.close()

        if self.closed:
            return

        while True:
            if self.window.should_close:
                break

            if not self.paused or self.render_updated:
                self.window.update_render()
                # self.system.step()
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

        self.render_scene.force_rebuild()

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

    def add_bounding_box(self, pose, half_size, color):
        vertices = np.array(
            [
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, -1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, 1, 1],
                [-1, -1, 1],
            ]
        )
        lines = [0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]
        vertices = vertices[lines]
        colors = np.ones((vertices.shape[0], 4)) * [*color[:3], 1]
        lineset = self.renderer_context.create_line_set(vertices, colors)

        # render_scene: R.Scene = self.system._internal_scene
        box = self.render_scene.add_line_set(lineset)
        box.set_position(pose.p)
        box.set_rotation(pose.q)
        box.set_scale(half_size)

        return box

    def update_bounding_box(self, box, pose, half_size):
        box.set_position(pose.p)
        box.set_rotation(pose.q)
        box.set_scale(half_size)

    def remove_bounding_box(self, box):
        # render_scene: R.Scene = self.system._internal_scene
        self.render_scene.remove_node(box)

    def draw_aabb(self, lower, upper, color):
        pose = sapien.Pose((lower + upper) / 2)
        half_size = (upper - lower) / 2
        return self.add_bounding_box(pose, half_size, color)

    def update_aabb(self, aabb, lower, upper):
        pose = sapien.Pose((lower + upper) / 2)
        half_size = (upper - lower) / 2
        self.update_bounding_box(aabb, pose, half_size)

    @property
    def control_window(self) -> ControlWindow:
        for plugin in self.plugins:
            if isinstance(plugin, ControlWindow):
                return plugin
        return None

    def loop(self, physx_steps=0):
        """
        A convenience method for opening a temporary viewer for a scene.
        Simply call scene.create_viewer().loop()
        """
        while not self.closed:
            for _ in range(physx_steps):
                self.scene.physx_system.step()
            self.scene.update_render()
            self.render()

    def register_click_handler(self, handler):
        ...

    def set_camera_xyz(self, x, y, z):
        ...

    def set_camera_rpy(self, r, p, y):
        ...

    def focus_entity(self, entity):
        ...

    def focus_camera(self, camera):
        ...
