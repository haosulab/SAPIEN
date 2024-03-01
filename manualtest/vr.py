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
from sapien.render import RenderVRDisplay

sapien.render.set_log_level("info")


class VRViewer:
    def __init__(self):
        self.vr = RenderVRDisplay()
        self.controllers = self.vr.get_controller_ids()
        self.renderer_context = sapien.render.SapienRenderer()._internal_context
        self._create_visual_models()

        self.reset()

    def reset(self):
        self.controller_axes = None

    @property
    def root_pose(self):
        return self.vr.root_pose

    @root_pose.setter
    def root_pose(self, pose):
        self.vr.root_pose = pose

    @property
    def render_scene(self):
        return self.vr._internal_scene

    @property
    def controller_poses(self):
        return [self.vr.get_controller_pose(c) for c in self.controllers]

    def set_scene(self, scene):
        self.scene = scene
        self.vr.set_scene(scene)

    def render(self):
        self._update_controller_axes()
        self.vr.update_render()
        self.vr.render()

    # helper visuals
    def _create_visual_models(self):
        self.cone = self.renderer_context.create_cone_mesh(16)
        self.capsule = self.renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)
        self.mat_red = self.renderer_context.create_material(
            [1, 0, 0, 1], [0, 0, 0, 1], 0, 1, 0
        )
        self.mat_green = self.renderer_context.create_material(
            [0, 1, 0, 1], [0, 0, 0, 1], 0, 1, 0
        )
        self.mat_blue = self.renderer_context.create_material(
            [0, 0, 1, 1], [0, 0, 0, 1], 0, 1, 0
        )
        self.mat_cyan = self.renderer_context.create_material(
            [0, 1, 1, 1], [0, 0, 0, 1], 0, 1, 0
        )
        self.mat_magenta = self.renderer_context.create_material(
            [1, 0, 1, 1], [0, 0, 0, 1], 0, 1, 0
        )
        self.red_cone = self.renderer_context.create_model([self.cone], [self.mat_red])
        self.green_cone = self.renderer_context.create_model(
            [self.cone], [self.mat_green]
        )
        self.blue_cone = self.renderer_context.create_model(
            [self.cone], [self.mat_blue]
        )
        self.red_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_red]
        )
        self.green_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_green]
        )
        self.blue_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_blue]
        )
        self.cyan_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_cyan]
        )
        self.magenta_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_magenta]
        )

    def _create_coordiate_axes(self):
        render_scene = self.render_scene

        node = render_scene.add_node()
        obj = render_scene.add_object(self.red_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([1, 0, 0])
        obj.shading_mode = 0
        obj.cast_shadow = False

        obj = render_scene.add_object(self.red_capsule, node)
        obj.set_position([0.52, 0, 0])
        obj.shading_mode = 0
        obj.cast_shadow = False

        obj = render_scene.add_object(self.green_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 1, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False

        obj = render_scene.add_object(self.green_capsule, node)
        obj.set_position([0, 0.51, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False

        obj = render_scene.add_object(self.blue_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 0, 1])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False

        obj = render_scene.add_object(self.blue_capsule, node)
        obj.set_position([0, 0, 0.5])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False

        node.set_scale([0.1, 0.1, 0.1])

        return node

    def _update_controller_axes(self):
        if self.controller_axes is None:
            self.controller_axes = [
                self._create_coordiate_axes() for c in self.controllers
            ]

        for n, pose in zip(self.controller_axes, self.controller_poses):
            c2w = self.vr.root_pose * pose
            n.set_position(c2w.p)
            n.set_rotation(c2w.q)


def run():
    scene = sapien.Scene()
    scene.load_widget_from_package("demo_arena", "DemoArena")

    viewer = VRViewer()
    viewer.set_scene(scene)

    viewer.root_pose = sapien.Pose([1, 0, 0], [0, 0, 0, 1])

    while True:
        scene.step()
        viewer.render()
        for c in viewer.controllers:
            print(f"button {viewer.vr.get_controller_button_state(c):x}")
            print(f"touch {viewer.vr.get_controller_touch_state(c):x}")
            print(f"axis 0 {viewer.vr.get_controller_axis_state(c, 0)}")
            print(f"axis 1 {viewer.vr.get_controller_axis_state(c, 1)}")
            print(f"axis 2 {viewer.vr.get_controller_axis_state(c, 2)}")
            break


def main():
    try:
        run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
