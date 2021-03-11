import sapien.core.pysapien.renderer as R
from sapien.core import Pose, VulkanRenderer, Scene, VulkanWindow
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector
import numpy as np


class FPSCameraController:
    def __init__(self, window: VulkanWindow):
        self.window = window
        self.forward = np.array([1, 0, 0])
        self.up = np.array([0, 0, 1])
        self.left = np.cross(self.up, self.forward)
        self.initial_rotation = mat2quat(
            np.array([-self.left, self.up, -self.forward]).T
        )
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)

    def setRPY(self, roll, pitch, yaw):
        self.rpy = np.array([roll, pitch, yaw])
        self.update()

    def setXYZ(self, x, y, z):
        self.xyz = np.array([x, y, z])
        self.update()

    def move(self, forward, left, up):
        q = qmult(
            qmult(aa(self.up, self.rpy[2]), aa(self.left, -self.rpy[1])),
            aa(self.forward, self.rpy[0]),
        )
        self.xyz = self.xyz + (
            rotate_vector(self.forward, q) * forward
            + rotate_vector(self.left, q) * left
            + rotate_vector(self.up, q) * up
        )
        self.update()

    def rotate(self, roll, pitch, yaw):
        self.rpy = self.rpy + np.array([roll, pitch, yaw])
        self.update()

    def update(self):
        self.rpy[1] = np.clip(self.rpy[1], -1.57, 1.57)
        if self.rpy[2] >= 3.15:
            self.rpy[2] = self.rpy[2] - 2 * np.pi
        elif self.rpy[2] <= -3.15:
            self.rpy[2] = self.rpy[2] + 2 * np.pi

        rot = qmult(
            qmult(
                qmult(aa(self.up, self.rpy[2]), aa(self.left, -self.rpy[1])),
                aa(self.forward, self.rpy[0]),
            ),
            self.initial_rotation,
        )
        self.window.set_camera_rotation(rot)
        self.window.set_camera_position(self.xyz)


class RenderController(object):
    def __init__(self, renderer: VulkanRenderer, shader_dir=""):
        self.renderer = renderer
        self.renderer_context: R.Context = renderer._internal_context
        self.window = renderer.create_window(shader_dir)

        self.cone = self.renderer_context.create_cone_mesh(16)
        self.capsule = self.renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)
        self.mat_red = self.renderer_context.create_material([1, 0, 0, 1], 0, 0, 0)
        self.mat_green = self.renderer_context.create_material([0, 1, 0, 1], 0, 0, 0)
        self.mat_blue = self.renderer_context.create_material([0, 0, 1, 1], 0, 0, 0)
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

        R.UISection()
        self.ui1 = (
            R.UIWindow()
            .Label("Main")
            .Pos(10, 10)
            .Size(200, 400)
            .append(
                R.UISection()
                .Label("Control")
                .Expanded(True)
                .append(
                    R.UICheckbox()
                    .Label("Pause")
                    .Callback(lambda p: self.toggle_pause(p.checked)),
                    R.UIButton().Label("Step").Callback(lambda p: self.step_button()),
                    R.UICheckbox()
                    .Label("Axes")
                    .Checked(True)
                    .Callback(lambda p: self.toggle_axes(p.checked)),
                ),
                R.UISection()
                .Label("Display")
                .append(
                    R.UIRadioButtonGroup()
                    .Labels(
                        ["Color"]
                        + [x for x in self.window.target_names if x != "Color"]
                    )
                    .Callback(lambda p: self.set_target(p.value)),
                ),
            )
        )
        self.axes = None
        self.selected_actor = None
        self.paused = False
        self.target_name = "Color"
        self.single_step = False

    def step_button(self):
        if self.paused:
            self.single_step = True

    def create_axes(self):
        assert self.scene is not None
        rs = self.scene.get_render_scene()
        render_scene: R.Scene = rs._internal_scene

        node = render_scene.add_node()
        obj = render_scene.add_object(self.red_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([1, 0, 0])
        obj.shading_mode = 2
        obj = render_scene.add_object(self.red_capsule, node)
        obj.set_position([0.5, 0, 0])
        obj.set_scale([1.02, 1.02, 1.02])
        obj.shading_mode = 2

        obj = render_scene.add_object(self.green_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 1, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 2
        obj = render_scene.add_object(self.green_capsule, node)
        obj.set_position([0, 0.5, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 2

        obj = render_scene.add_object(self.blue_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 0, 1])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 2
        obj = render_scene.add_object(self.blue_capsule, node)
        obj.set_position([0, 0, 0.5])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 2

        return node

    def set_scene(self, scene: Scene):
        self.axes = None
        self.scene = scene
        self.window.set_scene(scene)
        self.camera_controller = FPSCameraController(self.window)
        self.toggle_axes(True)

    def set_camera_xyz(self, x, y, z):
        self.camera_controller.setXYZ(x, y, z)

    def set_camera_rpy(self, r, p, y):
        self.camera_controller.setRPY(r, p, y)

    def toggle_pause(self, paused):
        self.paused = paused

    def toggle_axes(self, show):
        if show:
            self.axes = self.create_axes()
            self.axes.set_scale([0.3, 0.3, 0.3])
            if self.selected_actor:
                self.axes.set_position(self.selected_actor.pose.p)
                self.axes.set_rotation(self.selected_actor.pose.q)
            else:
                self.axes.set_position([0, 0, 0])
                self.axes.set_rotation([0, 0, 0, 1])
        elif self.scene:
            rs = self.scene.get_render_scene()
            render_scene: R.Scene = rs._internal_scene
            render_scene.remove_node(self.axes)
            self.axes = None

    def set_target(self, name):
        self.target_name = name

    def find_actor(self, id):
        actors = self.scene.get_all_actors()
        for actor in actors:
            if actor.id == id:
                return actor
        for a in self.scene.get_all_articulations():
            for link in a.get_links():
                if link.id == id:
                    return link

    @property
    def closed(self):
        return self.window is None

    def close(self):
        self.axes = None
        self.scene = None
        self.camera_controller = None
        self.window = None

    def render(self):
        if self.closed:
            return

        while True:
            self.window.render(self.target_name, [self.ui1])
            mx, my = self.window.mouse_position

            if self.window.mouse_click(0):
                pixel = self.window.download_uint32_target_pixel(
                    "Segmentation", int(mx), int(my)
                )
                actor = self.find_actor(pixel[1])
                if actor:
                    if self.selected_actor:
                        for v in self.selected_actor.get_visual_bodies():
                            v.set_visibility(1)
                    self.selected_actor = actor
                    for v in self.selected_actor.get_visual_bodies():
                        v.set_visibility(0.6)
                else:
                    if self.selected_actor:
                        for v in self.selected_actor.get_visual_bodies():
                            v.set_visibility(1)
                    self.selected_actor = None
                    if self.axes:
                        self.axes.set_position([0, 0, 0])
                        self.axes.set_rotation([0, 0, 0, 1])

            if self.selected_actor:
                if self.axes:
                    self.axes.set_position(self.selected_actor.pose.p)
                    self.axes.set_rotation(self.selected_actor.pose.q)

            speed_mod = 1
            if self.window.shift:
                speed_mod *= 0.1
            if self.window.key_down("w"):
                self.camera_controller.move(0.1 * speed_mod, 0, 0)
            if self.window.key_down("s"):
                self.camera_controller.move(-0.1 * speed_mod, 0, 0)
            if self.window.key_down("a"):
                self.camera_controller.move(0, 0.1 * speed_mod, 0)
            if self.window.key_down("d"):
                self.camera_controller.move(0, -0.1 * speed_mod, 0)
            if self.window.mouse_down(1):
                x, y = self.window.mouse_delta
                self.camera_controller.rotate(
                    0, -0.005 * speed_mod * y, -0.005 * speed_mod * x
                )

            if self.window.mouse_down(2):
                x, y = self.window.mouse_delta
                self.camera_controller.move(
                    0, 0.005 * speed_mod * x, 0.005 * speed_mod * y
                )
            wx, wy = self.window.mouse_wheel_delta
            self.camera_controller.move(0.1 * speed_mod * wx, 0, 0)

            if self.window.key_down("q"):
                self.close()
                return

            if not self.paused or (self.paused and self.single_step):
                self.single_step = False
                break
