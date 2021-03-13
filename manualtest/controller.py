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


class ArcRotateCameraController:
    def __init__(self, window: VulkanWindow):
        self.window = window
        self.forward = np.array([1, 0, 0])
        self.up = np.array([0, 0, 1])
        self.left = np.cross(self.up, self.forward)
        self.initial_rotation = mat2quat(
            np.array([-self.left, self.up, -self.forward]).T
        )
        self.center = np.zeros(3)
        self.yaw = 0
        self.pitch = 0
        self.radius = 1

    def set_center(self, center):
        self.center = np.array(center)
        self.update()

    def rotate_yaw_pitch(self, yaw, pitch):
        self.yaw += yaw
        self.pitch += pitch
        self.update()

    def set_yaw_pitch(self, yaw, pitch):
        self.yaw = yaw
        self.pitch = pitch
        self.update()

    def zoom(self, zoom_in):
        self.radius -= zoom_in
        self.radius = max(0.1, self.radius)
        self.radius = min(100, self.radius)
        self.update()

    def set_zoom(self, zoom):
        self.radius = zoom
        self.update()

    def update(self):
        rot = qmult(
            qmult(aa(self.up, self.yaw), aa(self.left, self.pitch)),
            self.initial_rotation,
        )
        pos = self.center - self.radius * rotate_vector(np.array([0, 0, -1]), rot)
        self.window.set_camera_rotation(rot)
        self.window.set_camera_position(pos)


class Viewer(object):
    def __init__(
        self,
        renderer: VulkanRenderer,
        shader_dir="",
        resolutions=[(800, 600), (1024, 768), (1920, 1080)],
    ):
        self.shader_dir = shader_dir
        self.renderer = renderer
        self.renderer_context: R.Context = renderer._internal_context
        self.set_window_resolutions(resolutions)

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

        self.axes = None
        self.selected_actor = None
        self.focused_actor = None
        self.paused = False
        self.target_name = "Color"
        self.single_step = False

        self.move_speed = 0.01
        self.rotate_speed = 0.005

        self.scene_window = None
        self.control_window = None

    def set_window_resolutions(self, resolutions):
        assert len(resolutions)
        for r in resolutions:
            assert len(r) == 2

        self.window = self.renderer.create_window(
            resolutions[0][0], resolutions[0][1], self.shader_dir
        )
        self.resolutions = resolutions

    def build_control_window(self):
        if not self.control_window:
            self.control_window = (
                R.UIWindow()
                .Label("Main")
                .Pos(10, 10)
                .Size(400, 400)
                .append(
                    R.UISection()
                    .Label("Control")
                    .Expanded(True)
                    .append(
                        R.UICheckbox()
                        .Label("Pause")
                        .Callback(lambda p: self.toggle_pause(p.checked)),
                        R.UIButton()
                        .Label("Step")
                        .Callback(lambda p: self.step_button()),
                        R.UICheckbox()
                        .Label("Axes")
                        .Checked(True)
                        .Callback(lambda p: self.toggle_axes(p.checked)),
                        R.UISliderFloat()
                        .Min(0.01)
                        .Max(1)
                        .Value(0.1)
                        .Label("Camera Move Speed")
                        .Callback(lambda w: self.set_move_speed(w.value)),
                        R.UISliderFloat()
                        .Min(0.001)
                        .Max(0.01)
                        .Value(0.005)
                        .Label("Camera Rotate Speed")
                        .Callback(lambda w: self.set_rotate_speed(w.value)),
                        R.UIOptions()
                        .Style("select")
                        .Label("Display Mode")
                        .Index(0)
                        .Items(
                            ["Color"]
                            + [x for x in self.window.target_names if x != "Color"]
                        )
                        .Callback(lambda p: self.set_target(p.value)),
                        R.UIOptions()
                        .Style("select")
                        .Label("Resolution")
                        .Index(0)
                        .Items(["{}x{}".format(r[0], r[1]) for r in self.resolutions])
                        .Callback(lambda p: self.set_resolution(p.index)),
                    ),
                )
            )

    def set_resolution(self, index):
        width, height = self.resolutions[index]
        self.window.resize(width, height)

    def build_scene_window(self):
        assert self.scene
        if not self.scene_window:
            self.scene_window = (
                R.UIWindow()
                .Pos(410, 10)
                .Size(400, 400)
                .Label("Scene Hierarchy")
                .append(
                    R.UITreeNode()
                    .Label("World")
                    .append(
                        R.UITreeNode().Label("Actors"),
                        R.UITreeNode().Label("Articulations"),
                    ),
                )
            )

        atree, arttree = self.scene_window.get_children()[0].get_children()
        atree: R.UITreeNode
        arttree: R.UITreeNode
        atree.remove_children()
        arttree.remove_children()
        for i, actor in enumerate(self.scene.get_all_actors()):
            atree.append(
                R.UISelectable()
                .Label(
                    "{}##actor{}".format(actor.name if actor.name else "(no name)", i)
                )
                .Selected(self.selected_actor == actor)
                .Callback(lambda _: self.select_actor(actor))
            )

        for i, art in enumerate(self.scene.get_all_articulations()):
            art_node = R.UITreeNode().Label(
                "{}##art{}".format(art.name if art.name else "(no name)", i)
            )
            for j, link in enumerate(art.get_base_links()):
                art_node.append(
                    R.UISelectable()
                    .Label(
                        "{}##link{}".format(link.name if link.name else "(no name)", j)
                    )
                    .Selected(self.selected_actor == link)
                    .Callback((lambda link: lambda _: self.select_actor(link))(link))
                )
            arttree.append(art_node)

    def build_actor_window(self):
        self.actor_window = R.UIWindow().Label("Actor")
        if not self.selected_actor:
            self.actor_window.append(R.UIDisplayText().Text("No actor selected."))
            return

        actor = self.selected_actor

        self.actor_window.append(
            R.UIDisplayText().Text("Name: {}".format(actor.name)),
            R.UIDisplayText().Text("Type: {}".format(actor.type)),
            R.UIDisplayText().Text("Id: {}".format(actor.id)),
            R.UIDisplayText().Text(
                "position(xyz): {:.5g},{:.5g},{:.5g}".format(*actor.pose.p)
            ),
            R.UIDisplayText().Text(
                "rotation(wxyz): {:.5g},{:.5g},{:.5g},{:.5g}".format(*actor.pose.q)
            ),
            R.UIDisplayText().Text(
                "collision: {:08x},{:08x},{:08x}".format(
                    actor.col1, actor.col2, actor.col3
                )
            ),
        )

    def set_move_speed(self, x):
        self.move_speed = x

    def set_rotate_speed(self, x):
        self.rotate_speed = x

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
        self.fps_camera_controller = FPSCameraController(self.window)
        self.arc_camera_controller = ArcRotateCameraController(self.window)
        self.toggle_axes(True)

    def set_camera_xyz(self, x, y, z):
        self.fps_camera_controller.setXYZ(x, y, z)

    def set_camera_rpy(self, r, p, y):
        self.fps_camera_controller.setRPY(r, p, y)

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
        self.fps_camera_controller = None
        self.window = None

    def focus_actor(self, actor):
        if actor == self.focused_actor:
            return

        self.focused_actor = actor
        if actor:
            pos = self.window.get_camera_position()
            rot = self.window.get_camera_rotation()
            x, y, z = rotate_vector([0, 0, -1], rot)
            yaw = np.arctan2(y, x)
            pitch = np.arctan2(z, np.linalg.norm([x, y]))
            self.arc_camera_controller.set_yaw_pitch(yaw, -pitch)
            self.arc_camera_controller.set_center(actor.pose.p)
            self.arc_camera_controller.set_zoom(np.linalg.norm(actor.pose.p - pos))
        else:
            rot = self.window.get_camera_rotation()
            x, y, z = rotate_vector([0, 0, -1], rot)
            yaw = np.arctan2(y, x)
            pitch = np.arctan2(z, np.linalg.norm([x, y]))
            self.fps_camera_controller.setXYZ(*self.window.get_camera_position())
            self.fps_camera_controller.setRPY(0, pitch, yaw)

    def select_actor(self, actor):
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

    def update_axes(self):
        if self.selected_actor and self.axes:
            self.axes.set_position(self.selected_actor.pose.p)
            self.axes.set_rotation(self.selected_actor.pose.q)

    def render(self):
        if self.closed:
            return

        while True:
            self.build_control_window()
            self.build_scene_window()
            self.build_actor_window()
            self.window.render(
                self.target_name,
                [self.control_window, self.scene_window, self.actor_window],
            )
            mx, my = self.window.mouse_position

            if self.window.mouse_click(0):
                pixel = self.window.download_uint32_target_pixel(
                    "Segmentation", int(mx), int(my)
                )
                actor = self.find_actor(pixel[1])
                self.select_actor(actor)

            self.update_axes()

            speed_mod = 1
            if self.window.shift:
                speed_mod *= 0.1
            if self.window.key_down("f"):
                if self.selected_actor:
                    self.focus_actor(self.selected_actor)
            if self.window.key_down("w"):
                self.focus_actor(None)
                self.fps_camera_controller.move(self.move_speed * speed_mod, 0, 0)
            if self.window.key_down("s"):
                self.focus_actor(None)
                self.fps_camera_controller.move(-self.move_speed * speed_mod, 0, 0)
            if self.window.key_down("a"):
                self.focus_actor(None)
                self.fps_camera_controller.move(0, self.move_speed * speed_mod, 0)
            if self.window.key_down("d"):
                self.focus_actor(None)
                self.fps_camera_controller.move(0, -self.move_speed * speed_mod, 0)
            if self.window.mouse_down(1):
                x, y = self.window.mouse_delta
                if self.focused_actor:
                    self.arc_camera_controller.rotate_yaw_pitch(-0.01 * x, 0.01 * y)
                else:
                    self.fps_camera_controller.rotate(
                        0,
                        -self.rotate_speed * speed_mod * y,
                        -self.rotate_speed * speed_mod * x,
                    )

            if self.window.mouse_down(2):
                x, y = self.window.mouse_delta
                self.focus_actor(None)
                self.fps_camera_controller.move(
                    0,
                    self.rotate_speed * speed_mod * x,
                    self.rotate_speed * speed_mod * y,
                )

            wx, wy = self.window.mouse_wheel_delta
            if wx != 0:
                if self.focused_actor:
                    self.arc_camera_controller.zoom(0.1 * wx)
                else:
                    self.fps_camera_controller.move(0.1 * speed_mod * wx, 0, 0)

            if self.focused_actor:
                self.arc_camera_controller.set_center(self.focused_actor.pose.p)

            if self.window.key_down("q"):
                self.close()
                return

            if not self.paused or (self.paused and self.single_step):
                self.single_step = False
                break
