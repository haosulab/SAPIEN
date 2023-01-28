from sapien.core import renderer as R
from sapien.core import (
    ActorBase,
    Pose,
    SapienRenderer,
    Scene,
    VulkanWindow,
    ArticulationBase,
    Joint,
    LinkBase,
    LightEntity,
    PointLightEntity,
    DirectionalLightEntity,
    SpotLightEntity,
    CameraEntity,
)
import sapien.core as sapien
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qmult, mat2quat, rotate_vector, qinverse
import numpy as np
import os

import pkg_resources
from pathlib import Path

shader_dir = Path(pkg_resources.resource_filename("sapien", "vulkan_shader"))
shader_list = []
for f in shader_dir.iterdir():
    if f.is_dir():
        if any(
            "gbuffer.frag" in x.name or "camera.rgen" in x.name for x in f.iterdir()
        ):
            shader_list.append(f)


imgui_ini = """
[Window][DockSpace Demo]
Pos=0,0
Size=1024,768
Collapsed=0

[Window][Actor/Entity]
Pos=726,23
Size=298,335
Collapsed=0
DockId=0x00000007,0

[Window][Control]
Pos=0,23
Size=248,368
Collapsed=0
DockId=0x00000003,0

[Window][Scene Hierarchy]
Pos=0,393
Size=248,314
Collapsed=0
DockId=0x00000004,0

[Window][Articulation]
Pos=726,360
Size=298,347
Collapsed=0
DockId=0x00000008,0

[Window][Info]
Pos=0,709
Size=1024,59
Collapsed=0
DockId=0x0000000A,0

[Window][IK]
Pos=726,360
Size=298,347
Collapsed=0
DockId=0x00000008,1

[Window][Debug##Default]
Pos=60,60
Size=400,400
Collapsed=0

[Docking][Data]
DockSpace         ID=0x4BBE4C7A Window=0x4647B76E Pos=0,23 Size=1024,745 Split=Y
  DockNode        ID=0x00000009 Parent=0x4BBE4C7A SizeRef=1024,684 Split=X
    DockNode      ID=0x00000005 Parent=0x00000009 SizeRef=724,747 Split=X
      DockNode    ID=0x00000001 Parent=0x00000005 SizeRef=248,747 Split=Y Selected=0x9A68760C
        DockNode  ID=0x00000003 Parent=0x00000001 SizeRef=399,368 Selected=0x226615D7
        DockNode  ID=0x00000004 Parent=0x00000001 SizeRef=399,314 Selected=0x9A68760C
      DockNode    ID=0x00000002 Parent=0x00000005 SizeRef=474,747 CentralNode=1
    DockNode      ID=0x00000006 Parent=0x00000009 SizeRef=298,747 Split=Y Selected=0x85B479FD
      DockNode    ID=0x00000007 Parent=0x00000006 SizeRef=121,335 Selected=0x0A9B993B
      DockNode    ID=0x00000008 Parent=0x00000006 SizeRef=121,347 Selected=0x816C7EAB
  DockNode        ID=0x0000000A Parent=0x4BBE4C7A SizeRef=1024,59 Selected=0x6BBB9E69
"""


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
            qmult(aa(self.up, -self.rpy[2]), aa(self.left, -self.rpy[1])),
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
                qmult(aa(self.up, -self.rpy[2]), aa(self.left, -self.rpy[1])),
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
        renderer: SapienRenderer,
        shader_dir="",
        resolutions=(1024, 768),
    ):
        if not os.path.exists("imgui.ini"):
            with open("imgui.ini", "w") as f:
                f.write(imgui_ini)

        if not shader_dir:
            self.shader_dir = sapien.render_config.viewer_shader_dir

        self.renderer = renderer
        self.renderer_context: R.Context = renderer._internal_context

        self.create_visual_models()

        self.scene = None

        self.resolution = np.array(resolutions).flatten()[:2]
        self.window = self.renderer.create_window(
            self.resolution[0], self.resolution[1], self.shader_dir
        )

        self.fovy = np.pi / 2

        self.axes = None
        self.show_axes = True
        self.axes_scale = 0.1

        self.selected_entity = None
        self.focused_entity = None
        self.paused = False
        self.do_not_update_render = False
        self.target_name = "Color"
        self.single_step = False

        self.focused_camera = None
        self.cameras = None
        self.camera_ui = None

        self.move_speed = 0.05
        self.rotate_speed = 0.005
        self.scroll_speed = 0.5
        self.selection_opacity = 0.7

        self.scene_window = None
        self.control_window = None
        self.info_window = None
        self.ik_window = None
        self.gizmo = None
        self.ik_enabled = False
        self.ik_display_objects = []

        self.move_group_joints = []
        self.move_group_selection = {}

        self.key_stack = ""
        self.initialize_key_action_map()

        self.mode = "normal"

        self.display_object = None
        self.coordinate_axes_mode = "Origin"
        self.immediate_mode = False

        self.camera_linesets = []
        self._show_camera_linesets = True

        self.window.set_drop_callback(self.drag_and_drop)

    def drag_and_drop(self, paths):
        b = self.scene.create_actor_builder()
        b.add_visual_from_file(paths[0])
        b.build_kinematic()

    def _clear_camera_linesets(self):
        if self.scene is None:
            return

        rs = self.scene.renderer_scene
        for n in self.camera_linesets:
            rs._internal_scene.remove_node(n)
        self.camera_linesets = []

    def _update_camera_linesets(self):
        if self.scene is None:
            return
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        cameras = self.scene.get_cameras()
        if len(self.camera_linesets) != len(cameras):
            self._clear_camera_linesets()
            for c in self.cameras:
                self.camera_linesets.append(
                    render_scene.add_line_set(self.camera_lineset)
                )
        for lineset, camera in zip(self.camera_linesets, cameras):
            lineset: R.LineSetObject
            mat = camera.get_model_matrix()
            lineset.set_position(mat[:3, 3])
            lineset.set_rotation(mat2quat(mat[:3, :3]))

            scaley = np.tan(camera.fovy / 2)
            scalex = np.tan(camera.fovx / 2)
            lineset.set_scale(np.array([scalex, scaley, 1]) * 0.3)

    def create_visual_models(self):
        self.cone = self.renderer_context.create_cone_mesh(16)
        self.capsule = self.renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)

        self.mat_red = self.renderer_context.create_material(
            [0, 0, 0, 1], [1, 0, 0, 1], 0, 0, 0
        )
        self.mat_green = self.renderer_context.create_material(
            [0, 0, 0, 1], [0, 1, 0, 1], 0, 0, 0
        )
        self.mat_blue = self.renderer_context.create_material(
            [0, 0, 0, 1], [0, 0, 1, 1], 0, 0, 0
        )

        self.mat_cyan = self.renderer_context.create_material(
            [0, 0, 0, 1], [0, 1, 1, 1], 0, 0, 0
        )
        self.mat_magenta = self.renderer_context.create_material(
            [0, 0, 0, 1], [1, 0, 1, 1], 0, 0, 0
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

        self.camera_lineset = self.renderer_context.create_line_set(
            [
                0,
                0,
                0,
                1,
                1,
                -1,
                0,
                0,
                0,
                -1,
                1,
                -1,
                0,
                0,
                0,
                1,
                -1,
                -1,
                0,
                0,
                0,
                -1,
                -1,
                -1,
                1,
                1,
                -1,
                1,
                -1,
                -1,
                1,
                -1,
                -1,
                -1,
                -1,
                -1,
                -1,
                -1,
                -1,
                -1,
                1,
                -1,
                -1,
                1,
                -1,
                1,
                1,
                -1,
                1,
                1.2,
                -1,
                0,
                2,
                -1,
                0,
                2,
                -1,
                -1,
                1.2,
                -1,
                -1,
                1.2,
                -1,
                1,
                1.2,
                -1,
            ],
            [0.9254901960784314, 0.5764705882352941, 0.18823529411764706, 1] * 22,
        )

    def _create_coordinate_axes(self):
        assert self.scene is not None
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        node = render_scene.add_node()
        obj = render_scene.add_object(self.red_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([1, 0, 0])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.red_capsule, node)
        obj.set_position([0.5, 0, 0])
        obj.set_scale([1.02, 1.02, 1.02])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.green_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 1, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.green_capsule, node)
        obj.set_position([0, 0.5, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.blue_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 0, 1])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.blue_capsule, node)
        obj.set_position([0, 0, 0.5])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 1

        return node

    def _create_grab_axes(self):
        assert self.scene is not None
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        grab_axes = [
            render_scene.add_object(model)
            for model in [self.red_capsule, self.green_capsule, self.blue_capsule]
        ]

        for obj in grab_axes:
            obj.set_position([0, 0, 0])
            obj.set_scale([100, 0.1, 0.1])
            obj.shading_mode = 2
            obj.cast_shadow = False
            obj.transparency = 1
        return grab_axes

    def _create_joint_axes(self):
        assert self.scene is not None
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        joint_axes = [
            render_scene.add_object(self.magenta_capsule),
            render_scene.add_object(self.cyan_capsule),
        ]
        for obj in joint_axes:
            obj.set_position([0, 0, 0])
            obj.set_scale([5, 0.1, 0.1])
            obj.shading_mode = 2
            obj.cast_shadow = False
            obj.transparency = 1
        return joint_axes

    def create_visual_objects(self):
        if hasattr(self, "coordinate_axes") and self.coordinate_axes:
            self.scene.renderer_scene._internal_scene.remove_node(self.coordinate_axes)
            for n in self.grab_axes:
                self.scene.renderer_scene._internal_scene.remove_node(n)
            for n in self.joint_axes:
                self.scene.renderer_scene._internal_scene.remove_node(n)

        self.coordinate_axes = self._create_coordinate_axes()
        self.grab_axes = self._create_grab_axes()
        self.joint_axes = self._create_joint_axes()

    def enter_mode(self, name):
        if self.mode == name:
            return
        self.leave_mode(self.mode)
        self.mode = name
        if name == "grab":
            self.window.cursor = False
        else:
            self.window.cursor = True

    def leave_mode(self, name):
        if name == "grab":
            for obj in self.grab_axes:
                obj.transparency = 1
            if self.display_object:
                rs = self.scene.renderer_scene
                render_scene: R.Scene = rs._internal_scene
                render_scene.remove_node(self.display_object)
                self.display_object = None
        elif name == "rotate":
            for obj in self.grab_axes:
                obj.transparency = 1
            if self.display_object:
                rs = self.scene.renderer_scene
                render_scene: R.Scene = rs._internal_scene
                render_scene.remove_node(self.display_object)
                self.display_object = None

    def add_display_object(self):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene
        if not isinstance(self.selected_entity, ActorBase):
            self.display_object = render_scene.add_node()
            self.display_object.set_position(self.selected_entity.pose.p)
            self.display_object.set_rotation(self.selected_entity.pose.q)
            return

        if self.display_object:
            render_scene.remove_node(self.display_object)
            self.display_object = None
        self.display_object = render_scene.add_node()
        selected2world = self.selected_entity.pose
        for body in self.selected_entity.get_visual_bodies():
            for obj in body._internal_objects:
                scale = obj.scale
                obj2world = Pose(obj.position, obj.rotation)
                obj2selected = selected2world.inv() * obj2world
                new_obj = render_scene.add_object(obj.model, self.display_object)
                new_obj.set_position(obj2selected.p)
                new_obj.set_rotation(obj2selected.q)
                new_obj.set_scale(scale)
                new_obj.transparency = 0.1
        self.display_object.set_position(selected2world.p)
        self.display_object.set_rotation(selected2world.q)

    def initialize_key_action_map(self):
        x2y = np.array([0.7071068, 0, 0, 0.7071068])
        x2z = np.array([0.7071068, 0, 0.7071068, 0])

        def f():
            if self.selected_entity:
                self.focus_entity(self.selected_entity)

        def r():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 1
            self.add_display_object()

            point = self.world_space_to_screen_space(self.selected_entity.pose.p)
            axis = self.screen_space_to_world_space(
                [point[0], point[1], 1]
            ) - self.screen_space_to_world_space([point[0], point[1], 0])
            axis = axis / np.linalg.norm(axis)
            self.rotate_axis = np.array(axis)
            self.rotate_direction = 1
            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def rx():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 1
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation([1, 0, 0, 0])
            self.add_display_object()
            self.rotate_axis = np.array([1, 0, 0])

            screen_vector = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_entity.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def rxx():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 1
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation(self.selected_entity.pose.q)
            self.add_display_object()
            self.rotate_axis = np.array(
                rotate_vector([1, 0, 0], self.selected_entity.pose.q)
            )

            screen_vector = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_entity.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def rxxx():
            self.key_stack = "r"
            r()

        def ry():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 1
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(x2y)
            self.add_display_object()
            self.rotate_axis = np.array([0, 1, 0])

            screen_vector = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_entity.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def ryy():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 1
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(qmult(self.selected_entity.pose.q, x2y))
            self.add_display_object()
            self.rotate_axis = np.array(
                rotate_vector([0, 1, 0], self.selected_entity.pose.q)
            )

            screen_vector = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_entity.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def ryyy():
            self.key_stack = "r"
            r()

        def rz():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 0
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(x2z)
            self.add_display_object()
            self.rotate_axis = np.array([0, 0, 1])

            screen_vector = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_entity.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def rzz():
            if not self.selected_entity:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 0
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(qmult(self.selected_entity.pose.q, x2z))
            self.add_display_object()
            self.rotate_axis = np.array(
                rotate_vector([0, 0, 1], self.selected_entity.pose.q)
            )

            screen_vector = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_entity.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_entity.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_entity.pose.p
            )[:2]

        def rzzz():
            self.key_stack = "r"
            r()

        def g():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 1
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = None

        def gx():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 1
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation([1, 0, 0, 0])
            self.add_display_object()
            self.grab_axis = np.array([1, 0, 0])
            self.grab_plane = None

        def gxx():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 1
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation(self.selected_entity.pose.q)
            self.add_display_object()
            self.grab_axis = rotate_vector([1, 0, 0], self.selected_entity.pose.q)
            self.grab_plane = None

        def gxxx():
            self.key_stack = "g"
            g()

        def gX():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 0
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(x2y)
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(x2z)
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = np.array([1, 0, 0])

        def gXX():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 0
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(qmult(self.selected_entity.pose.q, x2y))
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(qmult(self.selected_entity.pose.q, x2z))
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = rotate_vector(
                np.array([1, 0, 0]), self.selected_entity.pose.q
            )

        def gXXX():
            self.key_stack = "g"
            g()

        def gy():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 1
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(x2y)
            self.add_display_object()
            self.grab_axis = np.array([0, 1, 0])
            self.grab_plane = None

        def gyy():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 1
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(qmult(self.selected_entity.pose.q, x2y))
            self.add_display_object()
            self.grab_axis = rotate_vector([0, 1, 0], self.selected_entity.pose.q)
            self.grab_plane = None

        def gyyy():
            self.key_stack = "g"
            g()

        def gY():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 0
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation([1, 0, 0, 0])
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(x2z)
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = np.array([0, 1, 0])

        def gYY():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 0
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation(
                qmult(self.selected_entity.pose.q, [1, 0, 0, 0])
            )
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(qmult(self.selected_entity.pose.q, x2z))
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = rotate_vector(
                np.array([0, 1, 0]), self.selected_entity.pose.q
            )

        def gYYY():
            self.key_stack = "g"
            g()

        def gz():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 0
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(x2z)
            self.add_display_object()
            self.grab_axis = np.array([0, 0, 1])
            self.grab_plane = None

        def gzz():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 1
            self.grab_axes[1].transparency = 1
            self.grab_axes[2].transparency = 0
            self.grab_axes[2].set_position(self.selected_entity.pose.p)
            self.grab_axes[2].set_rotation(qmult(self.selected_entity.pose.q, x2z))
            self.add_display_object()
            self.grab_axis = rotate_vector([0, 0, 1], self.selected_entity.pose.q)
            self.grab_plane = None

        def gzzz():
            self.key_stack = "g"
            g()

        def gZ():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 1
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation([1, 0, 0, 0])
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(x2y)
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = np.array([0, 0, 1])

        def gZZ():
            if not self.selected_entity:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_axes[0].transparency = 0
            self.grab_axes[1].transparency = 0
            self.grab_axes[2].transparency = 1
            self.grab_axes[0].set_position(self.selected_entity.pose.p)
            self.grab_axes[0].set_rotation(
                qmult(self.selected_entity.pose.q, [1, 0, 0, 0])
            )
            self.grab_axes[1].set_position(self.selected_entity.pose.p)
            self.grab_axes[1].set_rotation(qmult(self.selected_entity.pose.q, x2y))
            self.add_display_object()
            self.grab_axis = None
            self.grab_plane = rotate_vector(
                np.array([0, 0, 1]), self.selected_entity.pose.q
            )

        def gZZZ():
            self.key_stack = "g"
            g()

        self.key_press_action_map = {
            "f": f,
            "r": r,
            "rx": rx,
            "rxx": rxx,
            "rxxx": rxxx,
            "ry": ry,
            "ryy": ryy,
            "ryyy": ryyy,
            "rz": rz,
            "rzz": rzz,
            "rzzz": rzzz,
            "g": g,
            "gx": gx,
            "gxx": gxx,
            "gxxx": gxxx,
            "gX": gX,
            "gXX": gXX,
            "gXXX": gXXX,
            "gy": gy,
            "gyy": gyy,
            "gyyy": gyyy,
            "gY": gY,
            "gYY": gYY,
            "gYYY": gYYY,
            "gz": gz,
            "gzz": gzz,
            "gzzz": gzzz,
            "gZ": gZ,
            "gZZ": gZZ,
            "gZZZ": gZZZ,
        }

        def w():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_entity(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(self.move_speed * speed_mod, 0, 0)
            self.key_stack = ""

        def s():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_entity(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(-self.move_speed * speed_mod, 0, 0)
            self.key_stack = ""

        def a():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_entity(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(0, self.move_speed * speed_mod, 0)
            self.key_stack = ""

        def d():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_entity(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(0, -self.move_speed * speed_mod, 0)
            self.key_stack = ""

        self.key_down_action_map = {"w": w, "s": s, "a": a, "d": d}

    def key_press_action(self, key):
        if len(key) == 1 and ord("a") <= ord(key) <= ord("z") and self.window.shift:
            key = key.upper()
        if self.key_stack + key in self.key_press_action_map:
            self.key_stack += key
            self.key_press_action_map[self.key_stack]()
            return
        if self.key_stack == "":
            if key in self.key_press_action_map:
                self.key_stack = key
                self.key_press_action_map[key]()
            return
        self.key_stack = self.key_stack[:-1]
        self.key_press_action(key)

    def key_down_action(self, key):
        if key in self.key_down_action_map:
            self.key_down_action_map[key]()

    def set_fovy(self, fovy):
        self.fovy = fovy
        self.window.set_camera_parameters(0.1, 100, fovy)

    def set_shader(self, index):
        self.shader_dir = str(shader_list[index])
        self.window.set_shader_dir(self.shader_dir)
        self.reset_windows()

    def enable_denoiser(self, enable):
        if enable:
            self.window.set_camera_property("_denoiser", 1)
        else:
            self.window.set_camera_property("_denoiser", 0)

    def build_control_window(self):
        if not self.control_window:
            self.cameras = self.scene.get_cameras()
            self.camera_ui = (
                R.UIOptions()
                .Style("select")
                .Label("Name##camera_name")
                .Index(
                    0
                    if self.focused_camera is None
                    else self.cameras.index(self.focused_camera) + 1
                )
                .Items(
                    ["None"]
                    + [x.get_name() + f"##{i}" for i, x in enumerate(self.cameras)]
                )
                .Callback(
                    lambda p: self.focus_camera(
                        self.cameras[p.index - 1] if p.index > 0 else None
                    )
                )
            )

            self.ui_pause_checkbox = (
                R.UICheckbox()
                .Label("Pause")
                .Checked(self.paused)
                .Callback(lambda p: self.toggle_pause(p.checked))
            )

            self.update_render_checkbox = (
                R.UICheckbox()
                .Label("Do not Update Render")
                .Checked(self.do_not_update_render)
                .Callback(lambda p: setattr(self, "do_not_update_render", p.checked))
            )

            self.control_window = (
                R.UIWindow().Label("Control").Pos(10, 10).Size(400, 400)
            )

            self.control_window.append(
                R.UISameLine().append(
                    self.ui_pause_checkbox,
                    R.UIButton()
                    .Label("Single Step")
                    .Callback(lambda p: self.step_button()),
                    self.update_render_checkbox,
                ),
                R.UIDisplayText().Text("Camera Speed"),
                R.UISliderFloat()
                .Min(0.01)
                .Max(1)
                .Value(self.move_speed)
                .Label("Move")
                .Callback(lambda w: self.set_move_speed(w.value)),
                R.UISliderFloat()
                .Min(0.001)
                .Max(0.01)
                .Value(self.rotate_speed)
                .Label("Rotate")
                .Callback(lambda w: self.set_rotate_speed(w.value)),
                R.UISliderFloat()
                .Min(0.1)
                .Max(1)
                .Value(self.scroll_speed)
                .Label("Scroll")
                .Callback(lambda w: self.set_scroll_speed(w.value)),
                R.UIDisplayText().Text("Camera"),
                self.camera_ui,
                R.UIButton()
                .Label("Copy Camera Settings")
                .Callback(lambda c: self.copy_camera_settings()),
                R.UIDisplayText().Text("Display Settings"),
                R.UIOptions()
                .Style("select")
                .Label("Shader")
                .Items([d.name for d in shader_list])
                .Callback(lambda p: self.set_shader(p.index)),
            )

            if "camera.rgen" in os.listdir(self.shader_dir):
                self.control_window.append(
                    R.UIDisplayText().Text("Denoiser"),
                    R.UISameLine().append(
                        R.UIButton()
                        .Label("Enable")
                        .Callback(lambda x: self.enable_denoiser(True)),
                        R.UIButton()
                        .Label("Disable")
                        .Callback(lambda x: self.enable_denoiser(False)),
                    ),
                )

            self.control_window.append(
                R.UISliderAngle()
                .Min(1)
                .Max(179)
                .Value(self.fovy)
                .Label("Fov Y")
                .Callback(lambda w: self.set_fovy(w.value)),
                R.UIOptions()
                .Style("select")
                .Label("Render Target")
                .Index(0)
                .Items(
                    ["Color"]
                    + [x for x in self.window.display_target_names if x != "Color"]
                )
                .Callback(lambda p: self.set_target(p.value)),
                R.UIInputInt2()
                .Label("Resolution")
                .Value(self.resolution)
                .Callback(lambda c: self.set_resolution(c.value)),
                R.UIDisplayText().Text("Actor Selection"),
                R.UICheckbox()
                .Label("Coordinate Axes")
                .Checked(True)
                .Callback(lambda p: self.toggle_axes(p.checked)),
                R.UICheckbox()
                .Label("Camera Display")
                .Checked(True)
                .Callback(lambda p: self.toggle_camera_lines(p.checked)),
                R.UIOptions()
                .Style("select")
                .Label("Axes Mode")
                .Index(0)
                .Items(["Origin", "Center of Mass"])
                .Callback(lambda p: self.set_coordinate_axes_mode(p.value)),
                R.UISliderFloat()
                .Label("Axes Scale")
                .Min(0)
                .Max(1)
                .Value(self.axes_scale)
                .Callback(lambda p: self.update_coordinate_axes_scale(p.value)),
                R.UISliderFloat()
                .Label("Opacity")
                .Min(0)
                .Max(1)
                .Value(self.selection_opacity)
                .Callback(lambda p: self.set_selection_opacity(p.value)),
                R.UICheckbox()
                .Label("Immediate Move")
                .Checked(self.immediate_mode)
                .Callback(lambda p: self.set_immediate_move(p.checked)),
                R.UIButton().Label("Take Screenshot").Callback(self.take_screenshot),
                R.UIDisplayText().Text("FPS: {:.2f}".format(self.window.fps)),
            )

        self.control_window.get_children()[-1].Text(
            "FPS: {:.2f}".format(self.window.fps)
        )

    def copy_camera_settings(self):
        p = self.window.get_camera_position()
        q = qmult(self.window.get_camera_rotation(), [0.5, -0.5, 0.5, 0.5])
        width, height = self.window.size
        fovy = self.window.fovy
        near = self.window.near
        far = self.window.far
        import pyperclip

        pyperclip.copy(
            f'camera = add_camera(name="", width={width}, height={height}, fovy={fovy:.3g}, near={near:.3g}, far={far:.3g})\n'
            f"camera.set_local_pose({Pose(p, q).__repr__()})"
        )

    def enable_ik(self, enable):
        self.ik_enabled = enable
        self.refresh_ik()
        self.refresh_ik_display_objects()

    def build_ik_window(self):
        if not self.ik_window:
            self.gizmo = R.UIGizmo().Matrix(np.eye(4))
            self.move_group = R.UISection().Label("Move Group")
            self.ik_window = (
                R.UIWindow()
                .Label("IK")
                .Pos(10, 10)
                .Size(400, 400)
                .append(
                    R.UISameLine().append(
                        R.UICheckbox()
                        .Label("Enable IK")
                        .Callback(lambda c: self.enable_ik(c.checked)),
                        R.UIButton().Label("Go!").Callback(self.execute_ik),
                    ),
                    self.move_group,
                )
            )

        if self.ik_enabled:
            if self.ik_window.get_children()[-1] != self.gizmo:
                self.ik_window.append(self.gizmo)
        elif self.ik_window.get_children()[-1] == self.gizmo:
            self.ik_window = (
                R.UIWindow()
                .Label("IK")
                .Pos(10, 10)
                .Size(400, 400)
                .append(
                    R.UISameLine().append(
                        R.UICheckbox()
                        .Label("Enable IK")
                        .Callback(lambda c: self.enable_ik(c.checked)),
                        R.UIButton().Label("Go!").Callback(self.execute_ik),
                    ),
                    self.move_group,
                )
            )

    def take_screenshot(self, _):
        picture = self.window.get_float_texture(self.target_name)
        for i in range(100000000):
            n = f"sapien_screenshot_{i}.png"
            if os.path.exists(n):
                continue
            from PIL import Image

            Image.fromarray((picture.clip(0, 1) * 255).astype(np.uint8)).save(n)
            break

    def set_immediate_move(self, enabled):
        self.immediate_mode = enabled

    def set_selection_opacity(self, opacity):
        self.selection_opacity = opacity

    # def set_resolution(self, index):
    #     self.resolution = self.resolutions[index]
    #     self.window.resize(*self.resolution)

    def set_resolution(self, res):
        if res[0] > 0 and res[1] > 0:
            self.resolution = res
            self.window.resize(*res)

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
                        R.UITreeNode().Label("Lights"),
                    ),
                )
            )

        atree, arttree, ltree = self.scene_window.get_children()[0].get_children()
        atree: R.UITreeNode
        arttree: R.UITreeNode
        ltree: R.UITreeNode
        atree.remove_children()
        arttree.remove_children()
        ltree.remove_children()
        for i, actor in enumerate(self.scene.get_all_actors()):
            atree.append(
                R.UISelectable()
                .Label(
                    "{}##actor{}".format(actor.name if actor.name else "(no name)", i)
                )
                .Selected(self.selected_entity == actor)
                .Callback((lambda link: lambda _: self.select_entity(link))(actor))
            )

        for i, art in enumerate(self.scene.get_all_articulations()):
            art_node = R.UITreeNode().Label(
                "{}##art{}".format(art.name if art.name else "(no name)", i)
            )
            for j, link in enumerate(art.get_links()):
                art_node.append(
                    R.UISelectable()
                    .Label(
                        "{}##link{}".format(link.name if link.name else "(no name)", j)
                    )
                    .Selected(self.selected_entity == link)
                    .Callback((lambda link: lambda _: self.select_entity(link))(link))
                )
            arttree.append(art_node)

        for i, light in enumerate(self.scene.get_all_lights()):
            ltree.append(
                R.UISelectable()
                .Label(
                    "{}##light{}".format(light.name if light.name else "(no name)", i)
                )
                .Selected(self.selected_entity == light)
                .Callback((lambda light: lambda _: self.select_entity(light))(light))
            )

    def build_actor_window(self):
        self.actor_window = R.UIWindow().Label("Actor/Entity")
        if not self.selected_entity:
            self.actor_window.append(
                R.UIDisplayText().Text("No actor/entity selected.")
            )
            return
        if isinstance(self.selected_entity, ActorBase):
            actor = self.selected_entity
            self.actor_window.append(
                R.UIDisplayText().Text("Name: {}".format(actor.name)),
                R.UIDisplayText().Text("Class: {}".format(actor.classname)),
                R.UIDisplayText().Text("Id: {}".format(actor.id)),
            )
            self.actor_window.append(
                R.UIDisplayText().Text("Position"),
                R.UIInputFloat()
                .Label("x##actorpx")
                .Value(actor.pose.p[0])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("y##actorpy")
                .Value(actor.pose.p[1])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("z##actorpz")
                .Value(actor.pose.p[2])
                .ReadOnly(True),
                R.UIDisplayText().Text("Rotation"),
                R.UIInputFloat()
                .Label("w##actorqw")
                .Value(actor.pose.q[0])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("x##actorqx")
                .Value(actor.pose.q[1])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("y##actorqy")
                .Value(actor.pose.q[2])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("z##actorqz")
                .Value(actor.pose.q[3])
                .ReadOnly(True),
                R.UISameLine().append(
                    R.UIButton()
                    .Label("Show")
                    .Callback(
                        (lambda actor: lambda _: actor.render_collision(True))(actor)
                    ),
                    R.UIButton()
                    .Label("Hide")
                    .Callback(
                        (lambda actor: lambda _: actor.render_collision(False))(actor)
                    ),
                    R.UIDisplayText().Text("Collision"),
                ),
            )

            if actor.classname in ["Actor", "Link"]:
                self.actor_window.append(
                    R.UIInputFloat().Label("Mass").Value(actor.mass).ReadOnly(True)
                )

            # collision shapes
            shape_section = R.UISection().Label("Collision Shapes")
            self.actor_window.append(shape_section)
            shapes = actor.get_collision_shapes()
            for shape_idx, shape in enumerate(shapes):
                c0, c1, c2, c3 = shape.get_collision_groups()
                shape_pose = shape.get_local_pose()
                mat = shape.get_physical_material()

                shape_info = (
                    R.UITreeNode()
                    .Label("{}##{}".format(shape.type, shape_idx))
                    .append(
                        R.UIDisplayText().Text(
                            "Contact offset: {:.3g}".format(shape.contact_offset)
                        ),
                        R.UIDisplayText().Text(
                            "Rest offset: {:.3g}".format(shape.rest_offset)
                        ),
                        R.UIDisplayText().Text(
                            "Patch radius: {:.3g}".format(shape.patch_radius)
                        ),
                        R.UIDisplayText().Text(
                            "Min path radius: {:.3g}".format(shape.min_patch_radius)
                        ),
                        R.UICheckbox().Label("Is trigger").Checked(shape.is_trigger),
                        R.UIDisplayText().Text(
                            "Static friction: {:.3g}".format(mat.get_static_friction())
                        ),
                        R.UIDisplayText().Text(
                            "Dynamic friction: {:.3g}".format(
                                mat.get_dynamic_friction()
                            )
                        ),
                        R.UIDisplayText().Text(
                            "Restitution: {:.3g}".format(mat.get_restitution())
                        ),
                        R.UIDisplayText().Text("Collision groups:"),
                        R.UIDisplayText().Text("  0x{:08x}  0x{:08x}".format(c0, c1)),
                        R.UIDisplayText().Text("  0x{:08x}  0x{:08x}".format(c2, c3)),
                        R.UIDisplayText().Text("Local position"),
                        R.UIInputFloat()
                        .Label("x##actorpx")
                        .Value(shape_pose.p[0])
                        .ReadOnly(True),
                        R.UIInputFloat()
                        .Label("y##actorpy")
                        .Value(shape_pose.p[1])
                        .ReadOnly(True),
                        R.UIInputFloat()
                        .Label("z##actorpz")
                        .Value(shape_pose.p[2])
                        .ReadOnly(True),
                        R.UIDisplayText().Text("Local rotation"),
                        R.UIInputFloat()
                        .Label("w##actorqw")
                        .Value(shape_pose.q[0])
                        .ReadOnly(True),
                        R.UIInputFloat()
                        .Label("x##actorqx")
                        .Value(shape_pose.q[1])
                        .ReadOnly(True),
                        R.UIInputFloat()
                        .Label("y##actorqy")
                        .Value(shape_pose.q[2])
                        .ReadOnly(True),
                        R.UIInputFloat()
                        .Label("z##actorqz")
                        .Value(shape_pose.q[3])
                        .ReadOnly(True),
                    )
                )

                shape_section.append(shape_info)

                if shape.type == "sphere":
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Sphere radius: {:.3g}".format(shape.geometry.radius)
                        )
                    )
                elif shape.type == "capsule":
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Capsule radius: {:.3g}".format(shape.geometry.radius)
                        ),
                        R.UIDisplayText().Text(
                            "Capsule half length: {:.3g}".format(
                                shape.geometry.half_length
                            )
                        ),
                    )
                elif shape.type == "box":
                    x, y, z = shape.geometry.half_lengths
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Box half lengths: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                elif shape.type == "convex_mesh":
                    x, y, z = shape.geometry.scale
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Mesh scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                elif shape.type == "nonconvex_mesh":
                    x, y, z = shape.geometry.scale
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Mesh scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )

            # render shapes
            body_section = R.UISection().Label("Visual Bodies")
            self.actor_window.append(body_section)
            bodies = actor.get_visual_bodies()
            for body_idx, body in enumerate(bodies):
                body_info = R.UITreeNode().Label("{}##{}".format(body.type, body_idx))
                body_section.append(body_info)

                if body.type == "sphere":
                    body_info.append(
                        R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius))
                    )
                elif body.type == "capsule":
                    body_info.append(
                        R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius)),
                        R.UIDisplayText().Text(
                            "Half length: {:.3g}".format(body.half_length)
                        ),
                    )
                elif body.type == "box":
                    x, y, z = body.half_lengths
                    body_info.append(
                        R.UIDisplayText().Text(
                            "Half extents: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                elif body.type == "mesh":
                    x, y, z = body.scale
                    body_info.append(
                        R.UIDisplayText().Text(
                            "Scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                body_info.append(
                    R.UIDisplayText().Text("Visual id: {}".format(body.visual_id))
                )
                shapes = body.get_render_shapes()
                for shape_idx, shape in enumerate(shapes):
                    mat = shape.material
                    dtex = (
                        (mat.diffuse_texture_filename or "(has texture)")
                        if mat.diffuse_texture
                        else "(no texture)"
                    )
                    rtex = (
                        (mat.roughness_texture_filename or "(has texture)")
                        if mat.roughness_texture
                        else "(no texture)"
                    )
                    mtex = (
                        (mat.metallic_texture_filename or "(has texture)")
                        if mat.metallic_texture
                        else "(no texture)"
                    )
                    ntex = (
                        (mat.normal_texture_filename or "(has texture)")
                        if mat.normal_texture
                        else "(no texture)"
                    )
                    etex = (
                        (mat.emission_texture_filename or "(has texture)")
                        if mat.emission_texture
                        else "(no texture)"
                    )

                    body_info.append(
                        R.UITreeNode()
                        .Label("material {}##{}".format(shape_idx, body_idx))
                        .append(
                            R.UIInputFloat4()
                            .Label("Diffuse")
                            .ReadOnly(True)
                            .Value(mat.base_color),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(dtex)
                            .Label("##dtex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat4()
                            .Label("Emission")
                            .ReadOnly(True)
                            .Value(mat.emission),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(etex)
                            .Label("##etex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat()
                            .Label("Roughness")
                            .ReadOnly(True)
                            .Value(mat.roughness),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(rtex)
                            .Label("##rtex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat()
                            .Label("Metallic")
                            .ReadOnly(True)
                            .Value(mat.metallic),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(mtex)
                            .Label("##mtex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat()
                            .Label("Specular")
                            .ReadOnly(True)
                            .Value(mat.specular),
                            R.UIDisplayText().Text("Normal map"),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(ntex)
                            .Label("##ntex{}_{}".format(shape_idx, body_idx)),
                        )
                    )

        if isinstance(self.selected_entity, LightEntity):
            light = self.selected_entity
            self.actor_window.append(
                R.UIDisplayText().Text("Name: {}".format(light.name)),
                R.UIDisplayText().Text("Class: {}".format(light.classname)),
            )

            def set_shadow(light, enable):
                light.shadow = enable

            self.actor_window.append(
                R.UICheckbox()
                .Label("Shadow")
                .Checked(light.shadow)
                .Callback((lambda light: lambda p: set_shadow(light, p.checked))(light))
            )

            self.actor_window.append(
                R.UIDisplayText().Text("Position"),
                R.UIInputFloat()
                .Label("x##actorpx")
                .Value(light.pose.p[0])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("y##actorpy")
                .Value(light.pose.p[1])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("z##actorpz")
                .Value(light.pose.p[2])
                .ReadOnly(True),
                R.UIDisplayText().Text("Rotation"),
                R.UIInputFloat()
                .Label("w##actorqw")
                .Value(light.pose.q[0])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("x##actorqx")
                .Value(light.pose.q[1])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("y##actorqy")
                .Value(light.pose.q[2])
                .ReadOnly(True),
                R.UIInputFloat()
                .Label("z##actorqz")
                .Value(light.pose.q[3])
                .ReadOnly(True),
            )
            self.actor_window.append(
                R.UIInputFloat().Label("Near").Value(light.shadow_near).ReadOnly(True),
                R.UIInputFloat().Label("Far").Value(light.shadow_far).ReadOnly(True),
            )

            if light.classname == "PointLightEntity":
                light: PointLightEntity
                pass
            elif light.classname == "DirectionalLightEntity":
                self.actor_window.append(
                    R.UIInputFloat()
                    .Label("Half Size")
                    .Value(light.shadow_half_size)
                    .ReadOnly(True),
                )
                pass
            elif light.classname == "SpotLightEntity":
                pass

    def build_articulation_window(self):
        self.articulation_window = R.UIWindow().Label("Articulation")
        if (
            not self.selected_entity
            or not isinstance(self.selected_entity, ActorBase)
            or self.selected_entity.classname not in ["Link", "KinematicLink"]
        ):
            self.articulation_window.append(
                R.UIDisplayText().Text("No articulation selected.")
            )
            return

        art = self.selected_entity.get_articulation()
        art: ArticulationBase
        self.articulation_window.append(
            R.UIDisplayText().Text(
                "Name: {}".format(art.name if art.name else "(no name)")
            ),
            R.UIDisplayText().Text("Class: {}".format(art.classname)),
            R.UIDisplayText().Text("Base Link Id: {}".format(art.get_links()[0].id)),
        )
        uijoints = R.UISection().Label("Joints")
        joints = []
        for j in art.get_joints():
            if j.get_dof() > 0:
                joints.append(j)

        def wrapper(art, i, qpos):
            def callback(slider):
                qpos[i] = slider.value
                art.set_qpos(qpos)

            return callback

        qpos = art.get_qpos()
        for i, (q, j) in enumerate(zip(qpos, joints)):
            line = R.UISameLine()
            line.append(
                R.UISliderFloat()
                .Label(j.name + "##joint_{}".format(i))
                .Min(max(j.get_limits()[0][0], -20))
                .Max(min(j.get_limits()[0][1], 20))
                .Value(q)
                .Callback(wrapper(art, i, qpos)),
            )
            if art.classname == "Articulation":
                j: Joint

                line.append(
                    R.UITreeNode()
                    .Label("##joint_expand_{}".format(i))
                    .append(
                        R.UISliderFloat()
                        .Label("Drive Target##{}".format(i))
                        .Min(max(j.get_limits()[0][0], -20))
                        .Max(min(j.get_limits()[0][1], 20))
                        .Value(j.get_drive_target())
                        .Callback((lambda j: lambda p: j.set_drive_target(p.value))(j)),
                        R.UIInputFloat()
                        .Label("Damping##{}".format(i))
                        .Value(j.damping)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness,
                                    p.value,
                                    j.force_limit,
                                    j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Stiffness##{}".format(i))
                        .Value(j.stiffness)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    p.value,
                                    j.damping,
                                    j.force_limit,
                                    j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Force Limit##{}".format(i))
                        .Value(j.force_limit)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness,
                                    j.damping,
                                    p.value,
                                    j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Friction##{}".format(i))
                        .Value(j.friction)
                        .Callback((lambda j: lambda p: j.set_friction(p.value))(j)),
                        R.UICheckbox()
                        .Label("Acceleration##{}".format(i))
                        .Checked(j.drive_mode == "acceleration")
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness,
                                    j.damping,
                                    j.force_limit,
                                    "acceleration" if p.checked else "force",
                                )
                            )(j)
                        ),
                    )
                )
            uijoints.append(line)
        self.articulation_window.append(uijoints)

        def wrapper(art):
            def copy_to_clipboard(_):
                import pyperclip

                pyperclip.copy(f"[{', '.join([str(x) for x in art.get_qpos()])}]")

            return copy_to_clipboard

        self.articulation_window.append(
            R.UIButton().Label("Copy Joint Positions").Callback(wrapper(art))
        )

        def wrapper(art, show):
            def show_link_collision(_):
                for link in art.get_links():
                    link.render_collision(show)

            return show_link_collision

        self.articulation_window.append(
            R.UISameLine().append(
                R.UIButton().Label("Show").Callback(wrapper(art, True)),
                R.UIButton().Label("Hide").Callback(wrapper(art, False)),
                R.UIDisplayText().Text("Collision"),
            ),
        )

    def build_info_window(self):
        if not self.info_window:
            self.info_window = R.UIWindow().Label("Info").append(R.UIDisplayText())
        self.info_window.get_children()[0].Text("-".join(self.key_stack))

    def set_move_speed(self, x):
        self.move_speed = x

    def set_rotate_speed(self, x):
        self.rotate_speed = x

    def set_scroll_speed(self, x):
        self.scroll_speed = x

    def step_button(self):
        if self.paused:
            self.single_step = True

    def set_scene(self, scene: Scene):
        if hasattr(self, "scene") and self.scene:
            if hasattr(self, "coordinate_axes") and self.coordinate_axes:
                self.scene.renderer_scene._internal_scene.remove_node(
                    self.coordinate_axes
                )
                for n in self.grab_axes:
                    self.scene.renderer_scene._internal_scene.remove_node(n)
                for n in self.joint_axes:
                    self.scene.renderer_scene._internal_scene.remove_node(n)
            self.coordinate_axes = None
            self.grab_axes = None
            self.joint_axes = None

        self._clear_camera_linesets()

        self.scene = scene
        self.window.set_scene(scene)
        self.fps_camera_controller = FPSCameraController(self.window)
        self.arc_camera_controller = ArcRotateCameraController(self.window)
        self.create_visual_objects()
        self.toggle_axes(True)
        self.set_fovy(np.pi / 2)

    def set_camera_xyz(self, x, y, z):
        self.fps_camera_controller.setXYZ(x, y, z)

    def set_camera_rpy(self, r, p, y):
        self.fps_camera_controller.setRPY(r, p, y)

    def toggle_pause(self, paused):
        self.paused = paused
        if hasattr(self, "ui_pause_checkbox"):
            self.ui_pause_checkbox.Checked(paused)

    def toggle_axes(self, show):
        self.show_axes = show
        for c in self.coordinate_axes.children:
            if show:
                c.transparency = 0
            else:
                c.transparency = 1

    def toggle_camera_lines(self, show):
        self._show_camera_linesets = show
        if not show:
            self._clear_camera_linesets()

    def set_coordinate_axes_mode(self, mode):
        self.coordinate_axes_mode = mode

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
        self.gizmo = None
        self.move_group = None
        if hasattr(self, "coordinate_axes") and self.coordinate_axes:
            self.scene.renderer_scene._internal_scene.remove_node(self.coordinate_axes)
            for n in self.grab_axes:
                self.scene.renderer_scene._internal_scene.remove_node(n)
            for n in self.joint_axes:
                self.scene.renderer_scene._internal_scene.remove_node(n)

            self.coordinate_axes = None
            self.grab_axes = None
            self.joint_axes = None

        self._clear_camera_linesets()

        self.axes = None
        self.scene = None
        self.fps_camera_controller = None
        self.arc_camera_controller = None
        self.window = None
        self.camera_ui = None
        self.control_window = None
        self.scene_window = None
        self.actor_window = None
        self.articulation_window = None
        self.info_window = None
        self.ik_window = None

    def focus_entity(self, actor):
        if actor == self.focused_entity:
            return

        self.focused_entity = actor
        if self.focused_entity is not None:
            self.focus_camera(None)
            pos = self.window.get_camera_position()
            rot = self.window.get_camera_rotation()
            x, y, z = rotate_vector([0, 0, -1], rot)
            yaw = -np.arctan2(y, x)
            pitch = np.arctan2(z, np.linalg.norm([x, y]))
            self.arc_camera_controller.set_yaw_pitch(-yaw, -pitch)
            self.arc_camera_controller.set_center(actor.pose.p)
            self.arc_camera_controller.set_zoom(np.linalg.norm(actor.pose.p - pos))
        else:
            rot = self.window.get_camera_rotation()
            x, y, z = rotate_vector([0, 0, -1], rot)
            yaw = -np.arctan2(y, x)
            pitch = np.arctan2(z, np.linalg.norm([x, y]))
            self.fps_camera_controller.setXYZ(*self.window.get_camera_position())
            self.fps_camera_controller.setRPY(0, pitch, yaw)

    def select_entity(self, entity):
        self.enter_mode("normal")
        self.key_stack = ""
        if entity:
            if self.selected_entity and isinstance(self.selected_entity, ActorBase):
                for v in self.selected_entity.get_visual_bodies():
                    v.set_visibility(1)
            self.selected_entity = entity
            if isinstance(self.selected_entity, ActorBase):
                for v in entity.get_visual_bodies():
                    v.set_visibility(self.selection_opacity)
        else:
            if self.selected_entity and isinstance(self.selected_entity, ActorBase):
                for v in self.selected_entity.get_visual_bodies():
                    v.set_visibility(1)
            self.selected_entity = None
            self.update_coordinate_axes()

        self.refresh_ik()
        self.refresh_ik_display_objects()

    def refresh_ik(self):
        if (
            self.selected_entity
            and isinstance(self.selected_entity, LinkBase)
            and self.ik_enabled
        ):
            self.pinocchio_model = (
                self.selected_entity.get_articulation().create_pinocchio_model()
            )
            self.gizmo.Matrix(self.selected_entity.pose.to_transformation_matrix())
            self.move_group_joints = [
                j.name
                for j in self.selected_entity.get_articulation().get_joints()
                if j.get_dof() != 0
            ]
            for n in self.move_group_joints:
                if n not in self.move_group_selection:
                    self.move_group_selection[n] = True

            self.move_group.remove_children()

            def select_move_group(name, select):
                self.move_group_selection[name] = select

            for n in self.move_group_joints:
                self.move_group.append(
                    R.UICheckbox()
                    .Label(n)
                    .Checked(self.move_group_selection[n])
                    .Callback((lambda n: lambda c: select_move_group(n, c.checked))(n))
                )
        else:
            self.pinocchio_model = None
            self.move_group.remove_children()

    def clear_ik_display_objects(self):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene
        for obj in self.ik_display_objects:
            render_scene.remove_node(obj)
        self.ik_display_objects = []

    def refresh_ik_display_objects(self):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        self.clear_ik_display_objects()
        if (
            self.selected_entity is not None
            and self.ik_enabled
            and isinstance(self.selected_entity, LinkBase)
        ):
            for link in self.selected_entity.get_articulation().get_links():
                link2world = link.pose
                display_obj = render_scene.add_node()
                for body in link.get_visual_bodies():
                    for obj in body._internal_objects:
                        scale = obj.scale
                        obj2world = Pose(obj.position, obj.rotation)
                        obj2selected = link2world.inv() * obj2world
                        new_obj = render_scene.add_object(obj.model, display_obj)
                        new_obj.set_position(obj2selected.p)
                        new_obj.set_rotation(obj2selected.q)
                        new_obj.set_scale(scale)
                        new_obj.transparency = 0.1
                display_obj.set_position(link.pose.p)
                display_obj.set_rotation(link.pose.q)
                self.ik_display_objects.append(display_obj)

    def update_ik_display_objects(self):
        if (
            self.selected_entity is not None
            and self.ik_enabled
            and isinstance(self.selected_entity, LinkBase)
        ):
            link_idx = (
                self.selected_entity.get_articulation()
                .get_links()
                .index(self.selected_entity)
            )
            pose = Pose.from_transformation_matrix(self.gizmo.matrix)
            self.ik_display_objects[link_idx].set_position(pose.p)
            self.ik_display_objects[link_idx].set_rotation(pose.q)

            result, success, error = self.compute_ik()
            self.ik_result = result
            self.ik_success = success
            self.ik_errpr = error
            self.pinocchio_model.compute_forward_kinematics(result)
            for idx, obj in enumerate(self.ik_display_objects):
                pose = (
                    self.selected_entity.get_articulation().pose
                    * self.pinocchio_model.get_link_pose(idx)
                )
                obj.set_position(pose.p)
                obj.set_rotation(pose.q)

    def compute_ik(self):
        if (
            self.selected_entity is not None
            and self.ik_enabled
            and isinstance(self.selected_entity, LinkBase)
        ):
            a = self.selected_entity.get_articulation()
            link_idx = a.get_links().index(self.selected_entity)
            mask = np.array(
                [self.move_group_selection[j] for j in self.move_group_joints]
            ).astype(int)

            pose = a.pose.inv() * Pose.from_transformation_matrix(self.gizmo.matrix)
            result, success, error = self.pinocchio_model.compute_inverse_kinematics(
                link_idx,
                pose,
                initial_qpos=a.get_qpos(),
                active_qmask=mask,
                max_iterations=100,
            )
            return result, success, error

    def execute_ik(self, _):
        if (
            self.selected_entity is not None
            and self.ik_enabled
            and isinstance(self.selected_entity, LinkBase)
        ):
            if hasattr(self, "ik_result"):
                mask = np.array(
                    [self.move_group_selection[j] for j in self.move_group_joints]
                ).astype(int)
                a = self.selected_entity.get_articulation()
                target = a.get_drive_target()
                target = self.ik_result * mask + target * (1 - mask)
                # self.selected_entity.get_articulation().set_qpos(target)
                self.selected_entity.get_articulation().set_drive_target(target)

    @staticmethod
    def get_camera_pose(camera: CameraEntity):
        """Get the camera pose in the Sapien world."""
        opengl_pose = camera.get_model_matrix()  # opengl camera-> sapien world
        # sapien camera -> opengl camera
        sapien2opengl = np.array(
            [
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [-1, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        cam_pose = Pose.from_transformation_matrix(opengl_pose @ sapien2opengl)
        return cam_pose

    def focus_camera(self, camera: CameraEntity):
        if self.focused_camera == camera:
            return

        self.focused_camera = camera
        if self.focused_camera is not None:
            self.focus_entity(None)

        if self.camera_ui is not None:
            # Lazy check if any camera has changed
            assert self.cameras == self.scene.get_cameras(), "Cameras have changed"
            index = (self.cameras.index(camera) + 1) if camera is not None else 0
            self.camera_ui.Index(index)

    def update_coordinate_axes_scale(self, scale):
        self.axes_scale = scale
        self.update_coordinate_axes()

    def update_coordinate_axes(self):
        if self.selected_entity:
            self.coordinate_axes.set_scale([self.axes_scale] * 3)
            if self.coordinate_axes_mode == "Origin":
                self.coordinate_axes.set_position(self.selected_entity.pose.p)
                self.coordinate_axes.set_rotation(self.selected_entity.pose.q)
            elif self.coordinate_axes_mode == "Center of Mass":
                if isinstance(
                    self.selected_entity, ActorBase
                ) and self.selected_entity.classname in [
                    "Actor",
                    "Link",
                    "KinematicLink",
                ]:
                    pose = (
                        self.selected_entity.pose
                        * self.selected_entity.cmass_local_pose
                    )
                else:
                    pose = self.selected_entity.pose
                self.coordinate_axes.set_position(pose.p)
                self.coordinate_axes.set_rotation(pose.q)
        else:
            self.coordinate_axes.set_position([0, 0, 0])
            self.coordinate_axes.set_rotation([1, 0, 0, 0])
            self.coordinate_axes.set_scale([self.axes_scale] * 3)

    def update_joint_axis(self):
        if (
            self.selected_entity
            and isinstance(self.selected_entity, ActorBase)
            and "Link" in self.selected_entity.classname
        ):
            link: LinkBase = self.selected_entity
            j = link.get_articulation().get_joints()[link.get_index()]
            if j.type not in ["revolute", "prismatic"]:
                for x in self.joint_axes:
                    x.transparency = 1

            j2c = j.get_pose_in_child()
            c2w = link.get_pose()
            j2w = c2w * j2c
            if j.type == "prismatic":
                j2w.set_p(c2w.p)
                self.joint_axes[1].set_position(j2w.p)
                self.joint_axes[1].set_rotation(j2w.q)
                self.joint_axes[1].transparency = 0 if self.show_axes else 1
                self.joint_axes[0].transparency = 1
            elif j.type == "revolute":
                self.joint_axes[0].set_position(j2w.p)
                self.joint_axes[0].set_rotation(j2w.q)
                self.joint_axes[0].transparency = 0 if self.show_axes else 1
                self.joint_axes[1].transparency = 1
        else:
            self.joint_axes[0].transparency = 1
            self.joint_axes[1].transparency = 1

    def reset_windows(self):
        self.control_window = None
        self.scene_window = None
        self.actor_window = None
        self.articulation_window = None
        self.info_window = None
        self.ik_window = None

    def render(self):
        if self.closed:
            return

        self.set_fovy(self.fovy)

        while True:
            if not self.paused or not self.do_not_update_render:
                self.scene.update_render()

            self.build_control_window()
            self.build_scene_window()
            self.build_actor_window()
            self.build_articulation_window()
            self.build_info_window()
            self.build_ik_window()

            proj = self.window.get_camera_projection_matrix()
            view = (
                Pose(
                    self.window.get_camera_position(), self.window.get_camera_rotation()
                )
                .inv()
                .to_transformation_matrix()
            )
            self.gizmo.CameraMatrices(view, proj)

            self.window.render(
                self.target_name,
                [
                    self.control_window,
                    self.scene_window,
                    self.actor_window,
                    self.articulation_window,
                    self.info_window,
                    self.ik_window,
                ],
            )

            if self.mode == "grab":
                if self.window.mouse_click(0) or self.immediate_mode:
                    new_pose = Pose(
                        self.display_object.position, self.display_object.rotation
                    )
                    if isinstance(self.selected_entity, LightEntity):
                        self.selected_entity.set_pose(new_pose)
                    elif self.selected_entity.classname == "Actor":
                        self.selected_entity.set_pose(new_pose)
                    elif self.selected_entity.classname in ["Link", "KinematicLink"]:
                        old_pose = self.selected_entity.pose
                        a = self.selected_entity.get_articulation()
                        a.set_root_pose(new_pose * old_pose.inv() * a.get_root_pose())

                if self.window.mouse_click(0):
                    self.enter_mode("normal")

            elif self.mode == "rotate":
                if self.window.mouse_click(0) or self.immediate_mode:
                    new_pose = Pose(
                        self.display_object.position, self.display_object.rotation
                    )
                    if isinstance(self.selected_entity, LightEntity):
                        self.selected_entity.set_pose(new_pose)
                    if self.selected_entity.classname == "Actor":
                        self.selected_entity.set_pose(new_pose)
                    elif self.selected_entity.classname in ["Link", "KinematicLink"]:
                        old_pose = self.selected_entity.pose
                        a = self.selected_entity.get_articulation()
                        a.set_root_pose(new_pose * old_pose.inv() * a.get_root_pose())

                if self.window.mouse_click(0):
                    self.enter_mode("normal")

            elif self.mode == "normal":
                if self.window.mouse_click(0):
                    mx, my = self.window.mouse_position
                    if not self.is_mouse_available(mx, my):
                        print("[W] Mouse not available")
                        continue

                    ww, wh = self.window.size
                    tw, th = self.window.get_target_size("Segmentation")
                    mx = mx * tw / ww
                    my = my * th / wh
                    pixel = self.window.get_uint32_texture_pixel(
                        "Segmentation", int(mx), int(my)
                    )
                    actor = self.find_actor(pixel[1])
                    self.select_entity(actor)

            self.update_coordinate_axes()
            self.update_joint_axis()
            self.update_ik_display_objects()

            if self._show_camera_linesets:
                self._update_camera_linesets()

            speed_mod = 1
            if self.window.shift:
                speed_mod *= 0.1

            if self.window.key_press("esc"):
                self.enter_mode("normal")

            for key in "frgxyz":
                if self.window.key_press(key):
                    self.key_press_action(key)

            for key in "wasd":
                if self.window.key_down(key):
                    self.key_down_action(key)

            if self.mode == "grab":
                x, y = self.window.mouse_delta
                vec = self.camera_space_to_world_space(np.array([x, -y, 0]))
                if self.grab_axis is not None:
                    vec = vec @ self.grab_axis * self.grab_axis
                elif self.grab_plane is not None:
                    v = self.world_space_to_camera_space(self.grab_plane)
                    vec = self.camera_space_to_world_space(
                        [x, -y, (v[0] * x + v[1] * (-y)) / (-v[2])]
                    )
                self.display_object.set_position(
                    self.display_object.position + vec * 0.01
                )
                self.coordinate_axes.set_position(self.display_object.position)
                self.coordinate_axes.set_rotation(self.display_object.rotation)

            if self.mode == "rotate":
                mouse_position = np.array(self.window.mouse_position)
                dir1 = self.rotate_initial_mouse_position - self.rotate_screen_center
                dir2 = mouse_position - self.rotate_screen_center

                angle = (
                    self.rotate_direction
                    * np.sign(np.cross(dir1, dir2))
                    * np.arccos((dir1 @ dir2) / ((dir1 @ dir1) * (dir2 @ dir2)) ** 0.5)
                )

                self.display_object.set_rotation(
                    qmult(aa(self.rotate_axis, angle), self.rotate_initial_rotation)
                )
                self.coordinate_axes.set_position(self.display_object.position)
                self.coordinate_axes.set_rotation(self.display_object.rotation)

            if self.mode == "normal":
                if self.window.mouse_down(1):
                    x, y = self.window.mouse_delta
                    if self.focused_entity:
                        self.arc_camera_controller.rotate_yaw_pitch(
                            -self.rotate_speed * speed_mod * x,
                            self.rotate_speed * speed_mod * y,
                        )
                    else:
                        self.fps_camera_controller.rotate(
                            0,
                            -self.rotate_speed * speed_mod * y,
                            self.rotate_speed * speed_mod * x,
                        )

                if self.window.mouse_down(2):
                    x, y = self.window.mouse_delta
                    self.focus_entity(None)
                    self.fps_camera_controller.move(
                        0,
                        self.rotate_speed * speed_mod * x,
                        self.rotate_speed * speed_mod * y,
                    )

                wx, wy = self.window.mouse_wheel_delta
                if wx != 0:
                    if self.focused_entity:
                        self.arc_camera_controller.zoom(
                            self.scroll_speed * speed_mod * wx
                        )
                    else:
                        self.fps_camera_controller.move(
                            self.scroll_speed * speed_mod * wx, 0, 0
                        )

                if self.focused_entity:
                    self.arc_camera_controller.set_center(self.focused_entity.pose.p)
                elif self.focused_camera:
                    cam_pose = self.get_camera_pose(self.focused_camera)
                    rpy = quat2euler(cam_pose.q)
                    self.set_camera_xyz(*cam_pose.p)
                    self.set_camera_rpy(rpy[0], -rpy[1], -rpy[2])

            if self.window.key_down("q") or self.window.should_close:
                self.close()
                return

            if not self.paused or (self.paused and self.single_step):
                self.single_step = False
                break

    def is_mouse_available(self, mx, my):
        w, h = self.window.size
        # print(f"[I] windowSize: {w, h}; mousePose: {mx, my}")
        return mx >= 0 and my >= 0

    def camera_space_to_world_space(self, vec):
        return rotate_vector(vec, self.window.get_camera_rotation())

    def world_space_to_camera_space(self, vec):
        return rotate_vector(vec, qinverse(self.window.get_camera_rotation()))

    def world_space_to_screen_space(self, point):
        q = qinverse(self.window.get_camera_rotation())
        point = rotate_vector(point, q) - rotate_vector(
            self.window.get_camera_position(), q
        )
        proj = self.window.get_camera_projection_matrix()
        point = proj @ np.concatenate((point, [1]))
        point = point[:3] / point[3]
        point[:2] = (point[:2] * 0.5 + 0.5) * np.array(self.window.size)
        return point

    def screen_space_to_world_space(self, point):
        proj = self.window.get_camera_projection_matrix()
        point[:2] = (point[:2] / np.array(self.window.size)) * 2 - 1
        point = np.linalg.inv(proj) @ np.concatenate((point, [1]))
        point = point[:3] / point[3]  # screen space
        q = self.window.get_camera_rotation()
        point = rotate_vector(point, q) + self.window.get_camera_position()
        return point
