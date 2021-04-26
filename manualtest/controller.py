import sapien.core.pysapien.renderer as R
from sapien.core import (
    Pose,
    VulkanRenderer,
    Scene,
    VulkanWindow,
    ArticulationBase,
    Joint,
    LinkBase,
    VulkanCamera,
)
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.euler import quat2euler
from transforms3d.quaternions import qmult, mat2quat, rotate_vector, qinverse
import numpy as np
import os

imgui_ini = '''
[Window][DockSpace Demo]
Pos=0,0
Size=1024,768
Collapsed=0

[Window][Actor]
Pos=807,23
Size=217,389
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
Pos=807,414
Size=217,293
Collapsed=0
DockId=0x00000008,0

[Window][Info]
Pos=0,709
Size=1024,59
Collapsed=0
DockId=0x0000000A,0

[Docking][Data]
DockSpace         ID=0x4BBE4C7A Window=0x4647B76E Pos=0,23 Size=1024,745 Split=Y
  DockNode        ID=0x00000009 Parent=0x4BBE4C7A SizeRef=1024,684 Split=X
    DockNode      ID=0x00000005 Parent=0x00000009 SizeRef=805,747 Split=X
      DockNode    ID=0x00000001 Parent=0x00000005 SizeRef=248,747 Split=Y Selected=0x9A68760C
        DockNode  ID=0x00000003 Parent=0x00000001 SizeRef=399,368 Selected=0x226615D7
        DockNode  ID=0x00000004 Parent=0x00000001 SizeRef=399,314 Selected=0x9A68760C
      DockNode    ID=0x00000002 Parent=0x00000005 SizeRef=555,747 CentralNode=1
    DockNode      ID=0x00000006 Parent=0x00000009 SizeRef=217,747 Split=Y Selected=0x85B479FD
      DockNode    ID=0x00000007 Parent=0x00000006 SizeRef=121,389 Selected=0x85B479FD
      DockNode    ID=0x00000008 Parent=0x00000006 SizeRef=121,293 Selected=0xA95BF184
  DockNode        ID=0x0000000A Parent=0x4BBE4C7A SizeRef=1024,59 Selected=0x6BBB9E69
'''


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
        renderer: VulkanRenderer,
        shader_dir="",
        resolutions=((1024, 768), (800, 600), (1920, 1080)),
    ):
        if not os.path.exists('imgui.ini'):
            with open('imgui.ini', 'w') as f:
                f.write(imgui_ini)

        self.shader_dir = shader_dir
        self.renderer = renderer
        self.renderer_context: R.Context = renderer._internal_context

        self.create_visual_models()

        self.window = None
        self.resolution = None
        self.resolutions = None
        self.set_window_resolutions(resolutions)

        self.axes = None
        self.axes_scale = 0.3

        self.selected_actor = None
        self.focused_actor = None
        self.paused = False
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

        self.key_stack = ""
        self.initialize_key_action_map()

        self.mode = "normal"

        self.display_object = None
        self.coordinate_axes_mode = "Origin"

    def create_visual_models(self):
        self.cone = self.renderer_context.create_cone_mesh(16)
        self.capsule = self.renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)

        self.mat_red = self.renderer_context.create_material([1, 0, 0, 1], 0, 0, 0)
        self.mat_green = self.renderer_context.create_material([0, 1, 0, 1], 0, 0, 0)
        self.mat_blue = self.renderer_context.create_material([0, 0, 1, 1], 0, 0, 0)

        self.mat_cyan = self.renderer_context.create_material([0, 1, 1, 1], 0, 0, 0)
        self.mat_magenta = self.renderer_context.create_material([1, 0, 1, 1], 0, 0, 0)

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

        self.grab_objects = [
            render_scene.add_object(model)
            for model in [self.red_capsule, self.green_capsule, self.blue_capsule]
        ]

        for obj in self.grab_objects:
            obj.set_position([0, 0, 0])
            obj.set_scale([100, 0.1, 0.1])
            obj.shading_mode = 2
            obj.cast_shadow = False
            obj.transparency = 1

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
            for obj in self.grab_objects:
                obj.transparency = 1
            if self.display_object:
                rs = self.scene.renderer_scene
                render_scene: R.Scene = rs._internal_scene
                render_scene.remove_node(self.display_object)
                self.display_object = None
        elif name == "rotate":
            for obj in self.grab_objects:
                obj.transparency = 1
            if self.display_object:
                rs = self.scene.renderer_scene
                render_scene: R.Scene = rs._internal_scene
                render_scene.remove_node(self.display_object)
                self.display_object = None

    def add_display_object(self, actor):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene
        if self.display_object:
            render_scene.remove_node(self.display_object)
            self.display_object = None
        self.display_object = render_scene.add_node()
        selected2world = self.selected_actor.pose
        for body in self.selected_actor.get_visual_bodies():
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
            if self.selected_actor:
                self.focus_actor(self.selected_actor)

        def r():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 1
            self.add_display_object(self.select_actor)

            point = self.world_space_to_screen_space(self.selected_actor.pose.p)
            axis = self.screen_space_to_world_space(
                [point[0], point[1], 1]
            ) - self.screen_space_to_world_space([point[0], point[1], 0])
            axis = axis / np.linalg.norm(axis)
            self.rotate_axis = np.array(axis)
            self.rotate_direction = 1
            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def rx():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 1
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation([1, 0, 0, 0])
            self.add_display_object(self.select_actor)
            self.rotate_axis = np.array([1, 0, 0])

            screen_vector = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_actor.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def rxx():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 1
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation(self.selected_actor.pose.q)
            self.add_display_object(self.select_actor)
            self.rotate_axis = np.array(
                rotate_vector([1, 0, 0], self.selected_actor.pose.q)
            )

            screen_vector = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_actor.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def rxxx():
            self.key_stack = "r"
            r()

        def ry():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 1
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(x2y)
            self.add_display_object(self.select_actor)
            self.rotate_axis = np.array([0, 1, 0])

            screen_vector = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_actor.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def ryy():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 1
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(qmult(self.selected_actor.pose.q, x2y))
            self.add_display_object(self.select_actor)
            self.rotate_axis = np.array(
                rotate_vector([0, 1, 0], self.selected_actor.pose.q)
            )

            screen_vector = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_actor.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def ryyy():
            self.key_stack = "r"
            r()

        def rz():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 0
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(x2z)
            self.add_display_object(self.select_actor)
            self.rotate_axis = np.array([0, 0, 1])

            screen_vector = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_actor.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def rzz():
            if not self.selected_actor:
                self.key_stack = ""
                return

            self.enter_mode("rotate")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 0
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(qmult(self.selected_actor.pose.q, x2z))
            self.add_display_object(self.select_actor)
            self.rotate_axis = np.array(
                rotate_vector([0, 0, 1], self.selected_actor.pose.q)
            )

            screen_vector = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            ) - self.world_space_to_screen_space(
                self.selected_actor.pose.p + self.rotate_axis
            )
            if screen_vector[2] <= 0:
                self.rotate_direction = 1
            else:
                self.rotate_direction = -1

            self.rotate_initial_mouse_position = np.array(self.window.mouse_position)
            self.rotate_initial_rotation = np.array(self.selected_actor.pose.q)
            self.rotate_screen_center = self.world_space_to_screen_space(
                self.selected_actor.pose.p
            )[:2]

        def rzzz():
            self.key_stack = "r"
            r()

        def g():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 1
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = None

        def gx():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 1
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation([1, 0, 0, 0])
            self.add_display_object(self.select_actor)
            self.grab_axis = np.array([1, 0, 0])
            self.grab_plane = None

        def gxx():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 1
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation(self.selected_actor.pose.q)
            self.add_display_object(self.select_actor)
            self.grab_axis = rotate_vector([1, 0, 0], self.selected_actor.pose.q)
            self.grab_plane = None

        def gxxx():
            self.key_stack = "g"
            g()

        def gX():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 0
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(x2y)
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(x2z)
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = np.array([1, 0, 0])

        def gXX():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 0
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(qmult(self.selected_actor.pose.q, x2y))
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(qmult(self.selected_actor.pose.q, x2z))
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = rotate_vector(
                np.array([1, 0, 0]), self.selected_actor.pose.q
            )

        def gXXX():
            self.key_stack = "g"
            g()

        def gy():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 1
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(x2y)
            self.add_display_object(self.select_actor)
            self.grab_axis = np.array([0, 1, 0])
            self.grab_plane = None

        def gyy():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 1
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(qmult(self.selected_actor.pose.q, x2y))
            self.add_display_object(self.select_actor)
            self.grab_axis = rotate_vector([0, 1, 0], self.selected_actor.pose.q)
            self.grab_plane = None

        def gyyy():
            self.key_stack = "g"
            g()

        def gY():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 0
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation([1, 0, 0, 0])
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(x2z)
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = np.array([0, 1, 0])

        def gYY():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 0
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation(
                qmult(self.selected_actor.pose.q, [1, 0, 0, 0])
            )
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(qmult(self.selected_actor.pose.q, x2z))
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = rotate_vector(
                np.array([0, 1, 0]), self.selected_actor.pose.q
            )

        def gYYY():
            self.key_stack = "g"
            g()

        def gz():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 0
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(x2z)
            self.add_display_object(self.select_actor)
            self.grab_axis = np.array([0, 0, 1])
            self.grab_plane = None

        def gzz():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 1
            self.grab_objects[1].transparency = 1
            self.grab_objects[2].transparency = 0
            self.grab_objects[2].set_position(self.selected_actor.pose.p)
            self.grab_objects[2].set_rotation(qmult(self.selected_actor.pose.q, x2z))
            self.add_display_object(self.select_actor)
            self.grab_axis = rotate_vector([0, 0, 1], self.selected_actor.pose.q)
            self.grab_plane = None

        def gzzz():
            self.key_stack = "g"
            g()

        def gZ():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 1
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation([1, 0, 0, 0])
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(x2y)
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = np.array([0, 0, 1])

        def gZZ():
            if not self.selected_actor:
                self.key_stack = ""
                return
            self.enter_mode("grab")
            self.grab_objects[0].transparency = 0
            self.grab_objects[1].transparency = 0
            self.grab_objects[2].transparency = 1
            self.grab_objects[0].set_position(self.selected_actor.pose.p)
            self.grab_objects[0].set_rotation(
                qmult(self.selected_actor.pose.q, [1, 0, 0, 0])
            )
            self.grab_objects[1].set_position(self.selected_actor.pose.p)
            self.grab_objects[1].set_rotation(qmult(self.selected_actor.pose.q, x2y))
            self.add_display_object(self.select_actor)
            self.grab_axis = None
            self.grab_plane = rotate_vector(
                np.array([0, 0, 1]), self.selected_actor.pose.q
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
            self.focus_actor(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(self.move_speed * speed_mod, 0, 0)
            self.key_stack = ""

        def s():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_actor(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(-self.move_speed * speed_mod, 0, 0)
            self.key_stack = ""

        def a():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_actor(None)
            self.focus_camera(None)
            self.fps_camera_controller.move(0, self.move_speed * speed_mod, 0)
            self.key_stack = ""

        def d():
            self.enter_mode("normal")
            speed_mod = 0.1 if self.window.shift else 1
            self.focus_actor(None)
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

    def set_window_resolutions(self, resolutions):
        assert len(resolutions)
        for r in resolutions:
            assert len(r) == 2

        self.window = self.renderer.create_window(
            resolutions[0][0], resolutions[0][1], self.shader_dir
        )
        self.resolution = resolutions[0]
        self.resolutions = resolutions

    def build_control_window(self):
        if not self.control_window:
            self.cameras = self.scene.get_mounted_cameras()
            self.camera_ui = (
                R.UIOptions().Style("select").Label("Name##camera_name")
                .Index(0 if self.focused_camera is None else self.cameras.index(self.focused_camera) + 1)
                .Items(['None'] + [x.get_name() for x in self.cameras])
                .Callback(lambda p: self.focus_camera(self.cameras[p.index - 1] if p.index > 0 else None))
            )

            self.control_window = (
                R.UIWindow()
                .Label("Control")
                .Pos(10, 10)
                .Size(400, 400)
                .append(
                    R.UISameLine().append(
                        R.UICheckbox()
                        .Label("Pause")
                        .Callback(lambda p: self.toggle_pause(p.checked)),
                        R.UIButton()
                        .Label("Single Step")
                        .Callback(lambda p: self.step_button()),
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
                    R.UIDisplayText().Text("Display Settings"),
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
                        + [x for x in self.window.target_names if x != "Color"]
                    )
                    .Callback(lambda p: self.set_target(p.value)),
                    R.UIOptions()
                    .Style("select")
                    .Label("Resolution")
                    .Index(0)
                    .Items(["{}x{}".format(r[0], r[1]) for r in self.resolutions])
                    .Callback(lambda p: self.set_resolution(p.index)),
                    R.UIDisplayText().Text("Actor Selection"),
                    R.UICheckbox()
                    .Label("Coordinate Axes")
                    .Checked(True)
                    .Callback(lambda p: self.toggle_axes(p.checked)),
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
                    R.UIDisplayText().Text("FPS: {:.2f}".format(self.window.fps)),
                )
            )
        self.control_window.get_children()[-1].Text(
            "FPS: {:.2f}".format(self.window.fps)
        )

    def set_selection_opacity(self, opacity):
        self.selection_opacity = opacity

    def set_resolution(self, index):
        self.resolution = self.resolutions[index]
        self.window.resize(*self.resolution)

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
                .Callback((lambda link: lambda _: self.select_actor(link))(actor))
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
        )
        self.actor_window.append(
            R.UIDisplayText().Text("Position"),
            R.UIInputFloat().Label("x##actorpx").Value(actor.pose.p[0]).ReadOnly(True),
            R.UIInputFloat().Label("y##actorpy").Value(actor.pose.p[1]).ReadOnly(True),
            R.UIInputFloat().Label("z##actorpz").Value(actor.pose.p[2]).ReadOnly(True),
            R.UIDisplayText().Text("Rotation"),
            R.UIInputFloat().Label("w##actorqw").Value(actor.pose.q[0]).ReadOnly(True),
            R.UIInputFloat().Label("x##actorqx").Value(actor.pose.q[1]).ReadOnly(True),
            R.UIInputFloat().Label("y##actorqy").Value(actor.pose.q[2]).ReadOnly(True),
            R.UIInputFloat().Label("z##actorqz").Value(actor.pose.q[3]).ReadOnly(True),
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
                        "Dynamic friction: {:.3g}".format(mat.get_dynamic_friction())
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
                        "Capsule half length: {:.3g}".format(shape.geometry.half_length)
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

    def build_articulation_window(self):
        self.articulation_window = R.UIWindow().Label("Articulation")
        if not self.selected_actor or self.selected_actor.type not in [
            "link",
            "kinematic_link",
        ]:
            self.articulation_window.append(
                R.UIDisplayText().Text("No articulation selected.")
            )
            return

        art = self.selected_actor.get_articulation()
        art: ArticulationBase
        self.articulation_window.append(
            R.UIDisplayText().Text(
                "Name: {}".format(art.name if art.name else "(no name)")
            ),
            R.UIDisplayText().Text("Type: {}".format(art.type)),
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
            if art.type == "dynamic":
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
                                    j.stiffness, p.value, j.force_limit, j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Stiffness##{}".format(i))
                        .Value(j.stiffness)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    p.value, j.damping, j.force_limit, j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Force Limit##{}".format(i))
                        .Value(j.force_limit)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness, j.damping, p.value, j.drive_mode,
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
        self.scene = scene
        self.window.set_scene(scene)
        self.fps_camera_controller = FPSCameraController(self.window)
        self.arc_camera_controller = ArcRotateCameraController(self.window)
        self.create_visual_objects()
        self.toggle_axes(True)
        self.set_fovy(np.pi/2)

    def set_camera_xyz(self, x, y, z):
        self.fps_camera_controller.setXYZ(x, y, z)

    def set_camera_rpy(self, r, p, y):
        self.fps_camera_controller.setRPY(r, p, y)

    def toggle_pause(self, paused):
        self.paused = paused

    def toggle_axes(self, show):
        print("toggle", show)
        for c in self.coordinate_axes.children:
            if show:
                c.transparency = 0
            else:
                c.transparency = 1

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
        self.axes = None
        self.scene = None
        self.fps_camera_controller = None
        self.window = None
        self.control_window = None
        self.scene_window = None
        self.actor_window = None
        self.articulation_window = None
        self.info_window = None

    def focus_actor(self, actor):
        if actor == self.focused_actor:
            return

        self.focused_actor = actor
        if self.focused_actor is not None:
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

    def select_actor(self, actor):
        self.enter_mode("normal")
        self.key_stack = ""
        if actor:
            if self.selected_actor:
                for v in self.selected_actor.get_visual_bodies():
                    v.set_visibility(1)
            self.selected_actor = actor
            for v in self.selected_actor.get_visual_bodies():
                v.set_visibility(self.selection_opacity)
        else:
            if self.selected_actor:
                for v in self.selected_actor.get_visual_bodies():
                    v.set_visibility(1)
            self.selected_actor = None
            self.update_coordinate_axes()

    @staticmethod
    def get_camera_pose(camera: VulkanCamera):
        """Get the camera pose in the Sapien world."""
        opengl_pose = camera.get_model_matrix()  # opengl camera-> sapien world
        # sapien camera -> opengl camera
        sapien2opengl = np.array([[0., -1., 0., 0.],
                                  [0., 0., 1., 0.],
                                  [-1, 0., 0., 0.],
                                  [0., 0., 0., 1.]])
        cam_pose = Pose.from_transformation_matrix(opengl_pose @ sapien2opengl)
        return cam_pose

    def focus_camera(self, camera: VulkanCamera):
        if self.focused_camera == camera:
            return

        self.focused_camera = camera
        if self.focused_camera is not None:
            self.focus_actor(None)

        if self.camera_ui is not None:
            # Lazy check if any camera has changed
            assert self.cameras == self.scene.get_mounted_cameras(), 'Cameras have changed'
            index = (self.cameras.index(camera) + 1) if camera is not None else 0
            self.camera_ui.Index(index)

    def update_coordinate_axes_scale(self, scale):
        self.axes_scale = scale
        self.update_coordinate_axes()

    def update_coordinate_axes(self):
        if self.selected_actor:
            self.coordinate_axes.set_scale([self.axes_scale] * 3)
            if self.coordinate_axes_mode == "Origin":
                self.coordinate_axes.set_position(self.selected_actor.pose.p)
                self.coordinate_axes.set_rotation(self.selected_actor.pose.q)
            elif self.coordinate_axes_mode == "Center of Mass":
                if self.selected_actor.type in [
                    "dynamic",
                    "kinematic",
                    "link",
                    "kinematic_link",
                ]:
                    pose = (
                        self.selected_actor.pose * self.selected_actor.cmass_local_pose
                    )
                else:
                    pose = self.selected_actor.pose
                self.coordinate_axes.set_position(pose.p)
                self.coordinate_axes.set_rotation(pose.q)
        else:
            self.coordinate_axes.set_position([0, 0, 0])
            self.coordinate_axes.set_rotation([1, 0, 0, 0])
            self.coordinate_axes.set_scale([self.axes_scale] * 3)

    def update_joint_axis(self):
        if self.selected_actor and "link" in self.selected_actor.type:
            link: LinkBase = self.selected_actor
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
                self.joint_axes[1].transparency = 0
                self.joint_axes[0].transparency = 1
            elif j.type == "revolute":
                self.joint_axes[0].set_position(j2w.p)
                self.joint_axes[0].set_rotation(j2w.q)
                self.joint_axes[0].transparency = 0
                self.joint_axes[1].transparency = 1
        else:
            self.joint_axes[0].transparency = 1
            self.joint_axes[1].transparency = 1

    def render(self):
        if self.closed:
            return

        while True:
            self.scene.update_render()

            self.build_control_window()
            self.build_scene_window()
            self.build_actor_window()
            self.build_articulation_window()
            self.build_info_window()

            self.window.render(
                self.target_name,
                [
                    self.control_window,
                    self.scene_window,
                    self.actor_window,
                    self.articulation_window,
                    self.info_window,
                ],
            )

            if self.mode == "grab":
                if self.window.mouse_click(0):
                    new_pose = Pose(
                        self.display_object.position, self.display_object.rotation
                    )
                    if self.selected_actor.type in ["dynamic", "kinematic"]:
                        self.selected_actor.set_pose(new_pose)
                    elif self.selected_actor.type in ["link", "kinematic_link"]:
                        old_pose = self.selected_actor.pose
                        a = self.selected_actor.get_articulation()
                        a.set_root_pose(new_pose * old_pose.inv() * a.get_root_pose())

                    self.enter_mode("normal")

            elif self.mode == "rotate":
                if self.window.mouse_click(0):
                    new_pose = Pose(
                        self.display_object.position, self.display_object.rotation
                    )
                    if self.selected_actor.type in ["dynamic", "kinematic"]:
                        self.selected_actor.set_pose(new_pose)
                    elif self.selected_actor.type in ["link", "kinematic_link"]:
                        old_pose = self.selected_actor.pose
                        a = self.selected_actor.get_articulation()
                        a.set_root_pose(new_pose * old_pose.inv() * a.get_root_pose())

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
                    pixel = self.window.download_uint32_target_pixel(
                        "Segmentation", int(mx), int(my)
                    )
                    actor = self.find_actor(pixel[1])
                    self.select_actor(actor)

            self.update_coordinate_axes()
            self.update_joint_axis()

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

            if self.mode == "normal":
                if self.window.mouse_down(1):
                    x, y = self.window.mouse_delta
                    if self.focused_actor:
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
                    self.focus_actor(None)
                    self.fps_camera_controller.move(
                        0,
                        self.rotate_speed * speed_mod * x,
                        self.rotate_speed * speed_mod * y,
                    )

                wx, wy = self.window.mouse_wheel_delta
                if wx != 0:
                    if self.focused_actor:
                        self.arc_camera_controller.zoom(
                            self.scroll_speed * speed_mod * wx
                        )
                    else:
                        self.fps_camera_controller.move(
                            self.scroll_speed * speed_mod * wx, 0, 0
                        )

                if self.focused_actor:
                    self.arc_camera_controller.set_center(self.focused_actor.pose.p)
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
        print(f"[I] windowSize: {w, h}; mousePose: {mx, my}")
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
