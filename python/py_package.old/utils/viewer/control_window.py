from pathlib import Path
from typing import List

import numpy as np
import pkg_resources
import sapien.core as sapien
from sapien.core import renderer as R
from transforms3d.euler import quat2euler
from transforms3d.quaternions import rotate_vector, mat2quat

from .camera_control import ArcRotateCameraController, FPSCameraController
from .plugin import Plugin


class ControlWindow(Plugin):
    def __init__(self):
        self.reset()

    def init(self, viewer):
        super().init(viewer)
        self.viewer.set_camera_xyz = self.set_camera_xyz
        self.viewer.set_camera_rpy = self.set_camera_rpy
        self.viewer.focus_entity = self.focus_entity
        self.viewer.focus_camera = self.focus_camera

        self._setup_shader_dir()
        self._create_visual_models()

    def _setup_shader_dir(self):
        default_shader_dir = Path(self.viewer.shader_dir)

        all_shader_dir = Path(
            pkg_resources.resource_filename("sapien", "vulkan_shader")
        )
        self.shader_list = []
        self.shader_types = []
        for f in all_shader_dir.iterdir():
            if f.is_dir():
                if any("gbuffer.frag" in x.name for x in f.iterdir()):
                    self.shader_list.append(f)
                    self.shader_types.append("rast")
                if any("camera.rgen" in x.name for x in f.iterdir()):
                    self.shader_list.append(f)
                    self.shader_types.append("rt")

        self.shader_list = [default_shader_dir] + self.shader_list
        if "camera.rgen" in default_shader_dir.iterdir():
            self.shader_types = ["rt"] + self.shader_types
        else:
            self.shader_types = ["rast"] + self.shader_types

        self.shader_type = self.shader_types[0]
        self.shader_dir = str(self.shader_list[0])

    @property
    def selected_entity(self):
        return self.viewer.selected_entity

    @property
    def window(self):
        return self.viewer.window

    @property
    def renderer_context(self):
        return self.viewer.renderer_context

    @property
    def show_camera_linesets(self):
        return self._show_camera_linesets

    @show_camera_linesets.setter
    def show_camera_linesets(self, v):
        self._show_camera_linesets = v
        if not v:
            self._clear_camera_linesets()

    @property
    def show_joint_axes(self):
        return self._show_joint_axes

    @show_joint_axes.setter
    def show_joint_axes(self, v):
        self._show_joint_axes = v
        if not v:
            self._clear_joint_axes()

    def _update_joint_axes(self):
        if not self.show_joint_axes:
            return

        if self.joint_axes is None:
            self._create_joint_axes()

        if isinstance(self.selected_entity, sapien.LinkBase):
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
                self.joint_axes[1].transparency = 0 if self.show_joint_axes else 1
                self.joint_axes[0].transparency = 1
            elif j.type == "revolute":
                self.joint_axes[0].set_position(j2w.p)
                self.joint_axes[0].set_rotation(j2w.q)
                self.joint_axes[0].transparency = 0 if self.show_joint_axes else 1
                self.joint_axes[1].transparency = 1
        else:
            self.joint_axes[0].transparency = 1
            self.joint_axes[1].transparency = 1

    def focus_entity(self, entity: sapien.Entity):
        if entity == self.focused_entity:
            return

        self.focused_entity = entity

        cam_pose = self.window.get_camera_pose()
        if self.focused_entity is not None:
            # initialize arc camera
            self.focus_camera(None)
            _, pitch, yaw = quat2euler(cam_pose.q)
            self.arc_camera_controller.set_yaw_pitch(yaw, pitch)
            self.arc_camera_controller.set_center(entity.pose.p)
            self.arc_camera_controller.set_zoom(
                np.linalg.norm(entity.pose.p - cam_pose.p)
            )
            self.viewer.set_camera_pose(self.arc_camera_controller.pose)
        else:
            # switch to fps camera
            self.fps_camera_controller.setXYZ(*cam_pose.p)
            r, p, y = quat2euler(cam_pose.q)
            self.fps_camera_controller.setRPY(r, -p, -y)
            self.viewer.set_camera_pose(self.fps_camera_controller.pose)

    def focus_camera(self, camera: sapien.CameraEntity):
        if self.focus_camera == camera:
            return

        self.focused_camera = camera
        if self.focused_camera is not None:
            self.focus_entity(None)

    def set_shader(self, index):
        self.shader_dir = str(self.shader_list[index])
        self.shader_type = str(self.shader_types[index])
        self.window.set_shader_dir(self.shader_dir)
        self.viewer.render_target = "Color"
        self._denoiser = False

    @property
    def is_rt(self):
        return getattr(self, "shader_type", None) == "rt"

    @property
    def denoiser(self):
        return self._denoiser

    @denoiser.setter
    def denoiser(self, enable):
        self._denoiser = enable
        self.window.set_camera_property("_denoiser", enable)

    @property
    def scene(self):
        return self.viewer.scene

    def notify_scene_change(self):
        if self.viewer.scene is None:
            self.reset()

    @property
    def camera_items(self):
        return ["None"] + [c.name for i, c in enumerate(self.scene.get_cameras())]

    @property
    def camera_index(self):
        return self._camera_index

    @camera_index.setter
    def camera_index(self, i):
        self._camera_index = i
        if i == 0:
            self.focus_camera(None)
        else:
            self.focus_camera(self.scene.get_cameras()[i - 1])

    def single_step(self, _):
        self._single_step = True

    def copy_camera_settings(self, _=None):
        p = self.window.get_camera_position()
        q = self.window.get_camera_rotation()
        width, height = self.window.size
        fovy = self.window.fovy
        near = self.window.near
        far = self.window.far
        import pyperclip

        pyperclip.copy(
            f'camera = add_camera(name="", width={width}, height={height}, fovy={fovy:.3g}, near={near:.3g}, far={far:.3g})\n'
            f"camera.set_local_pose({sapien.Pose(p, q).__repr__()})"
        )

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.ui_camera = (
                R.UIOptions()
                .Style("select")
                .Label("Name")
                .Id("camera")
                .BindItems(self, "camera_items")
                .BindIndex(self, "camera_index")
            )

            self.ui_pause_checkbox = (
                R.UICheckbox().Label("Pause").Bind(self.viewer, "paused")
            )

            self.ui_window = R.UIWindow().Label("Control").Pos(10, 10).Size(400, 400)

            self.ui_window.append(
                R.UISameLine().append(
                    self.ui_pause_checkbox,
                    R.UIButton().Label("Single Step").Callback(self.single_step),
                ),
                R.UISection()
                .Label("Movement Speed")
                .Expanded(True)
                .append(
                    R.UISliderFloat()
                    .Label("Move")
                    .Min(0.01)
                    .Max(1)
                    .Bind(self, "move_speed"),
                    R.UISliderFloat()
                    .Label("Rotate")
                    .Min(0.001)
                    .Max(0.01)
                    .Bind(self, "rotate_speed"),
                    R.UISliderFloat()
                    .Label("Scroll")
                    .Min(0.1)
                    .Max(1)
                    .Bind(self, "scroll_speed"),
                ),
                R.UISection()
                .Label("Camera")
                .Expanded(True)
                .append(
                    self.ui_camera,
                    R.UISliderAngle().Label("FOV Y").Min(1).Max(179).Bind(self, "fovy"),
                    R.UIButton()
                    .Label("Copy Camera Settings")
                    .Callback(self.copy_camera_settings),
                ),
                R.UISection()
                .Label("Display")
                .Expanded(True)
                .append(
                    R.UIInputInt2().Label("Resolution").Bind(self.viewer, "resolution"),
                    R.UIOptions()
                    .Label("Shader")
                    .Style("select")
                    .Items([d.name for i, d in enumerate(self.shader_list)])
                    .Callback(lambda p: self.set_shader(p.index)),
                    R.UIConditional()
                    .Bind(self, "is_rt")
                    .append(R.UICheckbox().Label("Denoiser").Bind(self, "denoiser")),
                    R.UIOptions()
                    .Label("Target")
                    .Style("select")
                    .BindItems(self.window, "display_target_names")
                    .BindIndex(self, "display_target_index"),
                    R.UICheckbox()
                    .Label("Show Cameras in Viewport")
                    .Bind(self, "show_camera_linesets"),
                    R.UICheckbox()
                    .Label("Show Joint Axes")
                    .Bind(self, "show_joint_axes"),
                ),
                R.UISection()
                .Label("Selection")
                .Expanded(True)
                .append(
                    R.UISliderFloat()
                    .Label("Opacity")
                    .Min(0)
                    .Max(1)
                    .Bind(self.viewer, "selected_entity_visibility"),
                    R.UIDisplayText().Bind(
                        lambda: "Selected: "
                        + (
                            self.selected_entity.name
                            if self.selected_entity
                            else "(none)"
                        )
                    ),
                    R.UIDisplayText().Bind(
                        lambda: "Focused: "
                        + (
                            self.focused_entity.name
                            if self.focused_entity
                            else "(none)"
                        )
                    ),
                ),
                R.UIDisplayText().Bind(lambda: "FPS: {:.2f}".format(self.window.fps)),
            )

    def set_camera_xyz(self, x, y, z):
        self.fps_camera_controller.setXYZ(x, y, z)
        self.viewer.set_camera_pose(self.fps_camera_controller.pose)

    def set_camera_rpy(self, r, p, y):
        self.fps_camera_controller.setRPY(r, p, y)
        self.viewer.set_camera_pose(self.fps_camera_controller.pose)

    # override
    def before_render(self):
        pass

    # override
    def after_render(self):
        if self._single_step and self.viewer.paused:
            self.viewer.paused = False
        elif self._single_step and not self.viewer.paused:
            self._single_step = False
            self.viewer.paused = True

        self._handle_focused_camera()
        self._handle_focused_entity()

        self._handle_click()

        self._handle_input_wasd()
        self._handle_input_mouse()
        self._handle_input_f()

        if self.show_camera_linesets:
            self._update_camera_linesets()

        if self.show_joint_axes:
            self._update_joint_axes()

    def _handle_click(self):
        if self.window.mouse_click(0):
            mx, my = self.window.mouse_position
            ww, wh = self.window.size
            if mx < 0 or my < 0 or mx >= ww or my >= wh:
                return
            tw, th = self.window.get_target_size("Segmentation")
            mx = mx * tw / ww
            my = my * th / wh
            pixel = self.window.get_uint32_texture_pixel(
                "Segmentation", int(mx), int(my)
            )

            actor = self.find_actor_by_id(pixel[1])
            self.viewer.select_entity(actor)

    def find_actor_by_id(self, id):
        actors = self.scene.get_all_actors()
        for actor in actors:
            if actor.id == id:
                return actor
        for a in self.scene.get_all_articulations():
            for link in a.get_links():
                if link.id == id:
                    return link

    def _handle_focused_camera(self):
        """
        make camera follow focused camera
        """
        if self.focused_camera is None:
            return
        self.viewer.set_camera_pose(self.focused_camera.pose)

    def _handle_focused_entity(self):
        if self.focused_entity is None:
            return
        self.arc_camera_controller.set_center(self.focused_entity.pose.p)

    def _handle_input_wasd(self):
        """
        camera movement from keyboard
        """
        speed_mod = 1
        if self.window.shift:
            speed_mod = 0.1

        forward = 0
        left = 0
        if self.window.key_down("w"):
            forward += self.move_speed * speed_mod
        if self.window.key_down("s"):
            forward += -self.move_speed * speed_mod
        if self.window.key_down("a"):
            left += self.move_speed * speed_mod
        if self.window.key_down("d"):
            left += -self.move_speed * speed_mod

        moved = forward != 0 or left != 0
        if moved:
            self.focus_entity(None)
            self.fps_camera_controller.move(forward, left, 0)
            self.viewer.set_camera_pose(self.fps_camera_controller.pose)

    def _handle_input_f(self):
        if self.window.key_down("f"):
            self.focus_entity(self.selected_entity)

    def _handle_input_mouse(self):
        """
        camera movement from mouse actions
        """
        speed_mod = 1
        if self.window.shift:
            speed_mod = 0.1

        # right click -> rotate
        if self.window.mouse_down(1):
            x, y = self.window.mouse_delta
            if x != 0 or y != 0:
                if self.focused_entity:
                    self.arc_camera_controller.rotate_yaw_pitch(
                        -self.rotate_speed * speed_mod * x,
                        self.rotate_speed * speed_mod * y,
                    )
                    self.viewer.set_camera_pose(self.arc_camera_controller.pose)
                else:
                    self.fps_camera_controller.rotate(
                        0,
                        -self.rotate_speed * speed_mod * y,
                        self.rotate_speed * speed_mod * x,
                    )
                    self.viewer.set_camera_pose(self.fps_camera_controller.pose)

        # middle click -> drag
        if self.window.mouse_down(2):
            x, y = self.window.mouse_delta
            if x != 0 or y != 0:
                self.focus_entity(None)
                self.fps_camera_controller.move(
                    0,
                    self.rotate_speed * speed_mod * x,
                    self.rotate_speed * speed_mod * y,
                )
                self.viewer.set_camera_pose(self.fps_camera_controller.pose)

        # wheel
        wx, wy = self.window.mouse_wheel_delta
        if wx != 0:
            if self.focused_entity:
                self.arc_camera_controller.zoom(self.scroll_speed * speed_mod * wx)
                self.viewer.set_camera_pose(self.arc_camera_controller.pose)
            else:
                self.fps_camera_controller.move(
                    self.scroll_speed * speed_mod * wx, 0, 0
                )
                self.viewer.set_camera_pose(self.fps_camera_controller.pose)

    def reset(self):
        self.fps_camera_controller = FPSCameraController()
        self.arc_camera_controller = ArcRotateCameraController()

        self.ui_window = None
        self.focused_camera: sapien.CameraEntity = None
        self.focused_entity: sapien.Entity = None

        self.ui_camera = None
        self.ui_pause_checkbox = None

        self.move_speed = 0.05
        self.rotate_speed = 0.005
        self.scroll_speed = 0.5

        self._single_step = False
        self.fps = "FPS: 0"

        self._camera_index = 0
        self._denoiser = False

        # display objects
        self.camera_linesets = []
        self.coordinate_axes = None
        self.joint_axes = None

        self._show_camera_linesets = True
        self.show_joint_axes = True

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []

    def close(self):
        self.reset()
        self._clear_visual_models()

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

        self.camera_lineset = self.renderer_context.create_line_set(
            # fmt: off
            [0, 0, 0, 1, 1, -1, 0, 0, 0, -1, 1, -1, 0, 0, 0, 1, -1, -1, 0, 0, 0, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1.2, -1, 0, 2, -1, 0, 2, -1, -1, 1.2, -1, -1, 1.2, -1, 1, 1.2, -1,],
            [0.9254901960784314, 0.5764705882352941, 0.18823529411764706, 1] * 22,
            # fmt: on
        )

    def _clear_visual_models(self):
        self.cone = None
        self.capsule = None
        self.mat_red = None
        self.mat_green = None
        self.mat_blue = None
        self.mat_cyan = None
        self.mat_magenta = None
        self.red_cone = None
        self.green_cone = None
        self.blue_cone = None
        self.red_capsule = None
        self.green_capsule = None
        self.blue_capsule = None
        self.cyan_capsule = None
        self.magenta_capsule = None
        self.camera_lineset = None

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
            obj.shading_mode = 0
            obj.cast_shadow = False
            obj.transparency = 1
        self.joint_axes = joint_axes

    def _clear_joint_axes(self):
        if self.joint_axes is None:
            return

        assert self.scene is not None
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        for x in self.joint_axes:
            render_scene.remove_node(x)

        self.joint_axes = None

    def _create_coordinate_axes(self):
        assert self.scene is not None
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        node = render_scene.add_node()
        obj = render_scene.add_object(self.red_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([1, 0, 0])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.red_capsule, node)
        obj.set_position([0.5, 0, 0])
        obj.set_scale([1.02, 1.02, 1.02])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.green_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 1, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.green_capsule, node)
        obj.set_position([0, 0.5, 0])
        obj.set_rotation([0.7071068, 0, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.blue_cone, node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([0, 0, 1])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 1

        obj = render_scene.add_object(self.blue_capsule, node)
        obj.set_position([0, 0, 0.5])
        obj.set_rotation([0, 0.7071068, 0, 0.7071068])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 1

        self.coordinate_axes = node

    def _update_camera_linesets(self):
        if self.scene is None:
            return
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        cameras = self.scene.get_cameras()
        if len(self.camera_linesets) != len(cameras):
            self._clear_camera_linesets()
            for c in self.scene.get_cameras():
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

    def _clear_camera_linesets(self):
        if self.scene is None:
            return

        rs = self.scene.renderer_scene
        for n in self.camera_linesets:
            rs._internal_scene.remove_node(n)
        self.camera_linesets = []

    @property
    def fovy(self):
        return self.window.fovy

    @fovy.setter
    def fovy(self, v):
        self.window.set_camera_parameters(self.window.near, self.window.far, v)

    @property
    def display_target_index(self):
        try:
            return self.window.display_target_names.index(self.viewer.render_target)
        except ValueError:
            return 0

    @display_target_index.setter
    def display_target_index(self, index):
        self.viewer.render_target = self.window.display_target_names[index]