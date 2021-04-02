import sapien.core.pysapien.renderer as R
from sapien.core import (
    Pose,
    VulkanRenderer,
    Scene,
    VulkanWindow,
    ArticulationBase,
    Articulation,
    Joint,
    LinkBase
)
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
        resolutions=((1024, 768), (800, 600), (1920, 1080)),
    ):
        self.shader_dir = shader_dir
        self.renderer = renderer
        self.renderer_context: R.Context = renderer._internal_context

        self.window = None
        self.resolution = None
        self.resolutions = None
        self.set_window_resolutions(resolutions)

        self.cone = self.renderer_context.create_cone_mesh(16)
        self.capsule = self.renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)
        self.mat_red = self.renderer_context.create_material([1, 0, 0, 1], 0, 0, 0)
        self.mat_joint_axis = self.renderer_context.create_material(
            [1, 0, 1, 1], 0, 0, 0
        )
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
        self.joint_axis_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_joint_axis]
        )
        self.green_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_green]
        )
        self.blue_capsule = self.renderer_context.create_model(
            [self.capsule], [self.mat_blue]
        )

        self.axes = None
        self.axes_scale = 0.3
        self.joint_axis = None

        self.selected_actor = None
        self.focused_actor = None
        self.paused = False
        self.target_name = "Color"
        self.single_step = False

        self.move_speed = 0.05
        self.rotate_speed = 0.005
        self.scroll_speed = 0.5
        self.selection_opacity = 0.7

        self.scene_window = None
        self.control_window = None

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
                    R.UIDisplayText().Text("Display Settings"),
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
                    R.UISliderFloat()
                    .Label("Axes Scale")
                    .Min(0)
                    .Max(1)
                    .Value(self.axes_scale)
                    .Callback(lambda p: self.update_axes_scale(p.value)),
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
                .Callback(lambda _: self.select_actor(actor))
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

    def set_move_speed(self, x):
        self.move_speed = x

    def set_rotate_speed(self, x):
        self.rotate_speed = x

    def set_scroll_speed(self, x):
        self.scroll_speed = x

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

    def create_joint_axis(self):
        assert self.scene is not None
        rs = self.scene.get_render_scene()
        render_scene: R.Scene = rs._internal_scene

        obj = render_scene.add_object(self.joint_axis_capsule)
        obj.set_position([0, 0, 0])
        obj.set_scale([5, 0.1, 0.1])
        obj.transparency = 1
        obj.shading_mode = 2

        return obj

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
            self.axes.set_scale([self.axes_scale] * 3)
            if self.selected_actor:
                self.axes.set_position(self.selected_actor.pose.p)
                self.axes.set_rotation(self.selected_actor.pose.q)
            else:
                self.axes.set_position([0, 0, 0])
                self.axes.set_rotation([1, 0, 0, 0])
            self.joint_axis = self.create_joint_axis()
        elif self.scene:
            rs = self.scene.get_render_scene()
            render_scene: R.Scene = rs._internal_scene
            render_scene.remove_node(self.axes)
            render_scene.remove_node(self.joint_axis)
            self.axes = None
            self.joint_axis = None

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
                v.set_visibility(self.selection_opacity)
        else:
            if self.selected_actor:
                for v in self.selected_actor.get_visual_bodies():
                    v.set_visibility(1)
            self.selected_actor = None
            if self.axes:
                self.axes.set_position([0, 0, 0])
                self.axes.set_rotation([1, 0, 0, 0])

    def update_axes_scale(self, scale):
        self.axes_scale = scale
        self.update_axes()

    def update_axes(self):
        if self.selected_actor and self.axes:
            self.axes.set_position(self.selected_actor.pose.p)
            self.axes.set_rotation(self.selected_actor.pose.q)
            self.axes.set_scale([self.axes_scale] * 3)

    def update_joint_axis(self):
        if not self.joint_axis:
            return

        if self.selected_actor and "link" in self.selected_actor.type:
            link: LinkBase = self.selected_actor
            j = link.get_articulation().get_joints()[link.get_index()]
            if j.type not in ["revolute", "prismatic"]:
                self.joint_axis.transparency = 1
                return
            j2c = j.get_pose_in_child_frame()
            c2w = link.get_pose()
            j2w = c2w * j2c
            if j.type == "prismatic":
                self.mat_joint_axis.set_base_color([0, 1, 1, 1])
                j2w.set_p(c2w.p)
            else:
                self.mat_joint_axis.set_base_color([1, 0, 1, 1])
            self.joint_axis.set_position(j2w.p)
            self.joint_axis.set_rotation(j2w.q)
            self.joint_axis.transparency = 0
        else:
            self.joint_axis.transparency = 1

    def render(self):
        if self.closed:
            return

        while True:
            self.build_control_window()
            self.build_scene_window()
            self.build_actor_window()
            self.build_articulation_window()
            self.window.render(
                self.target_name,
                [
                    self.control_window,
                    self.scene_window,
                    self.actor_window,
                    self.articulation_window,
                ],
            )

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

            self.update_axes()
            self.update_joint_axis()

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
                    self.arc_camera_controller.rotate_yaw_pitch(
                        -self.rotate_speed * speed_mod * x,
                        self.rotate_speed * speed_mod * y,
                    )
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
                    self.arc_camera_controller.zoom(self.scroll_speed * speed_mod * wx)
                else:
                    self.fps_camera_controller.move(
                        self.scroll_speed * speed_mod * wx, 0, 0
                    )

            if self.focused_actor:
                self.arc_camera_controller.set_center(self.focused_actor.pose.p)

            if self.window.key_down("q"):
                self.close()
                return

            if not self.paused or (self.paused and self.single_step):
                self.single_step = False
                break

    def is_mouse_available(self, mx, my):
        w, h = self.window.size
        print(f"[I] windowSize: {w, h}; mousePose: {mx, my}")
        return mx >= 0 and my >= 0
