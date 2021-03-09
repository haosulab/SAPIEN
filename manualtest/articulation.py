import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector

import sapien.core.pysapien.renderer as R


class FPSCameraController:
    def __init__(self, window: sapien.VulkanWindow):
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


sim = sapien.Engine()
renderer = sapien.VulkanRenderer()
renderer_context: R.Context = renderer._internal_context
sim.set_renderer(renderer)

copper = renderer.create_material()
copper.set_base_color([0.875, 0.553, 0.221, 1])
copper.set_metallic(1)
copper.set_roughness(0.4)

window = renderer.create_window("../shader/full")


def create_ant_builder(scene):
    builder = scene.create_articulation_builder()
    body = builder.create_link_builder()
    body.add_sphere_shape(Pose(), 0.25)
    body.add_sphere_visual_complex(Pose(), 0.25, copper)
    body.add_capsule_shape(Pose([0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual_complex(Pose([0.141, 0, 0]), 0.08, 0.141, copper)
    body.add_capsule_shape(Pose([-0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual_complex(Pose([-0.141, 0, 0]), 0.08, 0.141, copper)
    body.add_capsule_shape(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_visual_complex(
        Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper
    )
    body.add_capsule_shape(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_visual_complex(
        Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper
    )
    body.set_name("body")

    l1 = builder.create_link_builder(body)
    l1.set_name("l1")
    l1.set_joint_name("j1")
    l1.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[-0.5236, 0.5236]],
        Pose([0.282, 0, 0], [0.7071068, 0, 0.7071068, 0]),
        Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
        0.1,
    )
    l1.add_capsule_shape(Pose(), 0.08, 0.141)
    l1.add_capsule_visual_complex(Pose(), 0.08, 0.141, copper)

    l2 = builder.create_link_builder(body)
    l2.set_name("l2")
    l2.set_joint_name("j2")
    l2.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[-0.5236, 0.5236]],
        Pose([-0.282, 0, 0], [0, -0.7071068, 0, 0.7071068]),
        Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
        0.1,
    )
    l2.add_capsule_shape(Pose(), 0.08, 0.141)
    l2.add_capsule_visual_complex(Pose(), 0.08, 0.141, copper)

    l3 = builder.create_link_builder(body)
    l3.set_name("l3")
    l3.set_joint_name("j3")
    l3.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[-0.5236, 0.5236]],
        Pose([0, 0.282, 0], [0.5, -0.5, 0.5, 0.5]),
        Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
        0.1,
    )
    l3.add_capsule_shape(Pose(), 0.08, 0.141)
    l3.add_capsule_visual_complex(Pose(), 0.08, 0.141, copper)

    l4 = builder.create_link_builder(body)
    l4.set_name("l4")
    l4.set_joint_name("j4")
    l4.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[-0.5236, 0.5236]],
        Pose([0, -0.282, 0], [0.5, 0.5, 0.5, -0.5]),
        Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
        0.1,
    )
    l4.add_capsule_shape(Pose(), 0.08, 0.141)
    l4.add_capsule_visual_complex(Pose(), 0.08, 0.141, copper)

    f1 = builder.create_link_builder(l1)
    f1.set_name("f1")
    f1.set_joint_name("j11")
    f1.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f1.add_capsule_shape(Pose(), 0.08, 0.282)
    f1.add_capsule_visual_complex(Pose(), 0.08, 0.282, copper)

    f2 = builder.create_link_builder(l2)
    f2.set_name("f2")
    f2.set_joint_name("j21")
    f2.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f2.add_capsule_shape(Pose(), 0.08, 0.282)
    f2.add_capsule_visual_complex(Pose(), 0.08, 0.282, copper)

    f3 = builder.create_link_builder(l3)
    f3.set_name("f3")
    f3.set_joint_name("j31")
    f3.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f3.add_capsule_shape(Pose(), 0.08, 0.282)
    f3.add_capsule_visual_complex(Pose(), 0.08, 0.282, copper)

    f4 = builder.create_link_builder(l4)
    f4.set_name("f4")
    f4.set_joint_name("j41")
    f4.set_joint_properties(
        sapien.ArticulationJointType.REVOLUTE,
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f4.add_capsule_shape(Pose(), 0.08, 0.282)
    f4.add_capsule_visual_complex(Pose(), 0.08, 0.282, copper)

    return builder


scene = sim.create_scene()
scene.add_ground(-3)
scene.set_timestep(1 / 240)

ant_builder = create_ant_builder(scene)

ant = ant_builder.build()
ant.set_root_pose(Pose([0, 0, 2]))

window.set_scene(scene)
window.set_camera_parameters(0.5, 20, 1)

for j in ant.get_joints():
    j.set_friction(0)

scene.step()
ant.set_qpos([0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7])
ant.set_qvel([0] * 8)
f = [0.1] * 8
acc = ant.compute_forward_dynamics([0.1] * 8)
ant.set_qf(f)
scene.step()

rs = scene.get_render_scene()
rs.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
rs.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

rs.add_shadow_point_light([0, 1, 1], [1, 2, 2])
rs.add_shadow_point_light([0, -1, 1], [2, 1, 2])
rs.add_shadow_point_light([0, 1, -1], [2, 2, 1])

cc = FPSCameraController(window)
cc.setXYZ(-5, 0, 2)

render_scene: R.Scene = rs._internal_scene


cone = renderer_context.create_cone_mesh(16)
capsule = renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)
mat_red = renderer_context.create_material([1, 0, 0, 1], 0, 0, 0)
mat_green = renderer_context.create_material([0, 1, 0, 1], 0, 0, 0)
mat_blue = renderer_context.create_material([0, 0, 1, 1], 0, 0, 0)
red_cone = renderer_context.create_model([cone], [mat_red])
green_cone = renderer_context.create_model([cone], [mat_green])
blue_cone = renderer_context.create_model([cone], [mat_blue])
red_capsule = renderer_context.create_model([capsule], [mat_red])
green_capsule = renderer_context.create_model([capsule], [mat_green])
blue_capsule = renderer_context.create_model([capsule], [mat_blue])


def create_axes():
    node = render_scene.add_node()
    obj = render_scene.add_object(red_cone, node)
    obj.set_scale([0.5, 0.2, 0.2])
    obj.set_position([1, 0, 0])
    obj.shading_mode = 2
    obj = render_scene.add_object(red_capsule, node)
    obj.set_position([0.5, 0, 0])
    obj.set_scale([1.02, 1.02, 1.02])
    obj.shading_mode = 2

    obj = render_scene.add_object(green_cone, node)
    obj.set_scale([0.5, 0.2, 0.2])
    obj.set_position([0, 1, 0])
    obj.set_rotation([0.7071068, 0, 0, 0.7071068])
    obj.shading_mode = 2
    obj = render_scene.add_object(green_capsule, node)
    obj.set_position([0, 0.5, 0])
    obj.set_rotation([0.7071068, 0, 0, 0.7071068])
    obj.shading_mode = 2

    obj = render_scene.add_object(blue_cone, node)
    obj.set_scale([0.5, 0.2, 0.2])
    obj.set_position([0, 0, 1])
    obj.set_rotation([0, 0.7071068, 0, 0.7071068])
    obj.shading_mode = 2
    obj = render_scene.add_object(blue_capsule, node)
    obj.set_position([0, 0, 0.5])
    obj.set_rotation([0, 0.7071068, 0, 0.7071068])
    obj.shading_mode = 2

    return node


axes = create_axes()
axes.set_position([0, 0, -1])


target_name = "Color"
pause = False


def set_target(name):
    global target_name
    target_name = name


def toggle_pause(p):
    global pause
    pause = p


ui1 = (
    R.UIWindow()
    .Label("test window 1")
    .Pos(10, 10)
    .Size(200, 400)
    .append(
        R.UIDisplayText().Text("display text"),
        R.UICheckbox().Label("Pause").Callback(lambda p: toggle_pause(p.checked)),
        R.UICheckbox().Label("Checkbox1").Callback(lambda p: (print(p.checked))),
        R.UIRadioButtonGroup()
        .Labels(window.target_names)
        .Callback(lambda p: set_target(p.value)),
    )
)


def find_actor(id):
    actors = scene.get_all_actors()
    for actor in actors:
        if actor.id == id:
            return actor
    for a in scene.get_all_articulations():
        for link in a.get_links():
            if link.id == id:
                return link


# renderer.set_shader_dir("../shader/full")
# near, far = 0.1, 100
# camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
# camera = scene.add_mounted_camera(
#     "first_camera",
#     camera_mount_actor,
#     Pose(),
#     640,
#     480,
#     np.deg2rad(35),
#     np.deg2rad(35),
#     near,
#     far,
# )

# pos = np.array([3, -2, 3])
# forward = -pos / np.linalg.norm(pos)
# left = np.cross([0, 0, 1], forward)
# left = left / np.linalg.norm(left)
# up = np.cross(forward, left)
# mat44 = np.eye(4)
# mat44[:3, :3] = np.linalg.inv(np.array([forward, left, up]))
# mat44[:3, 3] = pos
# camera_mount_actor.set_pose(Pose.from_transformation_matrix(mat44))

# scene.step()
# scene.update_render()
# camera.take_picture()

# from PIL import Image

# rgba = camera.get_float_texture("Color")
# rgba = (rgba * 255).clip(0, 255).astype("uint8")
# rgba = Image.fromarray(rgba)
# rgba.save("color.png")


selected_actor = None

count = 0
while True:
    scene.update_render()
    for i in range(4):
        ant.set_qf(np.random.randn(8) * 500)
        scene.step()

    while True:
        window.render(target_name, [ui1])
        mx, my = window.mouse_position
        if window.mouse_click(0):
            pixel = window.download_uint32_target_pixel(
                "Segmentation", int(mx), int(my)
            )
            actor = find_actor(pixel[1])
            if actor:
                if selected_actor:
                    for v in selected_actor.get_visual_bodies():
                        v.set_visibility(1)
                selected_actor = actor
                for v in selected_actor.get_visual_bodies():
                    v.set_visibility(0.8)
            else:
                if selected_actor:
                    for v in selected_actor.get_visual_bodies():
                        v.set_visibility(1)
                selected_actor = None
                axes.set_position([0, 0, 0])
                axes.set_rotation([0, 0, 0, 1])
                axes.set_scale([0.3, 0.3, 0.3])

        if selected_actor:
            axes.set_scale([0.3, 0.3, 0.3])
            axes.set_position(selected_actor.pose.p)
            axes.set_rotation(selected_actor.pose.q)

        if window.key_down("w"):
            cc.move(0.1, 0, 0)
        if window.key_down("s"):
            cc.move(-0.1, 0, 0)
        if window.key_down("a"):
            cc.move(0, 0.1, 0)
        if window.key_down("d"):
            cc.move(0, -0.1, 0)
        if window.mouse_down(1):
            x, y = window.mouse_delta
            cc.rotate(0, -0.005 * y, -0.005 * x)

        if not pause:
            break

        if window.key_down("q"):
            break
    if window.key_down("q"):
        break

window = None
scene = None
