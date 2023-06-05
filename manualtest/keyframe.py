import sapien.core as sapien
from sapien.utils.viewer import Viewer

# Set up engine and renderer
engine = sapien.Engine()
engine.set_log_level('critical')
renderer = sapien.SapienRenderer()
engine.set_renderer(renderer)

# Set up scene
scene_config = sapien.SceneConfig()
scene = engine.create_scene(scene_config)
scene.set_timestep(1 / 240.0)
scene.add_ground(-0.8)
physical_material = scene.create_physical_material(1, 1, 0.0)
scene.default_physical_material = physical_material

builder = scene.create_actor_builder()
builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
table = builder.build_kinematic(name='table')
table.set_pose(sapien.Pose([0.56, 0, - 0.025]))

builder = scene.create_actor_builder()
builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
builder.add_box_visual(half_size=[0.02, 0.02, 0.07], color=[0, 0, 1])
blue_cube = builder.build(name='blue_cube')
blue_cube.set_pose(sapien.Pose([0.6, 0.1, 0.07]))

# Set up robot
loader = scene.create_urdf_loader()
robot = loader.load("../assets/robot/panda/panda.urdf")
robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))

# Set up initial joint positions
init_qpos = [0, 0.19634954084936207, 0.0, -2.617993877991494, 0.0, 2.941592653589793, 0.7853981633974483, 0, 0]
robot.set_qpos(init_qpos)

active_joints = robot.get_active_joints()
for joint_idx, joint in enumerate(active_joints):
    joint.set_drive_property(stiffness=1000, damping=200)
    joint.set_drive_target(init_qpos[joint_idx])

# Set up light
scene.set_ambient_light([0.5, 0.5, 0.5])
scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

# Set up viewer
viewer = Viewer(renderer, resolutions=(1920, 1080))
viewer.set_scene(scene)
viewer.set_camera_xyz(x=1.2, y=0.25, z=0.4)
viewer.set_camera_rpy(r=0, p=-0.4, y=2.7)

while not viewer.closed:
    qf = robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True)
    robot.set_qf(qf)

    scene.step()
    scene.update_render()
    viewer.render()
