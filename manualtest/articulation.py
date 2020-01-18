import pysapien
import numpy as np
from pysapien import Pose
from transforms3d.quaternions import axangle2quat as aa

sim = pysapien.Simulation()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = pysapien.OptifuserController(renderer)


def create_ant_builder(scene):
    builder = scene.create_articulation_builder()
    body = builder.create_link_builder()
    body.add_sphere_shape(Pose(), 0.25)
    body.add_sphere_visual(Pose(), 0.25)
    body.add_capsule_shape(Pose([0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual(Pose([0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_shape(Pose([-0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual(Pose([-0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_shape(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_visual(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_shape(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_visual(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.set_name("body")

    l1 = builder.create_link_builder(body)
    l1.set_name("l1")
    l1.set_joint_name("j1")
    l1.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([0.282, 0, 0], [0.7071068, 0, 0.7071068, 0]),
                            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]), 0.1)
    l1.add_capsule_shape(Pose(), 0.08, 0.141)
    l1.add_capsule_visual(Pose(), 0.08, 0.141)

    l2 = builder.create_link_builder(body)
    l2.set_name("l2")
    l2.set_joint_name("j2")
    l2.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([-0.282, 0, 0], [0, -0.7071068, 0, 0.7071068]),
                            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]), 0.1)
    l2.add_capsule_shape(Pose(), 0.08, 0.141)
    l2.add_capsule_visual(Pose(), 0.08, 0.141)

    l3 = builder.create_link_builder(body)
    l3.set_name("l3")
    l3.set_joint_name("j3")
    l3.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([0, 0.282, 0], [0.5, -0.5, 0.5, 0.5]),
                            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]), 0.1)
    l3.add_capsule_shape(Pose(), 0.08, 0.141)
    l3.add_capsule_visual(Pose(), 0.08, 0.141)

    l4 = builder.create_link_builder(body)
    l4.set_name("l4")
    l4.set_joint_name("j4")
    l4.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([0, -0.282, 0], [0.5, 0.5, 0.5, -0.5]),
                            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]), 0.1)
    l4.add_capsule_shape(Pose(), 0.08, 0.141)
    l4.add_capsule_visual(Pose(), 0.08, 0.141)

    f1 = builder.create_link_builder(l1)
    f1.set_name("f1")
    f1.set_joint_name("j11")
    f1.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f1.add_capsule_shape(Pose(), 0.08, 0.282)
    f1.add_capsule_visual(Pose(), 0.08, 0.282)

    f2 = builder.create_link_builder(l2)
    f2.set_name("f2")
    f2.set_joint_name("j21")
    f2.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f2.add_capsule_shape(Pose(), 0.08, 0.282)
    f2.add_capsule_visual(Pose(), 0.08, 0.282)

    f3 = builder.create_link_builder(l3)
    f3.set_name("f3")
    f3.set_joint_name("j31")
    f3.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f3.add_capsule_shape(Pose(), 0.08, 0.282)
    f3.add_capsule_visual(Pose(), 0.08, 0.282)

    f4 = builder.create_link_builder(l4)
    f4.set_name("f4")
    f4.set_joint_name("j41")
    f4.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f4.add_capsule_shape(Pose(), 0.08, 0.282)
    f4.add_capsule_visual(Pose(), 0.08, 0.282)

    return builder


render_controller.show_window()

s0 = sim.create_scene()
s0.add_ground(-1)
s0.set_timestep(1 / 240)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

ant_builder = create_ant_builder(s0)

ant = ant_builder.build()

render_controller.camera.set_position([-5, 0, 2])
render_controller.set_current_scene(s0)

while not render_controller.should_quit:
    ant.set_qf(np.random.rand(8) * 14000 - 7000)
    s0.update_render()
    for i in range(4):
        s0.step()
    render_controller.render()

s0 = None
