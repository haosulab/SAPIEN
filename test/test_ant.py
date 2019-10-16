import sapyen
import numpy as np
from sapyen import Pose
import time

renderer = sapyen.OptifuserRenderer()

renderer.cam.set_position(np.array([0, 1, 10]))
renderer.cam.set_forward(np.array([0, 1, 0]))
renderer.cam.set_up(np.array([0, 0, 1]))
renderer.cam.rotate_yaw_pitch(0, -1.5)

sim = sapyen.Simulation()
sim.set_renderer(renderer)
sim.set_time_step(1.0 / 200.0)

builder = sim.create_articulation_builder()

PxIdentity = np.array([0, 0, 0, 1])
body_link = builder.add_link(None, Pose(np.array([0, 0, 0]), PxIdentity), "body")
builder.add_sphere_shape_to_link(body_link, Pose(np.array([0, 0, 0]), PxIdentity), 0.25)
builder.add_sphere_visual_to_link(body_link, Pose(np.array([0, 0, 0]), PxIdentity), 0.25)

builder.add_capsule_shape_to_link(body_link, Pose(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(body_link, Pose(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)

builder.add_capsule_shape_to_link(body_link, Pose(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(body_link, Pose(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)

builder.add_capsule_shape_to_link(body_link,
                                  Pose(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])),
                                  0.08, 0.141)
builder.add_capsule_visual_to_link(body_link,
                                   Pose(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])),
                                   0.08, 0.141)

builder.add_capsule_shape_to_link(body_link,
                                  Pose(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])),
                                  0.08, 0.141)
builder.add_capsule_visual_to_link(
    body_link, Pose(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)

builder.update_link_mass_and_inertia(body_link, 1000)

l1 = builder.add_link(body_link, Pose(np.array([0, 0, 0]), PxIdentity), "l1", "j1",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      Pose(np.array([0.282, 0, 0]), np.array([0.7071068, 0, 0.7071068, 0])),
                      Pose(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l1, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l1, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l1, 1000)

l2 = builder.add_link(body_link, Pose(np.array([0, 0, 0]), PxIdentity), "l2", "j2",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      Pose(np.array([-0.282, 0, 0]), np.array([0, 0.7071068, 0, -0.7071068])),
                      Pose(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l2, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l2, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l2, 1000)

l3 = builder.add_link(body_link, Pose(np.array([0, 0, 0]), PxIdentity), "l3", "j3",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      Pose(np.array([0, 0.282, 0]), np.array([0.5, -0.5, 0.5, 0.5])),
                      Pose(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l3, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l3, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l3, 1000)

l4 = builder.add_link(body_link, Pose(np.array([0, 0, 0]), PxIdentity), "l4", "j4",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      Pose(np.array([0, -0.282, 0]), np.array([0.5, 0.5, 0.5, -0.5])),
                      Pose(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l4, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l4, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l4, 1000)

f1 = builder.add_link(l1, Pose(np.array([0, 0, 0]), PxIdentity), "f1", "j11",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      Pose(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      Pose(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f1, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f1, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f1, 1000)

f2 = builder.add_link(l2, Pose(np.array([0, 0, 0]), PxIdentity), "f2", "j21",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      Pose(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      Pose(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f2, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f2, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f2, 1000)

f3 = builder.add_link(l3, Pose(np.array([0, 0, 0]), PxIdentity), "f3", "j31",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      Pose(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      Pose(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f3, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f3, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f3, 1000)

f4 = builder.add_link(l4, Pose(np.array([0, 0, 0]), PxIdentity), "f4", "j41",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      Pose(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      Pose(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f4, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f4, Pose(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f4, 1000)

wrapper = builder.build(False)
ground = sim.add_ground(-1)

# renderer.show_window()

while True:
    start = time.time()
    for i in range(10000):
        wrapper.set_qf(np.random.rand(8) * 4000 - 2000)
        sim.step()
    print(time.time() - start, 's for 10000 steps')
