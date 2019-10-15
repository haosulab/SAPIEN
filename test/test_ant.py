import sapyen
import numpy as np
from sapyen import PxTransform

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
body_link = builder.add_link(None, PxTransform(np.array([0, 0, 0]), PxIdentity), "body")
builder.add_sphere_shape_to_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)
builder.add_sphere_visual_to_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)

builder.add_capsule_shape_to_link(body_link, PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(body_link, PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)

builder.add_capsule_shape_to_link(body_link, PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(body_link, PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)

builder.add_capsule_shape_to_link(body_link,
                                  PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])),
                                  0.08, 0.141)
builder.add_capsule_visual_to_link(body_link,
                                   PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])),
                                   0.08, 0.141)

builder.add_capsule_shape_to_link(body_link,
                                  PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])),
                                  0.08, 0.141)
builder.add_capsule_visual_to_link(
    body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)

builder.update_link_mass_and_inertia(body_link, 1000)

l1 = builder.add_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), "l1", "j1",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      PxTransform(np.array([0.282, 0, 0]), np.array([0.7071068, 0, 0.7071068, 0])),
                      PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l1, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l1, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l1, 1000)

l2 = builder.add_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), "l2", "j2",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      PxTransform(np.array([-0.282, 0, 0]), np.array([0, 0.7071068, 0, -0.7071068])),
                      PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l2, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l2, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l2, 1000)

l3 = builder.add_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), "l3", "j3",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      PxTransform(np.array([0, 0.282, 0]), np.array([0.5, -0.5, 0.5, 0.5])),
                      PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l3, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l3, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l3, 1000)

l4 = builder.add_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), "l4", "j4",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                      PxTransform(np.array([0, -0.282, 0]), np.array([0.5, 0.5, 0.5, -0.5])),
                      PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.add_capsule_shape_to_link(l4, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.add_capsule_visual_to_link(l4, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.update_link_mass_and_inertia(l4, 1000)

f1 = builder.add_link(l1, PxTransform(np.array([0, 0, 0]), PxIdentity), "f1", "j11",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f1, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f1, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f1, 1000)

f2 = builder.add_link(l2, PxTransform(np.array([0, 0, 0]), PxIdentity), "f2", "j21",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f2, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f2, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f2, 1000)

f3 = builder.add_link(l3, PxTransform(np.array([0, 0, 0]), PxIdentity), "f3", "j31",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f3, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f3, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f3, 1000)

f4 = builder.add_link(l4, PxTransform(np.array([0, 0, 0]), PxIdentity), "f4", "j41",
                      sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                      PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                      PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.add_capsule_shape_to_link(f4, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.add_capsule_visual_to_link(f4, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.update_link_mass_and_inertia(f4, 1000)

wrapper = builder.build(False)
ground = sim.add_ground(-1)

renderer.show_window()

while True:
    wrapper.set_qf(np.random.rand(8) * 4000 - 2000)
    for i in range(10):
        sim.step()
    sim.update_renderer()
    renderer.render()
    qpos = wrapper.get_qpos()
    qvel = wrapper.get_qvel()
    print(qpos)
    print(qvel)
