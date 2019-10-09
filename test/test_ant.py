import sapyen
import numpy as np
import keyboard
from sapyen import PxTransform
renderer = sapyen.OptifuserRenderer()
renderer.init()
renderer.cam.set_position(np.array([0, 1, 10]))
renderer.cam.setForward(np.array([0, 1, 0]))
renderer.cam.setUp(np.array([0, 0, 1]))
renderer.cam.rotateYawPitch(0, -1.5)

sim = sapyen.Simulation()
sim.setRenderer(renderer)
sim.setTimestep(1.0 / 200.0)

builder = sim.createArticulationBuilder()

PxIdentity = np.array([0, 0, 0, 1])
body_link = builder.addLink(None, PxTransform(np.array([0, 0, 0]), PxIdentity), "body")
builder.addSphereShapeToLink(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)
builder.addSphereVisualToLink(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)


builder.addCapsuleShapeToLink(body_link,  PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)

builder.addCapsuleShapeToLink(body_link,  PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)

builder.addCapsuleShapeToLink(body_link, PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)

builder.addCapsuleShapeToLink(body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)

builder.updateLinkMassAndInertia(body_link, 1000)

l1 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l1", "j1",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                             PxTransform(np.array([0.282, 0, 0]), np.array([0.7071068, 0, 0.7071068, 0])),
                             PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.addCapsuleShapeToLink(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.addCapsuleVisualToLink(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.updateLinkMassAndInertia(l1, 1000)

l2 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l2", "j2",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                             PxTransform(np.array([-0.282, 0, 0]), np.array([0, 0.7071068, 0, -0.7071068])),
                             PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.addCapsuleShapeToLink(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.addCapsuleVisualToLink(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.updateLinkMassAndInertia(l2, 1000)

l3 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l3", "j3",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                             PxTransform(np.array([0, 0.282, 0]), np.array([0.5, -0.5, 0.5, 0.5])),
                             PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
builder.addCapsuleShapeToLink(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.addCapsuleVisualToLink(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.updateLinkMassAndInertia(l3, 1000)

l4 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l4", "j4",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                             PxTransform(np.array([0, -0.282, 0]), np.array([0.5, 0.5, 0.5, -0.5])),
                             PxTransform(np.array([0.141, 0, 0]), np.array([ 0.7071068, 0, -0.7071068, 0])))
builder.addCapsuleShapeToLink(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.addCapsuleVisualToLink(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
builder.updateLinkMassAndInertia(l4, 1000)

f1 = builder.addLink(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f1", "j11",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                             PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                             PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.addCapsuleShapeToLink(f1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.addCapsuleVisualToLink(f1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.updateLinkMassAndInertia(f1, 1000)

f2 = builder.addLink(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f2", "j21",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                             PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                             PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.addCapsuleShapeToLink(f2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.addCapsuleVisualToLink(f2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.updateLinkMassAndInertia(f2, 1000)

f3 = builder.addLink(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f3", "j31",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                             PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                             PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.addCapsuleShapeToLink(f3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.addCapsuleVisualToLink(f3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.updateLinkMassAndInertia(f3, 1000)


f4 = builder.addLink(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f4", "j41",
                             sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                             PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                             PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
builder.addCapsuleShapeToLink(f4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.addCapsuleVisualToLink(f4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
builder.updateLinkMassAndInertia(f4, 1000)

wrapper = builder.build(False)
ground = sim.addGround(-1)

while True:
    for i in range(10):
        sim.step()
    sim.updateRenderer()
    renderer.render()
    qpos = wrapper.get_qpos()
    qvel = wrapper.get_qvel()
    print(qpos)
    print(qvel)
