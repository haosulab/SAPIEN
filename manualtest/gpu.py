import sapien
import torch

sapien.physx.enable_gpu()
sapien.set_cuda_tensor_backend("torch")


def create_scene(offset):
    scene = sapien.Scene()
    scene.physx_system.set_scene_offset(scene, offset)
    scene.set_ambient_light([0.5, 0.5, 0.5])
    return scene


scene0 = create_scene([0, 0, 0])
scene1 = create_scene([10, 0, 0])

assert scene0.physx_system == scene1.physx_system
physx_system: sapien.physx.PhysxGpuSystem = scene0.physx_system

loader = scene0.create_urdf_loader()
builder = loader.load_file_as_articulation_builder("../assets/robot/panda/panda.urdf")

builder.set_scene(scene0)
r0 = builder.build()

builder.set_scene(scene1)
r1 = builder.build()

builder = scene0.create_actor_builder()
builder.add_box_collision(half_size=[0.1, 0.1, 0.1])
builder.add_box_visual(half_size=[0.1, 0.1, 0.1])
builder.set_initial_pose(sapien.Pose([0, 0, 5]))

builder.set_scene(scene0)
a0 = builder.build()

builder.set_scene(scene1)
a1 = builder.build()

physx_system.gpu_init()
physx_system.step()

ai = physx_system.gpu_create_articulation_index_buffer([r0, r1])
print("ai", ai)

qpos_buffer = physx_system.gpu_create_articulation_q_buffer()
print("qpos", qpos_buffer.shape)

qvel_buffer = physx_system.gpu_create_articulation_q_buffer()
print("qvel", qvel_buffer.shape)

offset_buffer = physx_system.gpu_create_articulation_offset_buffer()
print("offset", offset_buffer)

root_pose_buffer = physx_system.gpu_create_articulation_root_pose_buffer()
print("root_pose", root_pose_buffer.shape)

physx_system.gpu_query_articulation_qpos(qpos_buffer, ai)
physx_system.gpu_query_articulation_qvel(qvel_buffer, ai)
print("qpos", qpos_buffer)
print("qvel", qvel_buffer)

physx_system.gpu_query_articulation_root_pose(root_pose_buffer, ai, offset_buffer)
print("root pose", root_pose_buffer)

root_pose_buffer[0] = torch.tensor([0.1, 0, 0, 0, 1, 0, 0])
root_pose_buffer[1] = torch.tensor([0, 0.1, 0, 0, 0, 1, 0])

physx_system.gpu_apply_articulation_root_pose(root_pose_buffer, ai, offset_buffer)
root_pose_buffer[0] = torch.tensor([0, 0, 0, 0, 0, 0, 0])
root_pose_buffer[1] = torch.tensor([0, 0, 0, 0, 0, 0, 0])

# physx_system.gpu_update_articulation_kinematics()

physx_system.gpu_query_articulation_root_pose(root_pose_buffer, ai, offset_buffer)
print("sapien pose", root_pose_buffer)

physx_system._gpu_query_articulation_root_pose_raw(root_pose_buffer, ai)
print("raw pose", root_pose_buffer)


bodies = [
    a.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent) for a in [a0, a1]
]
bi = physx_system.gpu_create_body_index_buffer(bodies)
body_data_buffer = physx_system.gpu_create_body_data_buffer(2)
body_offset_buffer = physx_system.gpu_create_body_offset_buffer(bodies)

print("bi", bi)
print(body_data_buffer)
print(body_offset_buffer)

physx_system.gpu_query_body_data(body_data_buffer, bi, body_offset_buffer)

print("body data", body_data_buffer)

# p, q, v, w
body_data_buffer[0, :13] = torch.tensor([0.1, 0, 10, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0.5])
body_data_buffer[1, :13] = torch.tensor([0, 0.1, 8, 0, 0, 1, 0, 0, 0, -2, 0, 0, 0.8])
physx_system.gpu_apply_body_data(body_data_buffer, bi, body_offset_buffer)

physx_system.step()

physx_system.gpu_query_body_data(body_data_buffer, bi, body_offset_buffer)
print(body_data_buffer)
