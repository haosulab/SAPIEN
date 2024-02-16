import sapien
import torch

sapien.set_log_level("info")
sapien.render.set_log_level("info")

sapien.physx.enable_gpu()

physx_system = sapien.physx.PhysxGpuSystem()

print(physx_system.device)


def create_scene(offset):
    scene = sapien.Scene([physx_system, sapien.render.RenderSystem()])
    physx_system.set_scene_offset(scene, offset)
    scene.set_ambient_light([0.5, 0.5, 0.5])
    return scene


scene0 = create_scene([0, 0, 0])
scene1 = create_scene([10, 0, 0])

assert scene0.physx_system == scene1.physx_system

loader = scene0.create_urdf_loader()
builder = loader.load_file_as_articulation_builder("../assets/robot/panda/panda.urdf")

builder.set_scene(scene0)
r0 = builder.build()

builder.set_scene(scene1)
r1 = builder.build()

builder = scene0.create_actor_builder()
builder.add_box_collision(half_size=[0.1, 0.1, 0.1])
builder.add_box_visual(half_size=[0.1, 0.1, 0.1])
builder.add_box_visual(half_size=[0.2, 0.2, 0.2])
builder.set_initial_pose(sapien.Pose([0, 0, 5]))

builder.set_scene(scene0)
a0 = builder.build()

builder.set_scene(scene1)
a1 = builder.build()

physx_system.gpu_init()


# physx_system.step()

# ai = physx_system.gpu_create_articulation_index_buffer([r0, r1])
# print("ai", ai)

# qpos_buffer = physx_system.gpu_create_articulation_q_buffer()
# print("qpos", qpos_buffer.shape)

# qvel_buffer = physx_system.gpu_create_articulation_q_buffer()
# print("qvel", qvel_buffer.shape)

# offset_buffer = physx_system.gpu_create_articulation_offset_buffer()
# print("offset", offset_buffer)

# root_pose_buffer = physx_system.gpu_create_articulation_root_pose_buffer()
# print("root_pose", root_pose_buffer.shape)

# physx_system.gpu_query_articulation_qpos(qpos_buffer, ai)
# physx_system.gpu_query_articulation_qvel(qvel_buffer, ai)
# print("qpos", qpos_buffer)
# print("qvel", qvel_buffer)

# physx_system.gpu_query_articulation_root_pose(root_pose_buffer, ai, offset_buffer)
# print("root pose", root_pose_buffer)

# root_pose_buffer[0] = torch.tensor([0.1, 0, 0, 0, 1, 0, 0])
# root_pose_buffer[1] = torch.tensor([0, 0.1, 0, 0, 0, 1, 0])

# physx_system.gpu_apply_articulation_root_pose(root_pose_buffer, ai, offset_buffer)
# root_pose_buffer[0] = torch.tensor([0, 0, 0, 0, 0, 0, 0])
# root_pose_buffer[1] = torch.tensor([0, 0, 0, 0, 0, 0, 0])

# # physx_system.gpu_update_articulation_kinematics()

# physx_system.gpu_query_articulation_root_pose(root_pose_buffer, ai, offset_buffer)
# print("sapien pose", root_pose_buffer)

# physx_system._gpu_query_articulation_root_pose_raw(root_pose_buffer, ai)
# print("raw pose", root_pose_buffer)


all_actors = [a0, a1]

bodies = [
    a.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
    for a in all_actors
]


qpos = physx_system.cuda_articulation_qpos
qvel = physx_system.cuda_articulation_qvel
qacc = physx_system.cuda_articulation_qacc
qf = physx_system.cuda_articulation_qf
target_qpos = physx_system.cuda_articulation_target_qpos
target_qvel = physx_system.cuda_articulation_target_qvel

body_data = physx_system.cuda_rigid_body_data

actor_data = physx_system.cuda_rigid_dynamic_data
actor_pose_slice = actor_data[:, :7]  # this is in-place

link_data = physx_system.cuda_articulation_link_data
import ipdb

ipdb.set_trace()

physx_system.gpu_fetch_rigid_dynamic_data()
print(actor_data)
print(actor_pose_slice)

physx_system.gpu_fetch_articulation_link_pose()
physx_system.gpu_fetch_articulation_link_velocity()
print(link_data)

physx_system.gpu_fetch_articulation_qpos()
print(qpos)

physx_system.gpu_fetch_articulation_qvel()
print(qvel)

physx_system.gpu_fetch_articulation_qacc()
print(qacc)

physx_system.gpu_fetch_articulation_target_qpos()
print(target_qpos)

physx_system.gpu_fetch_articulation_target_qvel()
print(target_qvel)


# bi = physx_system.gpu_create_body_index_buffer(bodies)
# body_data_buffer = physx_system.gpu_create_body_data_buffer(2)
# body_offset_buffer = physx_system.gpu_create_body_offset_buffer(bodies)

# print("bi", bi)
# print(body_data_buffer)
# print(body_offset_buffer)

# physx_system.gpu_query_body_data(body_data_buffer, bi, body_offset_buffer)
# print("body data", body_data_buffer)

# physx_system._gpu_query_body_data_raw(body_data_buffer, bi)
# print("body data raw", body_data_buffer)

# # p, q, v, w
# body_data_buffer[0, :13] = torch.tensor([0.1, 0, 10, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0.5])
# body_data_buffer[1, :13] = torch.tensor([0, 0.1, 8, 0, 0, 1, 0, 0, 0, -2, 0, 0, 0.8])
# physx_system.gpu_apply_body_data(body_data_buffer, bi, body_offset_buffer)

# print("should not change", body_data_buffer)

# physx_system.step()

# physx_system.gpu_query_body_data(body_data_buffer, bi, body_offset_buffer)
# print(body_data_buffer)

# physx_system._gpu_query_body_data_raw(body_data_buffer, bi)
# print(body_data_buffer)


# scene0.update_render()
# scene1.update_render()

# cam0 = scene0.add_camera("", 256, 256, 1, 0.01, 10)
# cam1 = scene1.add_camera("", 256, 256, 1, 0.01, 10)

# cam0.gpu_init()
# cam1.gpu_init()


# render_bodies0 = scene0.render_system.render_bodies
# render_bodies1 = scene1.render_system.render_bodies

# b0 = scene0.render_system.cuda_object_transforms
# b1 = scene1.render_system.cuda_object_transforms

# print(cam0.cuda_buffer)
# print(cam1.cuda_buffer)

# scene_dict = {scene0: 0, scene1: 1}

# scene_render_local_pose = [[], []]
# scene_render_index = [[], []]

# for physx_idx, a in enumerate(all_actors):
#     comp = a.find_component_by_type(sapien.render.RenderBodyComponent)
#     if comp is None:
#         continue

#     scene_idx = scene_dict[a.scene]

#     for shape in comp.render_shapes:
#         local_pose = shape.local_pose
#         gpu_idx = shape.get_gpu_transform_index()

#         scene_render_index[scene_idx].append([physx_idx, gpu_idx])
#         scene_render_local_pose[scene_idx].append(local_pose)
