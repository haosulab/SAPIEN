import sapyen
import json
import sys
from tifffile import imwrite
from sapyen import Pose
import numpy as np
import os
import transforms3d
from PIL import Image
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('folder', type=str)
parser.add_argument('--output', type=str, required=True)
parser.add_argument('--minyaw', type=float, default=0)
parser.add_argument('--maxyaw', type=float, default=360)
parser.add_argument('--minpitch', type=float, default=20)
parser.add_argument('--maxpitch', type=float, default=40)
parser.add_argument('--yawcount', type=int, default=10)
parser.add_argument('--pitchcount', type=int, default=2)
parser.add_argument('--width', type=int, default=256)
parser.add_argument('--height', type=int, default=256)
args = parser.parse_args()
annoid = os.path.basename(args.folder)


def rand_cam_pose(rlow, rhigh, plow, phigh, ylow=0, yhigh=360):
    plow *= np.pi / 180
    phigh *= np.pi / 180
    ylow *= np.pi / 180
    yhigh *= np.pi / 180

    yaw = np.random.rand() * (yhigh - ylow) + ylow
    pitch = np.random.rand() * (phigh - plow) + plow
    r = np.random.rand() * (rhigh - rlow) + rlow

    x = r * np.cos(pitch) * np.cos(yaw)
    y = r * np.cos(pitch) * np.sin(yaw)
    z = r * np.sin(pitch)

    x2 = -np.array([x, y, z])
    x2 /= np.linalg.norm(x2)

    y2 = np.cross(np.array([0, 0, 1]), x2)
    y2 /= np.linalg.norm(y2)

    z2 = np.cross(x2, y2)

    quat = transforms3d.quaternions.mat2quat(np.array([x2, y2, z2]).T)
    return Pose([x, y, z], quat)


def rand_qpos(low, high):
    return np.random.rand() * (high - low) + low


poses = []
dyaw = (args.maxyaw - args.minyaw) / args.yawcount
for i in range(args.yawcount):
    dpitch = (args.maxpitch - args.minpitch) / args.pitchcount
    for j in range(args.pitchcount):
        poses.append(
            rand_cam_pose(
                2,
                3,
                args.minpitch + j * dpitch,
                args.minpitch + (j + 1) * dpitch,
                args.minyaw + i * dyaw,
                args.minyaw + (i + 1) * dyaw,
            ))

urdf = os.path.join(args.folder, 'mobility.urdf')
print(urdf, file=sys.stderr)

semantics = os.path.join(args.folder, 'semantics.txt')
link2semantics = {}
link2motion = {}
with open(semantics, 'r') as f:
    for line in f:
        if line.strip():
            link, motion, semantics = line.split()
            link2semantics[link] = semantics
            link2motion[link] = motion

renderer = sapyen.OptifuserRenderer()
renderer.cam.set_position(np.array([0, -2, 1]))
renderer.cam.rotate_yaw_pitch(0, -0.5)

renderer.set_ambient_light([.4, .4, .4])
renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
renderer.add_point_light([2, 2, 2], [1, 1, 1])
renderer.add_point_light([2, -2, 2], [1, 1, 1])
renderer.add_point_light([-2, 0, 2], [1, 1, 1])

sim = sapyen.Simulation()
sim.set_renderer(renderer)
sim.set_time_step(1.0 / 200.0)

loader = sim.create_urdf_loader()
wrapper = loader.load(urdf)
if wrapper is None:
    print('fall back to kinematic articulation')
    wrapper = loader.load_kinematic(urdf)

builder = sim.create_actor_builder()
mount = builder.build(False, True, "Camera Mount")
cam = sim.add_mounted_camera("Floating Camera", mount, Pose([0, 0, 0], [1, 0, 0, 0]), args.width, args.height,
                             1.22172944444, 1.22172944444, 0.01, 100)

zeros = np.zeros(wrapper.dof())
wrapper.set_qvel(zeros)
wrapper.set_qpos(zeros)
# wrapper.set_qf(zeros)

limits = wrapper.get_joint_limits()
limits = [[max(-100, l), min(100, h)] for l, h in limits]
cam0 = renderer.get_camera(0)

link2id = dict(zip(wrapper.get_link_names(), wrapper.get_link_ids()))
id2name = dict(zip(wrapper.get_link_ids(), wrapper.get_link_names()))
id2semantic = [[i, link2motion[id2name[i]], link2semantics[id2name[i]]] for i in id2name
               if id2name[i] in link2semantics]

with open(f'{args.output}/{annoid}_semantics.txt', 'w') as f:
    for item in id2semantic:
        f.write(' '.join([str(i) for i in item]))
        f.write('\n')

joint_names = wrapper.get_joint_names()
jname2qidx = {}
link2qidx = {}
idx = 0
for dof, jname in zip(wrapper.get_joint_dofs(), joint_names):
    if dof > 0:
        jname2qidx[jname] = idx
        link2qidx['link_' + jname.split('_')[1]] = idx
        idx += 1

link2lidx = dict((l, i) for i, l in enumerate(wrapper.get_link_names()))

for idx, pose in enumerate(poses):

    mount.set_global_pose(pose)
    p = [rand_qpos(l, h) for l, h in limits]
    wrapper.set_qvel(zeros)
    wrapper.set_qpos(p)
    sim.step()
    sim.step()

    sim.update_renderer()
    cam0 = renderer.get_camera(0)
    cam0.take_picture()
    Image.fromarray(
        (cam0.get_color_rgba() * 255).astype(np.uint8)).save(f'{args.output}/{annoid}_{idx}_rgba.png')
    imwrite(f'{args.output}/{annoid}_{idx}_normal.tif', cam0.get_normal_rgba(), compress=6)
    depth = cam0.get_depth()
    imwrite(f'{args.output}/{annoid}_{idx}_depth.tif', depth, compress=6, photometric='minisblack')
    seg = cam0.get_segmentation()
    imwrite(f'{args.output}/{annoid}_{idx}_segmentation.tif', seg, compress=6, photometric='minisblack')

    config = {
        "camera": {
            "intrinsic": list(cam0.get_camera_matrix().reshape(-1).astype(float)),
            "extrinsic": list(cam0.get_model_mat().reshape(-1).astype(float))
        },
    }

    true_limits = wrapper.get_joint_limits()
    qpos = wrapper.get_qpos()
    config["links"] = {}
    for l in link2semantics:
        if l in link2lidx:
            pose = wrapper.get_link_joint_pose(link2lidx[l])
            direction = transforms3d.quaternions.quat2mat(pose.q) @ np.array([1, 0, 0])
            origin = pose.p

        config['links'][l] = {
            "id": link2id[l],
            "semantic": link2semantics[l],
            "motion": link2motion[l],
            "limit": list(true_limits[link2qidx[l]].astype(float)) if l in link2qidx else None,
            "origin": list(origin.astype(float)) if l in link2lidx else None,
            "direction": list(direction.astype(float)) if l in link2lidx else None,
            "qpos": float(qpos[link2qidx[l]]) if l in link2qidx else None
        }

    with open(f'{args.output}/{annoid}_{idx}_config.json', 'w') as f:
        json.dump(config, f)

renderer.show_window()
while True:
    sim.step()
    sim.update_renderer()
    renderer.render()
    depth = cam0.get_depth()
