import sapien.core as sapien
from sapien.utils.viewer import Viewer
import numpy as np


class SimplePID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.p = kp
        self.i = ki
        self.d = kd

        self._cp = 0
        self._ci = 0
        self._cd = 0

        self._last_error = 0

    def compute(self, current_error, dt):
        self._cp = current_error
        self._ci += current_error * dt
        self._cd = (current_error - self._last_error) / dt
        self._last_error = current_error
        signal = (self.p * self._cp) + \
            (self.i * self._ci) + (self.d * self._cd)
        return signal


def pid_forward(pids: list,
                target_pos: np.ndarray, 
                current_pos: np.ndarray, 
                dt: float) -> np.ndarray:
    errors = target_pos - current_pos
    qf = [pid.compute(error, dt) for pid, error in zip(pids, errors)]
    return np.array(qf)


def demo(use_internal_drive, use_external_pid):
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    # A small timestep for higher control accuracy
    scene.set_timestep(1 / 2000.0)
    scene.add_ground(0)


    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2, y=0, z=1)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)

    # Load URDF
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot: sapien.Articulation = loader.load("../assets/robot/jaco2/jaco2.urdf")
    robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))

    # Set joint positions
    arm_zero_qpos = [0, 3.14, 0, 3.14, 0, 3.14, 0]
    gripper_init_qpos = [0, 0, 0, 0, 0, 0]
    zero_qpos = arm_zero_qpos + gripper_init_qpos
    robot.set_qpos(zero_qpos)
    arm_target_qpos = [4.71, 2.84, 0.0, 0.75, 4.62, 4.48, 4.88]
    target_qpos = arm_target_qpos + gripper_init_qpos

    active_joints = robot.get_active_joints()
    # Or other equivalent way to get active joints
    # active_joints = [joint for joint in robot.get_joints() if joint.get_dof() > 0]

    if use_internal_drive:
        for joint_idx, joint in enumerate(active_joints):
            joint.set_drive_property(stiffness=20, damping=5)
            joint.set_drive_target(target_qpos[joint_idx])
        # Or you can directly set joint targets for an articulation
        # robot.set_drive_target(target_qpos)

    if use_external_pid:
        pids = []
        pid_parameters = [
            (40, 5, 2), (40, 5, 2), (40, 5, 2), (20, 5.0, 2),
            (5, 0.8, 2), (5, 0.8, 2), (5, 0.8, 0.4),
            (0.1, 0, 0.02), (0.1, 0, 0.02), (0.1, 0, 0.02),
            (0.1, 0, 0.02), (0.1, 0, 0.02), (0.1, 0, 0.02),
        ]
        for i, joint in enumerate(active_joints):
            pids.append(SimplePID(*pid_parameters[i]))

    camera = create_camera(scene, [-2, 0, 1], [0, -0.3, 0])
    import imageio
    images = []

    steps = 0
    total_steps = 4000 if use_internal_drive else 4000
    render_freq = 4 * 5
    while steps < total_steps:
        for _ in range(render_freq):
            qf = robot.compute_passive_force(
                gravity=True,
                coriolis_and_centrifugal=True,
            )
            if use_external_pid:
                pid_qf = pid_forward(
                    pids,
                    target_qpos,
                    robot.get_qpos(),
                    scene.get_timestep()
                )
                qf += pid_qf
            robot.set_qf(qf)
            scene.step()
            steps += 1

        scene.update_render()
        viewer.render()

        camera.take_picture()
        rgba = camera.get_float_texture('Color')  # [H, W, 4]
        rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
        images.append(rgba_img)

        if steps % 100 == 0:
            print(steps)

    viewer.close()
    if use_internal_drive:
        filename = 'pid_internal.gif'
    elif use_external_pid:
        filename = 'pid_external.gif'
    else:
        raise NotImplementedError()
    imageio.mimsave(filename, images, fps=25)
    from pygifsicle import optimize
    optimize(filename)


def create_camera(scene, xyz, rpy):
    from sapien.core import Pose
    from transforms3d.euler import euler2quat
    cam_mount_actor = scene.create_actor_builder().build_kinematic()
    cam_pose = Pose(xyz, euler2quat(rpy[0], -rpy[1], -rpy[2]))
    camera = scene.add_mounted_camera(
        'recorder',
        cam_mount_actor,
        cam_pose,
        width=1280,
        height=720,
        fovx=1.0,
        fovy=1.0,
        near=0.001,
        far=100,
    )
    return camera


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--use-internal-drive', action='store_true')
    parser.add_argument('--use-external-pid', action='store_true')
    args = parser.parse_args()

    demo(use_internal_drive=args.use_internal_drive,
         use_external_pid=args.use_external_pid)


if __name__ == '__main__':
    main()
