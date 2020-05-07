import sapien.core as sapien
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
        d_error = current_error - self._last_error

        self._cp = current_error
        self._ci += current_error * dt
        if abs(self._last_error) > 0.01:
            self._cd = d_error / dt

        self._last_error = current_error
        signal = (self.p * self._cp) + (self.i * self._ci) + (self.d * self._cd)
        return signal


def pid_forward(pids: list, target_pos: np.ndarray, current_pos: np.ndarray, dt: float) -> np.ndarray:
    qf = np.zeros(len(pids))
    errors = target_pos - current_pos
    for i in range(len(pids)):
        qf[i] = pids[i].compute(errors[i], dt)
    return qf


def robot_basic_control_demo(use_internal_pd, use_external_pid):
    sim = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    renderer.enable_global_axes(False)
    sim.set_renderer(renderer)
    renderer_controller = sapien.OptifuserController(renderer)
    renderer_controller.set_camera_position(-2, 0, 1)
    renderer_controller.set_camera_rotation(0, -0.5)

    scene0 = sim.create_scene(gravity=[0, 0, -9.81])
    renderer_controller.set_current_scene(scene0)
    scene0.add_ground(altitude=0)
    scene0.set_timestep(1 / 2000)
    scene0.set_ambient_light([0.5, 0.5, 0.5])
    scene0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

    loader = scene0.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load("assets/robot/jaco2.urdf")

    arm_init_qpos = [4.71, 2.84, 0, 0.75, 4.62, 4.48, 4.88]
    arm_zero_qpos = [0, 3.14, 0, 3.14, 0, 3.14, 0]
    gripper_init_qpos = [0, 0, 0, 0, 0, 0]
    target_pos = np.array(arm_init_qpos + gripper_init_qpos)
    zero_qpos = arm_zero_qpos + gripper_init_qpos
    robot.set_qpos(zero_qpos)

    pids = []
    pid_parameters = [(40, 5, 2), (40, 5.0, 2), (40, 5.0, 2), (20, 5.0, 2), (5, 0.8, 2), (5, 0.8, 2),
                      (5, 0.8, 0.4), (0.1, 0, 0.02), (0.1, 0, 0.02), (0.1, 0, 0.02), (0.1, 0, 0.02), (0.1, 0, 0.02),
                      (0.1, 0, 0.02)]
    active_joints = [joint for joint in robot.get_joints() if joint.get_dof() > 0]
    for i, joint in enumerate(active_joints):
        if use_internal_pd:
            joint.set_drive_property(stiffness=5, damping=5)
        if use_external_pid:
            pids.append(SimplePID(pid_parameters[i][0], pid_parameters[i][1], pid_parameters[i][2]))
    robot.set_drive_target(target_pos)

    steps = 0
    renderer_controller.show_window()
    while not renderer_controller.should_quit:
        scene0.update_render()
        for i in range(4):
            qf = robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True, external=True)
            if use_external_pid:
                pid_qf = pid_forward(pids, target_pos, robot.get_qpos(), scene0.get_timestep())
                qf += pid_qf
            robot.set_qf(qf)
            scene0.step()
            steps += 1
        renderer_controller.render()

    scene0 = None


if __name__ == '__main__':
    robot_basic_control_demo(False, True)
