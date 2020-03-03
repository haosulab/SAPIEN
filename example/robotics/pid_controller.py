import pysapien as sapien


class SimplePID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.p = kp
        self.i = kp
        self.d = kp

        self._cp = 0
        self._ci = 0
        self._cd = 0

        self._last_error = 0

    def compute(self, current_error, dt):
        d_error = current_error - self._last_error

        self._cp = current_error
        self._ci += current_error * dt
        self._cd = d_error / dt

        self._last_error = current_error
        signal = (self.p * self._cp) + (self.i * self._ci) + (self.d * self._cd)
        return signal


def robot_basic_control_demo(fix_robot_root, balance_passive_force, customer_force_control):
    sim = sapien.Simulation()
    renderer = sapien.OptifuserRenderer()
    sim.set_renderer(renderer)
    renderer_controller = sapien.OptifuserController(renderer)
    renderer_controller.set_camera_position(-5, 0, 0)

    scene0 = sim.create_scene(gravity=[0, 0, -9.81])
    renderer_controller.set_current_scene(scene0)
    scene0.add_ground(altitude=0)
    scene0.set_timestep(1 / 240)
    scene0.set_ambient_light([0.5, 0.5, 0.5])
    scene0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

    loader = scene0.create_urdf_loader()
    loader.fix_root_link = 0
    robot: sapien.Articulation = loader.load("assets/robot/jaco3.urdf")
    arm_init_qpos = [4.71, 2.84, 0, 0.75, 4.62, 4.48, 4.88]
    gripper_init_qpos = [0, 0, 0, 0, 0, 0]
    init_qpos = arm_init_qpos + gripper_init_qpos
    robot.set_qpos(init_qpos)

    steps = 0
    renderer_controller.show_window()
    while not renderer_controller.should_quit:
        scene0.update_render()
        for i in range(4):
            qf = robot.compute_passive_force(gravity=True, coriolisAndCentrifugal=True, external=True)
            robot.set_qf(qf)
            scene0.step()
            steps += 1
        renderer_controller.render()


if __name__ == '__main__':
    robot_basic_control_demo()
