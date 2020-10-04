import sapyen
import sapyen_robot
import sys
import os

PARTNET_DIR = '/home/sim/project/mobility-v0-prealpha3/mobility_verified'


class Recorder:
    def __init__(self, partnet_id: str):
        # Rendering
        self.renderer = sapyen.OptifuserRenderer()
        self.renderer.set_ambient_light([.4, .4, .4])
        self.renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
        self.renderer.add_point_light([2, 2, 2], [1, 1, 1])
        self.renderer.add_point_light([2, -2, 2], [1, 1, 1])
        self.renderer.add_point_light([-2, 0, 2], [1, 1, 1])
        self.renderer.cam.set_position([0.5, -4, 0.5])
        self.renderer.cam.set_forward([0, 1, 0])
        self.renderer.cam.set_up([0, 0, 1])

        # Simulation
        self.sim = sapyen.Simulation()
        self.sim.set_renderer(self.renderer)
        self.sim.set_time_step(1.0 / 200)
        self.sim.add_ground(0, material=None)

        # Partnet Object
        urdf = os.path.join(PARTNET_DIR, partnet_id, "mobility.urdf")
        self.loader = self.sim.create_urdf_loader()
        self.obj = self.loader.load(urdf)
        self.obj.set_root_pose([3, 0, 0.5], [1, 0, 0, 0])

        # Robot Model and controller manger
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = True
        self.robot = self.loader.load('../assets/robot/all_robot.urdf')
        self.robot.set_drive_qpos([0.25, -1.9347, 0, -1.5318, 0, 0.9512, -2.24, 0.34, 0.64, -1.413, 0, 0, 0])
        self.robot.set_pd(5000, 1500, 50000)
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
        self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)
        self.ps3 = sapyen_robot.MOVOPS3(self.manger)

        # Init
        self.renderer.show_window()
        self.ps3.set_demonstration_mode()
        self.dump_data = []
        self.control_signal = []
        self.object_force_array = []
        self.robot_force_array = []

    def step(self):
        self.sim.step()
        self.sim.update_renderer()
        self.renderer.render()
        self.ps3.step()

        # Cache
        self.control_signal.append(self.ps3.get_cache())
        self.dump_data.append(self.sim.dump())
        self.object_force_array.append(self.obj.get_cfrc_ext())
        self.robot_force_array.append(self.robot.get_cfrc_ext())

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        header.update({"object_joint_name": self.obj.get_joint_names()})
        header.update({"object_link_name": self.obj.get_link_names()})
        return header
