import sapyen
import sapyen_robot
from .base_env import BaseEnv


class SingleGripperEnv(BaseEnv):
    def __init__(self):
        super(SingleGripperEnv, self).__init__()
        self._load_robot()
        self._prepare_controller()

    def _load_robot(self):
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = True
        gripper_material = self.sim.create_material(3.1, 2.1, 0.01)
        self.robot = self.loader.load('../assets/robot/single_hand.urdf', gripper_material)

        # Link mapping
        links = self.robot.get_links()
        link_names = self.robot.get_link_names()
        self.robot_name2link = {link_names[i]: links[i] for i in range(len(link_names))}

    def _prepare_controller(self):
        # Controllers
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
        self.gripper_joint = ["right_gripper_finger1_joint", "right_gripper_finger2_joint",
                              "right_gripper_finger3_joint"]
        self.translation_joint = ["x_axis_joint", "y_axis_joint",
                                  "z_axis_joint"]
        self.rotation_joint = ["r_rotation_joint", "p_rotation_joint",
                               "y_rotation_joint"]
        self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)
        self.gripper_controller = self.manger.create_joint_velocity_controller(self.gripper_joint, "gripper")
        self.translation_controller = self.manger.create_joint_velocity_controller(self.translation_joint,
                                                                                   "translation")
        self.rotation_controller = self.manger.create_joint_velocity_controller(self.rotation_joint, "rotation")
        # Init robot pose and controller
        self.robot.set_pd(200, 40, 20, [0, 1, 2, 3, 4, 5])
        self.robot.set_pd(1, 0.05, 1, [6, 7, 8])
        self.init_qpos = [0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.sim.step()

        # Note that you should always start the manger before use any ROS utility
        # other wise the spinner will not going to process the thread callback
        self.manger.start()
