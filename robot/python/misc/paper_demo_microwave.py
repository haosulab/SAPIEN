from robot.python.env.movo_env import MOVOEnv, MOVOFreeBaseEnv
from robot.python.env.base_env import BaseEnv
from robot.python.env.xarm_env import XArmEnv
from robot.python.env.single_gripper_env import SingleGripperBaseEnv
import os
from robot.python.env.base_env import BaseEnv
import sapyen
import numpy as np
import sapyen_robot
import pickle
import transforms3d
from PIL import Image
from time import time
import matplotlib.pyplot as plt

data_path = "/home/sim/mobility_dataset/mobility_convex_alpha5"


class MOVORecorder(MOVOFreeBaseEnv, BaseEnv):
    def __init__(self, index: str, on_screening_rendering: bool = True):
        BaseEnv.__init__(self, on_screening_rendering)
        self._init_robot()

        self.ps3 = sapyen_robot.MOVOFreeBasePS3(self.manger)
        self.ps3.set_demonstration_mode()

        # Init
        self.recording_index = 0
        self.index = index
        self.recording = False
        self.saves = []
        self.last_saved_step = None

    def step(self):
        self._step()
        self.ps3.step()

        if self.ps3.record_current_step():
            self.saves = []
            print("Start record")
            self.recording = True
            if self.last_saved_step:
                self.sim.pack(self.last_saved_step)

        # Stop recording
        if self.ps3.start_record():
            print("Stop record")
            self.recording = False
            self.last_saved_step = self.sim.dump()
            self.save()
            self.recording_index += 1
            self.saves = []

        if self.recording:
            self.saves.append(self.sim.dump())

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        return header

    def save(self):
        if self.saves:
            data = {"header": self.generate_header()}
            data.update({"state": self.saves})
            save_file = f"data/demo_{self.index}_{self.recording_index}.p"
            with open(save_file, 'wb') as f:
                pickle.dump(data, f)


def load_scene(sim: sapyen.Simulation):
    builder: sapyen.ActorBuilder = sim.create_actor_builder()
    global_scale = 1
    builder.add_multiple_shapes("../assets/object/walls/wall_scene.obj", sapyen.Pose([0, 0, 0]),
                                [global_scale, global_scale, global_scale])
    builder.add_obj_visual("../assets/object/walls/wall_scene.obj", sapyen.Pose([0, 0, 0]),
                           [global_scale, global_scale, global_scale])
    builder.build(True, False, "scene")

    loader = sim.create_urdf_loader()
    loader.fix_loaded_object = True
    loader.scale = global_scale * 1.5
    obj8: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/8867/mobility.urdf")
    obj8.set_root_pose(np.array([-3.127, 2.281, 1.481]) * global_scale)


def load_all_env(recorder, mobility_index):
    load_scene(recorder.sim)
    # Load rendering
    # color = [4, 0.773 * 4, 0.561 * 4]
    color = [3, 3, 3]
    # recorder.renderer.add_directional_light([1, 0.5, -2], [0.2, 0.2, 0.2])
    recorder.renderer.add_point_light([0, 0, 3.6], color)
    recorder.renderer.add_point_light([-0.2, -4, 4.2], color)
    recorder.renderer.add_point_light([0.3, -2, 3.7], color)

    builder: sapyen.ActorBuilder = recorder.sim.create_actor_builder()
    # builder.add_multiple_shapes("/home/sim/mobility_dataset/hotdog.dae")
    # builder.build().set_global_pose(sapyen.Pose([1.0, 0, 2]))

    loader = recorder.sim.create_urdf_loader()
    loader.fix_loaded_object = False
    loader.scale = 0.8
    table: sapyen.ArticulationWrapper = loader.load(os.path.join(data_path, str(mobility_index), "mobility.urdf"))
    table.set_root_pose([0.8, -3, 1])

    loader.scale = 0.4
    microwave: sapyen.ArticulationWrapper = loader.load(
        "/home/sim/mobility_dataset/mobility_v1_alpha5/7128/mobility.urdf")
    microwave.set_root_pose([0.8, -3, 1.5])
    microwave.set_qf([0, 0, 0, 0, -1])
    microwave.set_pd(10000, 1000, 10, [3])
    microwave.set_qpos([0, 0, 0, 0, 0])

    return microwave


def main():
    index: str = "20279"
    recorder = MOVORecorder(index)
    recorder.robot.set_root_pose([-2, -3, 0.06], [1, 0, 0, 0])
    microwave = load_all_env(recorder, index)
    locked = False

    while True:
        try:
            while 1:
                recorder.step()
                if microwave.get_qpos()[3] >= 0.004 and locked:
                    print("Unlock!")
                    locked = False
                    microwave.set_qvel([0, 0, 0, 0, 0.1])
                    microwave.set_qf([0, 0, 0, 0, 0])
                if not locked:
                    if microwave.get_qpos()[4] <= 0.01:
                        locked = True
        finally:
            recorder.save()


def replay():
    index: str = "20279"
    # recorder = MOVORecorder(index, False)
    recorder = BaseEnv(True)
    robot = recorder.loader.load("../assets/robot/movo_free_base.urdf")
    load_all_env(recorder, index)
    load_camera(recorder)
    data_files = os.listdir("./data")
    data_files = [file for file in data_files if file.startswith(f"demo_{index}")]
    data_files = sorted(data_files, key=lambda x: int(x.split(".")[0].split("_")[2]))
    print(data_files)
    whole_trajectory = []
    for data_file in data_files:
        whole_trajectory.extend(np.load(os.path.join("data", data_file), allow_pickle=True)["state"])

    step_num = 0
    for name in recorder.camera_name_list:
        os.makedirs(f"video/{index}/{name}", exist_ok=True)
    for single_step in whole_trajectory:
        recorder.sim.pack(single_step)
        recorder._step()
        if step_num % 30 == 0 and step_num > 1000:
            tic = time()
            name_size = step_num // 30
            for j in range(len(recorder.cam_list)):
                recorder.cam_list[j].take_picture()
                photo = recorder.cam_list[j].take_raytraced_picture(256)[:, :, :3]
                photo = np.power(photo, 1 / 2.2)
                photo = (np.clip(photo, 0, 1) * 255).astype(np.uint8)
                Image.fromarray(
                    photo).save(os.path.join(f"video/{index}/{recorder.camera_name_list[j]}/{name_size:04}.png"))
            print(f"Using {time() - tic}s to render a frame")
        step_num += 1


def load_camera(recorder: BaseEnv):
    camera_pose = sapyen.Pose([-0.5, -1.5, 2.5], transforms3d.euler.euler2quat(0, 0.7, -0.9))
    recorder.add_camera("left_view", camera_pose, width=960, height=540)
    camera_pose = sapyen.Pose([0.7, -3, 2.5], transforms3d.euler.euler2quat(0, 1.3, 3.14))
    recorder.add_camera("front_view", camera_pose, width=960, height=540)
    camera_pose = sapyen.Pose([0.5, -3, 3], transforms3d.euler.euler2quat(0, 1.57, 0))
    recorder.add_camera("bird_view", camera_pose, width=960, height=540)


if __name__ == '__main__':
    import sys

    sapyen_robot.ros.init(sys.argv, "paper_demo")
    # main()
    replay()
