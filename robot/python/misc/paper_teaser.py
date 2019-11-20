from robot.python.env.movo_env import MOVOEnv, MOVOFreeBaseEnv
from robot.python.env.base_env import BaseEnv
import sapyen
import numpy as np
import sapyen_robot
import pickle
import transforms3d


class MOVORecorder(MOVOFreeBaseEnv, BaseEnv):
    def __init__(self, on_screening_rendering: bool = True):
        BaseEnv.__init__(self, on_screening_rendering)
        self._init_robot()

        self.ps3 = sapyen_robot.MOVOFreeBasePS3(self.manger)
        self.ps3.set_demonstration_mode()

        # Init
        self.dump_data = []
        self.control_signal = []

    def step(self):
        self._step()
        self.ps3.step()

        # Cache
        if self.ps3.start_record():
            print("Recording")
            self.control_signal.append(self.ps3.get_cache())
            self.dump_data.append(self.sim.dump())

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        return header


def load_scene(sim: sapyen.Simulation):
    builder: sapyen.ActorBuilder = sim.create_actor_builder()
    global_scale = 1
    builder.add_multiple_shapes("../assets/object/walls/wall_scene.obj", sapyen.Pose([0, 0, 0]),
                                [global_scale, global_scale, global_scale])
    builder.add_obj_visual("../assets/object/walls/wall_scene.obj", sapyen.Pose([0, 0, 0]),
                           [global_scale, global_scale, global_scale])
    builder.build(True, False, "scene")
    loader: sapyen.URDFLoader = sim.create_urdf_loader()
    loader.fix_loaded_object = True

    loader.scale = global_scale
    obj1: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/102044/mobility.urdf")
    obj1.set_root_pose(np.array([1.221, 1.244, 0.614]) * global_scale, [1, 0, 0, 0])

    loader.scale = global_scale * 1.4
    obj2: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/10905/mobility.urdf")
    obj2.set_root_pose(np.array([0.9, -0.954, 0.622]) * global_scale, [1, 0, 0, 0])

    loader.scale = global_scale * 0.8
    obj3: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/12065/mobility.urdf")
    obj3.set_root_pose(np.array([1.12, 0.109, 0.582]) * global_scale, [1, 0, 0, 0])
    obj3.set_pd(20000, 3000, 2000, [1])

    loader.scale = global_scale * 1
    obj4: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/23511/mobility.urdf")
    obj4.set_root_pose(np.array([-2.246, -3.518, 0.910]) * global_scale,
                       transforms3d.quaternions.axangle2quat([0, 0, 1], 3.14159))

    loader.scale = global_scale * 1
    obj5: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/45594/mobility.urdf")
    obj5.set_root_pose(np.array([1.271, 2.393, 0.946]) * global_scale)

    loader.scale = global_scale * 1
    noj6: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/46037/mobility.urdf")
    noj6.set_root_pose(np.array([0.597, -3.789, 0.774]) * global_scale,
                       transforms3d.quaternions.axangle2quat([0, 0, 1], -1.5708))

    loader.scale = global_scale * 0.4
    obj7: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/7310/mobility.urdf")
    obj7.set_root_pose(np.array([1.195, 0.847, 1.259]) * global_scale,
                       transforms3d.quaternions.axangle2quat([0, 0, 1], -0.1))

    loader.scale = global_scale * 1.5
    obj8: sapyen.ArticulationWrapper = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/8867/mobility.urdf")
    obj8.set_root_pose(np.array([-3.127, 2.281, 1.481]) * global_scale)


def main():
    recorder = MOVORecorder()
    recorder.robot.set_root_pose([-0.3, 0, 0.06], [1, 0, 0, 0])
    color = [4, 0.773 * 4, 0.561 * 4]
    # recorder.renderer.add_point_light([0, 0, 3.6], color)
    # recorder.renderer.add_point_light([-0.2, -4, 4.2], color)
    # recorder.renderer.add_point_light([0.3, -2, 3.7], color)
    recorder.renderer.add_directional_light([1, 0, -1], [1, 1, 1])
    # self.renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
    load_scene(recorder.sim)

    while True:
        data = {}

        try:
            while 1:
                recorder.step()
        finally:
            data.update({"control": np.stack(recorder.control_signal)})
            data.update({"state": np.stack(recorder.dump_data)})
            data.update({"header": recorder.generate_header()})
            save_file = "data/paper_demo.p"

            # Save image
            all_pc = recorder.render_point_cloud(0, xyz=True, rgba=True, segmentation=True, normal=True,
                                                 world_coordinate=False)
            depth = recorder.cam_list[0].get_depth()
            data.update({"all_point_cloud": all_pc, "depth": depth})
            with open(save_file, 'wb') as f:
                pickle.dump(data, f)


def replay():
    from PIL import Image
    recorder = MOVORecorder()
    recorder.robot.set_root_pose([-0.3, 0, 0.06], [1, 0, 0, 0])
    color = [4, 0.773 * 4, 0.561 * 4]
    recorder.renderer.add_point_light([0, 0, 3.6], color)
    recorder.renderer.add_point_light([-0.2, -4, 4.2], color)
    recorder.renderer.add_point_light([0.3, -2, 3.7], color)
    load_scene(recorder.sim)

    data = np.load("data/paper_demo.p", allow_pickle=True)
    state = data["state"][0, :]

    step = 0
    while True:
        recorder.sim.pack(state)
        recorder.step()
        step += 1
        # if :
        #     normal = recorder.render_point_cloud(0, xyz=False, rgba=False, segmentation=False, normal=True,
        #                                          world_coordinate=False)
        #     Image.fromarray(np.abs((normal[:, :, :3]) * 255).astype(np.uint8)).save("data/normal.png")
        #     break


if __name__ == '__main__':
    import sys

    sapyen_robot.ros.init(sys.argv, "paper_demo")
    main()
    # replay()
