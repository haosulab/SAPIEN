from robot.python.demonstration.replayer import Replayer
import numpy as np
import sys
import sapyen
import pickle
import transforms3d


def main():
    partnet_id: str = "44826"
    recorder = Replayer(partnet_id)

    data = {}

    recorder.renderer.show_window()
    add_arena_camera(recorder)
    camera_name = recorder.camera_name_list
    cloud_list = [[] for _ in range(len(camera_name))]
    try:
        for step in range(recorder.total_steps):
            recorder.step()
            if step % 5 == 0:
                recorder.renderer.render()

            if recorder.simulation_steps % 300 == 0:
                for i in range(len(camera_name)):
                    cloud_array, valid_index, _ = recorder.render_point_cloud(cam_id=i, rgba=True, segmentation=True,
                                                                              use_open3d=False)
                    cloud_list[i].append(cloud_array[valid_index, :])

    finally:
        save_file = "data/{}_v0_point_cloud.p".format(partnet_id)
        for i in range(len(camera_name)):
            print(cloud_list)
            data.update({"{}_cloud_array".format(camera_name[i]): np.stack(cloud_list, axis=0)})
        with open(save_file, 'wb') as f:
            pickle.dump(data, f)
        print("Save data for all {} cameras: {}".format(len(camera_name), camera_name))


def add_arena_camera(recorder: Replayer):
    # Back view
    pose = np.eye(4)
    pose[0:3, 3] = [0, 0, 2]
    pose[:3, :3] = transforms3d.euler.euler2mat(0, 0.5, 0)
    recorder.add_camera("back_view", pose, 640, 480)

    # Front view
    pose = np.eye(4)
    pose[0:3, 3] = [6, 0, 2]
    pose[:3, :3] = transforms3d.euler.euler2mat(0, 0.5, 3.141)
    recorder.add_camera("front_view", pose, 640, 480)

    # Left view
    pose = np.eye(4)
    pose[0:3, 3] = [3, 3, 2.5]
    pose[:3, :3] = transforms3d.euler.euler2mat(0, 0.5, -1.507)
    recorder.add_camera("left_view", pose, 640, 480)

    # Right view
    pose = np.eye(4)
    pose[0:3, 3] = [3, -3, 2.5]
    pose[:3, :3] = transforms3d.euler.euler2mat(0, 5, 1.507)
    recorder.add_camera("right_view", pose, 640, 480)

    # Bird view
    pose = np.eye(4)
    pose[0:3, 3] = [3, 0, 5]
    pose[:3, :3] = transforms3d.euler.euler2mat(0, 1.507, 0)
    recorder.add_camera("bird_view", pose, 640, 480)


if __name__ == '__main__':
    main()
