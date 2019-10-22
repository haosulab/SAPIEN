from robot.python.demonstration.replayer import Replayer
import numpy as np
import sys
import pickle
import transforms3d


def main():
    partnet_id: str = "45940"
    recorder = Replayer(partnet_id)
    recorder.init_camera()
    pose = np.eye(4)
    pose[0:3, 3] = [0, 0, 2]
    pose[:3, :3] = transforms3d.euler.euler2mat(0, 0.5, 0)
    recorder.add_camera("test", pose, 640, 480)
    data = {}
    cloud_list = []
    recorder.renderer.show_window()
    try:
        while 1:
            recorder.step()
            recorder.renderer.render()
            cloud_array, valid_index, _ = recorder.render_point_cloud(cam_id=1, rgb=True, use_open3d=False)
            # cloud_array, valid_index, open3d_cloud recorder.render_point_cloud(cam_id=1, rgb=True, use_open3d=True)
            cloud_list.append(cloud_array[valid_index, :])
    finally:
        save_file = "data/{}_v0_point_cloud.p".format(partnet_id)
        data.update({"camera1_cloud_array": np.stack(cloud_list, axis=0)})
        with open(save_file, 'wb') as f:
            pickle.dump(data, f)


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "recorder")
    main()
