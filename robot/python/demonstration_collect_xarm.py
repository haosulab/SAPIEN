from robot.python.env.xarm_sapien_env import XArmRecorder
import sapyen_robot
import sys
import pickle
import numpy as np
import transforms3d
from robot.python.env.path_utils import *

CONVEX_PARTNET_DIR = "/home/sim/mobility_dataset/mobility_v1_alpha5"
door_with_handle = ['35059', '40147', '40417', '41003', '41004', '41083', '41085', '41086', '41452', '41510',
                    '41529', '44781', '44826', '45001', '45007', '45087', '45091', '45130', '45134', '45146',
                    '45159', '45162', '45164', '45166', '45168', '45173', '45176', '45177', '45189', '45194',
                    '45203', '45212', '45213', '45219', '45235', '45238', '45244', '45247', '45249', '45267',
                    '45271', '45297', '45305', '45332', '45354', '45372', '45378', '45384', '45385', '45387',
                    '45397', '45403', '45415', '45419', '45420', '45423', '45443', '45448', '45463', '45503',
                    '45504', '45505', '45523', '45524', '45526', '45573', '45575', '45594', '45600', '45606',
                    '45612', '45621', '45622', '45623', '45632', '45633', '45636', '45638', '45645', '45661',
                    '45662', '45667', '45670', '45671', '45676', '45687', '45689', '45690', '45693', '45694',
                    '45696', '45699', '45717', '45747', '45749', '45759', '45767', '45776', '45779', '45780',
                    '45783', '45784', '45790', '45850', '45853', '45908', '45915', '45916', '45922', '45936',
                    '45937', '45940', '45948', '45949', '45950', '45961', '45963', '45964', '45984', '46002',
                    '46019', '46029', '46033', '46037', '46044', '46045', '46084', '46092', '46108', '46117',
                    '46120', '46134', '46145', '46166', '46179', '46180', '46197', '46199', '46236', '46277',
                    '46401', '46408', '46417', '46427', '46430', '46452', '46456', '46480', '46490', '46598',
                    '46616', '46700', '46732', '46741', '46744', '46787', '46801', '46825', '46839', '46847',
                    '46856', '46859', '46874', '46889', '46906', '46922', '46944', '46955', '46966', '46981',
                    '47021', '47024', '47088', '47133', '47180', '47182', '47185', '47187', '47227', '47252',
                    '47254', '47278', '47281', '47290', '47315', '47388', '47419', '47514', '47529', '47570',
                    '47577', '47585', '47595', '47601', '47613', '47632', '47648', '47669', '47701', '47729',
                    '47742', '47747', '47808', '47817', '47853', '47926', '47944', '47976', '48018', '48023',
                    '48036', '48063', '48167', '48177', '48243', '48271', '48356', '48379', '48381', '48413',
                    '48452', '48467', '48490', '48513', '48519', '48623', '48700', '48721', '48797', '48859',
                    '48878', '49025', '49038', '49042', '49062', '49132', '49133', '49188']

vertical_door_handle_index = [1, 3, 6, 9, 11, 14, 15, 16, 19, 20, 21, 24, 26, 27, 28, 29, 32, 35, 36, 38, 40, 42, 43,
                              44, 45, 46, 47, 50, 52, 54]
duplicated = [(3, 6)]


def main():
    print(len(VALID_DOOR_INDEX))
    partnet_id = DOOR_WITH_HANDLE_LIST[54]
    print("*" * 10, "Partnet ID: {}".format(partnet_id), "*" * 10)
    recorder = XArmRecorder(dataset_dir=CONVEX_PARTNET_DIR, data_id=partnet_id, on_screening_rendering=True)
    target_pose = recorder.calculate_pose_in_front_of_semantics("rotation_door")
    recorder.set_robot_base_pose(target_pose)
    data = {}

    try:
        while 1:
            recorder.step()
    finally:
        data.update({"control": np.stack(recorder.control_signal)})
        data.update({"state": np.stack(recorder.dump_data)})
        data.update({"robot_link_force": np.stack(recorder.robot_force_array),
                     "object_link_force": np.stack(recorder.object_force_array)})
        data.update({"header": recorder.generate_header()})
        save_file = "data/{}_gripper_v1.p".format(partnet_id)
        with open(save_file, 'wb') as f:
            pickle.dump(data, f)


def replay():
    partnet_id: str = DOOR_WITH_HANDLE_LIST[29]
    all_data = np.load(f"data/{partnet_id}_gripper_v1.p", allow_pickle=True)
    data = all_data['state']
    recorder = XArmRecorder(dataset_dir=CONVEX_PARTNET_DIR, data_id=partnet_id, on_screening_rendering=True)

    recorder.renderer.show_window()
    camera_name = recorder.camera_name_list
    cloud_list = [[] for _ in range(len(camera_name))]
    total_steps = data.shape[0]
    recorder.ps3.set_replay_mode()

    for _ in range(8000):
        recorder.sim.pack(data[1])
        recorder.step()

    for step in range(total_steps):
        recorder.step()
        recorder.sim.pack(data[step])


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "recorder")
    main()
    # replay()
