from robot.python.demonstration.recorder import Recorder
import sapyen_robot
import sys
import pickle
import numpy as np


def main():
    partnet_id = "45940"
    recorder = Recorder(partnet_id)
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
        save_file = "data/{}_v0.p".format(partnet_id)
        with open(save_file, 'wb') as f:
            pickle.dump(data, f)


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "recorder")
    main()
