from ..core.pysapien_ros2.ros2 import *
import os

__CURRENT_PATH = os.path.abspath(os.path.dirname(__file__))
__LD_PATH = os.path.abspath(os.path.join(__CURRENT_PATH, "../../sapien_robot.libs"))
if not os.environ["LD_LIBRARY_PATH"]:
    os.environ["LD_LIBRARY_PATH"] = __LD_PATH
else:
    if not os.environ["LD_LIBRARY_PATH"].endswith(':'):
        os.environ["LD_LIBRARY_PATH"] += ":"
    os.environ["LD_LIBRARY_PATH"] += __LD_PATH
print("Current LD_LIBRARY_PATH when init SAPIEN ROS2 module is: {}".format(os.environ["LD_LIBRARY_PATH"]))


def __init():
    import os
    init_spd_logger()
    set_ros2_logging_level("warning")
    resources_dir = os.path.abspath(os.path.dirname(__file__)).rstrip('/')
    set_resources_directory(resources_dir)
    print(resources_dir)


__init()
