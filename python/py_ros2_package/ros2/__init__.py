from ..core.pysapien_ros2.ros2 import *


def __init():
    import os
    init_spd_logger()
    set_ros2_logging_level("warning")
    set_resources_directory(os.path.abspath("./").strip('/'))
    print("Resources directory: {}".format(os.path.abspath("./").strip('/')))
