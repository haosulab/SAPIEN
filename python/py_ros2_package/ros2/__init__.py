from ..core.pysapien_ros2.ros2 import *


def __init():
    import os
    init_spd_logger()
    set_ros2_logging_level("warning")
    resources_dir = os.path.abspath(os.path.dirname(__file__)).rstrip('/')
    set_resources_directory(resources_dir)
    print(resources_dir)


__init()
