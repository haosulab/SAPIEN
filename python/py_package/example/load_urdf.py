#
# Copyright 2025 Hillbot Inc.
# Copyright 2020-2024 UCSD SU Lab
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import sapien
import numpy as np
from sapien.utils import Viewer


def main(filename, package_dir):
    sapien.physx.set_scene_config(gravity=[0, 0, 0])
    scene = sapien.Scene()
    scene.set_timestep(1 / 125)
    scene.set_ambient_light([0.4, 0.4, 0.4])
    scene.add_directional_light([1, -1, -1], [0.5, 0.5, 0.5])
    scene.add_point_light([2, 2, 2], [1, 1, 1])
    scene.add_point_light([2, -2, 2], [1, 1, 1])
    scene.add_point_light([-2, 0, 2], [1, 1, 1])

    viewer = scene.create_viewer()
    viewer.set_camera_xyz(-1, 0, 1)
    viewer.set_camera_rpy(0, -0.8, 0)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True

    robot = loader.load(filename, package_dir=package_dir)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "filename", type=str, help="Filename of the urdf you would like load."
    )
    parser.add_argument(
        "--package",
        type=str,
        default=None,
        help="used to resolve package:// for urdf assets",
    )
    args = parser.parse_args()
    main(args.filename, args.package)
