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
"""Test offscreen renderer for SAPIEN
"""
import sapien as sapien
import numpy as np
from PIL import Image


def main():
    sapien.render.set_log_level("info")
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)  # Set the simulation frequency

    scene.add_ground(altitude=0)  # Add a ground
    actor_builder = scene.create_actor_builder()
    actor_builder.add_box_collision(half_size=[0.5, 0.5, 0.5])
    actor_builder.add_box_visual(half_size=[0.5, 0.5, 0.5], material=[1.0, 0.0, 0.0])
    box = actor_builder.build(name="box")
    box.set_pose(sapien.Pose(p=[0, 0, 0.5]))

    # Add some lights so that you can observe the scene
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    actor = scene.create_actor_builder().build_kinematic()
    actor.set_pose(sapien.Pose([-3, 0, 0.5]))
    cam = scene.add_mounted_camera("", actor, sapien.Pose(), 128, 128, 1, 0.01, 10)

    scene.update_render()
    cam.take_picture()
    color = cam.get_picture("Color")

    Image.fromarray((color[..., :3].clip(0, 1) * 255).astype(np.uint8)).save(
        "sapien_offscreen.png"
    )


if __name__ == "__main__":
    main()
