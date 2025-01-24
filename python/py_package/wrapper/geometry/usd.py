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
import shutil
import os
import subprocess
import sys

from .cache import cached


def find_blender():
    blender = shutil.which("blender") or shutil.which("org.blender.Blender")
    if blender is not None:
        return ["blender", "--background", "--python"]

    try:
        import bpy

        return [sys.executable]
    except ModuleNotFoundError:
        pass

    raise FileNotFoundError(
        "Failed to find Blender. Please install Blender and add it to PATH. Also make sure Blender can access to your filesystem."
    )


@cached(".sapien.glb.checksum")
def convert_usd_to_glb(usd_file: str, glb_file: str):
    usd_file = os.path.abspath(usd_file)

    if not os.path.exists(usd_file):
        raise FileNotFoundError(usd_file)

    command = find_blender()

    subprocess.run(
        command
        + [
            os.path.abspath(
                os.path.join(os.path.dirname(__file__), "blender_usd_to_glb.py")
            ),
            "--",
            usd_file,
            glb_file,
        ],
        check=True,
    )
