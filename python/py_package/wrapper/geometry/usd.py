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
