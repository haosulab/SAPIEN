from ..pysapien.physx import *
from ..pysapien.physx import _enable_gpu
from pathlib import Path
import platform
from zipfile import ZipFile
import requests
import io


def enable_gpu():
    if is_gpu_enabled():
        return

    if platform.system() != "Linux":
        raise Exception("GPU PhysX is currently only supported on Linux")

    parent = Path.home() / ".sapien" / "physx" / "5.3.0"
    parent.mkdir(exist_ok=True, parents=True)

    dll = parent / "libPhysXGpu_64.so"
    url = "https://github.com/sapien-sim/physx-gpu/raw/main/5.3.0/libPhysXGpu_64.so.zip"

    if not dll.exists():
        print(
            "Downloading PhysX GPU library to ~/.sapien/physx/5.3.0 from Github. This can take several minutes. If it fails to download, please manually download https://github.com/sapien-sim/physx-gpu/raw/main/5.3.0/libPhysXGpu_64.so.zip and unzip at ~/.sapien/physx/5.3.0/"
        )
        res = requests.get(url)
        z = ZipFile(io.BytesIO(res.content))
        z.extractall(parent)
        print("Download complete.")

    import ctypes

    ctypes.CDLL("libcuda.so", ctypes.RTLD_GLOBAL)
    ctypes.CDLL(str(dll), ctypes.RTLD_LOCAL)

    _enable_gpu()
