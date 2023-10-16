import importlib
import io
import os
import sys
import tempfile
import zipfile
from pathlib import Path
import shutil

import requests

loaded_packages = dict()

# TODO: implement global package
# TODO: implement versioning
# global_package_dir = Path.home() / ".sapien" / "sapien_packages_global"
local_package_dir = Path(".") / "sapien_packages"

sys.path.insert(0, local_package_dir.parent)
# sys.path.insert(0, global_package_dir.parent)


def check_loaded(name: str, version: str):
    if name in loaded_packages:
        lv = loaded_packages[name].__version__
        if version is None or lv == version:
            return loaded_packages[name]
        raise Exception(
            f"pacakge {name} version mismatch. {lv} loaded but {version} requested"
        )
    return None


def check_local(name: str, version: str):
    try:
        m = importlib.import_module(f"sapien_packages.{name}")
    except ModuleNotFoundError:
        return None

    if version is not None and m.__version__ != version:
        return None
    return m


# def check_global(name: str, version: str):
#     try:
#         m = importlib.import_module(f"sapien_packages_global.{name}")
#     except ModuleNotFoundError:
#         return None

#     if m.__version__ != version:
#         return None
#     return m


def download_to_local(path: str, name: str):
    print(f"Downloading package from {path}")
    if path.startswith("https://") or path.startswith("http://"):
        r = requests.get(path, stream=True)
        if not r.ok:
            raise Exception(f"failed to load package from url {path}")

        z = zipfile.ZipFile(io.BytesIO(r.content))
    else:
        z = zipfile.ZipFile(path)

    local_package_dir.mkdir(exist_ok=True, parents=True)

    with tempfile.TemporaryDirectory() as temp:
        z.extractall(temp)
        files = os.listdir(temp)
        if len(files) != 1:
            raise Exception(f"invalid package {path}")
        if not os.path.isdir(os.path.join(temp, files[0])) or files[0] != name:
            raise Exception(f"invalid package {path}")

        shutil.rmtree(str(local_package_dir / name), ignore_errors=True)
        shutil.copytree(os.path.join(temp, files[0]), str(local_package_dir / name))

    return check_local(name, None)


def load_package(path: str, version: str = None, reinstall=False):
    is_url = path.startswith("https://") or path.startswith("http://")
    if is_url:
        name = path.split("/")[-1]
    else:
        name = os.path.split(path)[-1]
        assert name.lower().endswith(".zip")

    if name.lower().endswith(".zip"):
        name = name[:-4]

    if not reinstall:
        loaded = check_loaded(name, version)
        if loaded is not None:
            return loaded

        loaded = check_local(name, version)
        if loaded is not None:
            loaded_packages[name] = loaded
            return loaded

    # loaded = check_global(name, version)
    # if loaded is not None:
    #     loaded_packages[name] = loaded
    #     return loaded

    loaded = download_to_local(path, name)
    if loaded is not None:
        loaded_packages[name] = loaded
        return loaded

    raise Exception(f"failed to load package {name} from {path}")
