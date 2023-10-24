import importlib
import warnings


def load_package(name: str):
    if name.startswith("sapien_"):
        name = name[7:]

    # try to import the package if exists
    import importlib

    try:
        return importlib.import_module("sapien_" + name)
    except ModuleNotFoundError:
        pass

    warnings.warn(
        f"Installing sapien package at runtime can be unsafe and should be avoided. Please use `sapien install {name}` instead."
    )

    import subprocess
    import sys

    args = [
        sys.executable,
        "-m",
        "pip",
        "install",
        "--user",
        f"git+https://github.com/sapien-sim/{name}",
    ]
    subprocess.run(args)

    return importlib.import_module("sapien_" + name)
