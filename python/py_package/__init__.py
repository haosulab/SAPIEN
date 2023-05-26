import os
import warnings
import platform

if platform.system() == "Linux":
    if not os.path.exists("/proc/version"):
        warnings.warn("You are not using a standard Linux based OS. SAPIEN probably will not work")
    with open("/proc/version") as f:
        if 'microsoft' in '\n'.join(f.readlines()).lower():
            warnings.warn("It seems you are using WSL. SAPIEN renderer is not supported on WSL. Please use SAPIEN on native Windows.")

from sapien import core, sensor, asset, example, utils
from sapien.version import __version__
