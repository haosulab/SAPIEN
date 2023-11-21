from warnings import warn
import pkg_resources
import os
from pathlib import Path
import platform
from .version import __version__

os.environ["SAPIEN_PACKAGE_PATH"] = os.path.dirname(__file__)
from . import _oidn_tricks

from .pysapien import *
from . import serialization

from . import _vulkan_tricks

from .wrapper.scene import Scene, SceneConfig, Widget
from .wrapper.engine import Engine
from .wrapper.renderer import SapienRenderer
from .wrapper.actor_builder import ActorBuilder
from .wrapper.articulation_builder import ArticulationBuilder
from .wrapper.pinocchio_model import PinocchioModel

import pkg_resources

try:
    render.set_imgui_ini_filename(str(Path.home() / ".sapien" / "imgui.ini"))
    render._internal_set_shader_search_path(
        pkg_resources.resource_filename("sapien", "vulkan_shader")
    )
    render.set_viewer_shader_dir("default")
    render.set_camera_shader_dir("default")
except RuntimeError:
    pass

from . import utils
from . import asset
from . import render_server
