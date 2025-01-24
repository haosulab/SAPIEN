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
from warnings import warn
import pkg_resources
import os
from pathlib import Path
import platform
from .version import __version__

os.environ["SAPIEN_PACKAGE_PATH"] = os.path.dirname(__file__)
from . import _oidn_tricks

from . import pysapien

from .pysapien import Entity, Component, System, CudaArray, Pose, Device
from .pysapien import profile
from .pysapien import set_log_level
from .pysapien import math

import platform
if platform.system() != "Darwin":
    from .pysapien import simsense

from . import physx
from . import render

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
    pysapien.render._internal_set_shader_search_path(
        pkg_resources.resource_filename("sapien", "vulkan_shader")
    )
    render.set_viewer_shader_dir("default")
    render.set_camera_shader_dir("default")
except RuntimeError:
    pass

from . import utils
from . import asset
