import os
import pkg_resources


def get_project_root() -> str:
    return pkg_resources.resource_filename("sapien", "")


def get_assets_path() -> str:
    return pkg_resources.resource_filename("sapien", "assets")


def init_path():
    project_root = get_project_root()
    import shutil
    glsl_path = os.path.join(project_root, "glsl_shader")
    assert os.path.exists(glsl_path), "root glsl_shader not exist"
    if not os.path.exists("glsl_shader"):
        shutil.copytree(glsl_path, "glsl_shader")
