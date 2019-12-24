import os


def get_project_root() -> str:
    project_path = os.path.join(__file__, "../../../")
    project_path = os.path.abspath(project_path)
    return project_path


def get_robot_root() -> str:
    project_path = os.path.join(__file__, "../../")
    project_path = os.path.abspath(project_path)
    return project_path


def get_assets_path() -> str:
    root = get_project_root()
    return os.path.abspath(os.path.join(root, "assets"))


def init_path():
    import sys
    project_root = get_project_root()
    # sys.path.append(os.path.join(project_root, "build"))
    # sys.path.append(os.path.join(project_root, "cmake-build-debug"))

    import shutil
    glsl_path = os.path.join(project_root, "glsl_shader")
    assert os.path.exists(glsl_path), "root glsl_shader not exist"
    shutil.copytree(glsl_path, "glsl_shader")
