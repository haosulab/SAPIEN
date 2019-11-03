import os


def get_project_root() -> str:
    project_path = os.path.join(__file__, "../../../../")
    project_path = os.path.abspath(project_path)
    return project_path


def get_robot_root() -> str:
    project_path = os.path.join(__file__, "../../../")
    project_path = os.path.abspath(project_path)
    return project_path


def get_robot_python_root() -> str:
    project_path = os.path.join(__file__, "../../")
    project_path = os.path.abspath(project_path)
    return project_path


def get_assets_path() -> str:
    root = get_project_root()
    return os.path.abspath(os.path.join(root, "assets"))
