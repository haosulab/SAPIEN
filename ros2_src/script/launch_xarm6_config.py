import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchService


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    assert os.path.exists(absolute_file_path), "No file exist for path: \n {}".format(absolute_file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    assert os.path.exists(absolute_file_path), "No file exist for path: \n {}".format(absolute_file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file, yaml.FullLoader)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = load_file('sapien_resources', 'xarm6_description/urdf/xarm6.urdf')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('sapien_resources', 'xarm6_moveit_config/config/xarm6.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('sapien_resources', 'xarm6_moveit_config/config/kinematics.yaml')
    # robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    controllers_yaml = load_yaml('sapien_resources', 'xarm6_moveit_config/config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml}

    ompl_planning_pipeline_config = {'ompl': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml('sapien_resources', 'xarm6_moveit_config/config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # MoveItCpp demo executable
    parameter_node = Node(node_name='xarm6_config',
                          # node_namespace="scene1",
                          package='demo_nodes_cpp',
                          node_executable='parameter_blackboard',
                          output='screen',
                          parameters=[
                              robot_description,
                              robot_description_semantic,
                              kinematics_yaml,
                              ompl_planning_pipeline_config,
                              moveit_controllers
                          ])

    return LaunchDescription([parameter_node])


def main():
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()


if __name__ == '__main__':
    main()
