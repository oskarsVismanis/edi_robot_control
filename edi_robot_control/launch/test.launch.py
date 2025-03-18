import os
import yaml
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
# import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    # robot_description_config = load_file(
    #     "edi_robot_description", "urdf/edi_ur5e.urdf.xacro"
    # )
    # robot_description = {"robot_description": robot_description_config}

    # robot_description_semantic_config = load_file(
    #     "moveit_resources_panda_moveit_config", "config/ur5e.srdf.xacro"
    # )
    # robot_description_semantic = {
    #     "robot_description_semantic": robot_description_semantic_config
    # }

    # kinematics_yaml = load_yaml(
    #     "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    # )

    # planning_yaml = load_yaml(
    #     "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    # )

    # planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    # multi_server = Node(
    #     package="nl_srv",
    #     executable="multi_server",
    #     output="screen",
    # )

    edi_parameters = {
        "use_sim_time": True
    }

    # Start the actual move_group node/action server
    edi_pick_place_node = Node(
        package="edi_robot_control",
        executable="edi_pick_and_place",
        output="screen",
        # output={
        #     "stdout": "screen",
        #     "stderr": "screen",
        # },
        parameters=[
            edi_parameters,
            # robot_description,
            # robot_description_semantic,
            # kinematics_yaml,
            # planning_yaml,
            # planning_plugin,
        ],
    )



    return LaunchDescription([edi_pick_place_node,])# multi_server])