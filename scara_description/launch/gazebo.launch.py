import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("scara_description")
    yaml_path = os.path.join(share, "config", "controllers.yaml")
    with open(os.path.join(share, "urdf", "scara.urdf")) as f:
        robot_description = f.read().replace("@CONTROLLERS_YAML@", yaml_path)
    # YAML-unsafe (": ") text lives in XML comments; strip them
    robot_description = re.sub(r"<!--.*?-->", "", robot_description, flags=re.S)

    old = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_path = (old + ":" if old else "") + os.path.dirname(share)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"),
                         "launch", "gazebo.launch.py")))

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
        output="screen")

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "scara"],
        output="screen")

    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager-timeout", "60"],
        output="screen")

    jtc = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_trajectory_controller",
                   "--controller-manager-timeout", "60"],
        output="screen")

    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        UnsetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE"),
        gazebo, rsp, spawn, jsb, jtc,
    ])
