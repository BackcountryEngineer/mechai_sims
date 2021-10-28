import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]),
    )

    mechanum_car_xacro = os.path.join(get_package_share_directory("mechai_sims"), "urdf", "mechanum_car.xacro.urdf")

    doc = xacro.parse(open(mechanum_car_xacro))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    urdf_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        arguments=["-topics", "robot_description","-entity", "mechanum_car"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        urdf_spawner
    ])