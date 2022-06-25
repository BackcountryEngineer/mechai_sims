import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory("mechai_sims")
    default_model_path = os.path.join(pkg_share, "urdf/diff_drive.xacro.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    default_world_path = os.path.join(pkg_share, "world/sample_nav2.world")
    default_controller_config_path = os.path.join(pkg_share, "config/diff_drive_controllers.yaml")

    # localization = launch_ros.actions.Node(
    #    package="robot_localization",
    #    executable="ekf_node",
    #    name="ekf_filter_node",
    #    output="screen",
    #    parameters=[os.path.join(pkg_share, "config/ekf.yaml"), {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    # )

    # slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         pkg_share, "launch"), "/online_async_launch.py"]),
    # )

    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         pkg_share, "launch"), "/navigation_launch.py"]),
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]),
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}]
    )

    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "diff_drive", 
            "-topic", "robot_description"
        ],
        output="screen"
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot urdf file"
        ),
        launch.actions.DeclareLaunchArgument(
            "world", 
            default_value=default_world_path,
            description="Specify world file name"
        ),
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig", 
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file"
        ),
        launch.actions.DeclareLaunchArgument(
            name="controller_config", 
            default_value=default_controller_config_path,
            description="Configuration for ros2 controllers"
        ),
        launch.actions.DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time"
        ),
        launch.actions.DeclareLaunchArgument(
            name="gui", 
            default_value="False",
            description="Flag to enable gui"
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # rviz_node,
    ])