from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory("aizu_spider_description")

    urdf_file = os.path.join(pkg, "urdf", "Hobbydrone.urdf")
    rviz_file = os.path.join(pkg, "rviz", "hobby_disp.rviz")

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_description = ParameterValue(robot_desc, value_type=str)

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="HobbyDrone",
            output="screen",
            parameters=[{
                "robot_description": robot_description
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
            parameters=[{
                "robot_description": robot_description
            }]
        ),
    ])