from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controllers = [
        "joint_state_broadcaster",
        "arm1_manipulator_controller",
        "arm2_manipulator_controller",
        "arm1_fingers_controller",
        "arm2_fingers_controller"
    ]

    spawner_nodes = []
    for ctrl in controllers:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[ctrl, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    return LaunchDescription(spawner_nodes)