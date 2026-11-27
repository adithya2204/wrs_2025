import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("AizuSpiderDA", package_name="aizu_moveit")
        .robot_description(file_path="config/AizuSpiderDA.urdf.xacro")
        .to_moveit_configs()
    )

    # --- MoveGroup Node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"octomap_resolution": 0.05},
            {"use_sim_time": True}
        ],
        # MoveIt now listens to the fully merged joint states!
        remappings=[("joint_states", "/joint_states")]
    )

    # --- RViz Node ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d", os.path.join(get_package_share_directory("aizu_moveit"), "config", "moveit.rviz"),
            "--ros-args", "--log-level", "error"
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ]
    )
    
    # --- Joint State Publisher (The Merger) ---
    # This reads the partial arm states from Choreonoid and auto-fills missing flippers!
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_merger',
        parameters=[
            moveit_config.robot_description,
            {'source_list': ['/AizuSpider/moveit_joint_states']}
        ]
    )

    # --- Joystick and Control Nodes ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1}]
    )

    multi_joint_teleop_node = Node(
        package='aizu_moveit',
        executable='multi_joint_teleop.py',
        name='multi_joint_teleop',
        output='screen'
    )

    controller_switcher_node = Node(
        package='aizu_moveit',
        executable='controller_switcher.py',
        name='controller_switcher',
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        move_group_node,
        jsp_node,
        joy_node,
        multi_joint_teleop_node,
        controller_switcher_node
    ])