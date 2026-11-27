import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_pkg = get_package_share_directory('aizu_spider_description')
    moveit_pkg = get_package_share_directory('aizu_moveit')
    
    # Use MoveIt URDF to provide <ros2_control> tags to the parameter server
    urdf_file = os.path.join(moveit_pkg, 'config', 'AizuSpiderDA.urdf.xacro')
    initial_pos_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')
    
    params_file = os.path.join(desc_pkg, 'config', 'ros2_controllers.yaml')
    # Workspace root: setup_env.sh sets WRS_WS; without it, assume the clone is at ~/wrs_2025
    workspace = os.environ.get('WRS_WS', os.path.expanduser('~/wrs_2025'))
    project_file = os.path.join(workspace, 'src', 'choreonoid', 'ext', 'WRS2025',
                                'project', 'project_prasid', 'Aizu_modified_M3.cnoid')
    robot_desc = ParameterValue(
        Command(['xacro ', urdf_file, ' initial_positions_file:=', initial_pos_file]), 
        value_type=str
    )

    # 1. The Global Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Launch Choreonoid and remap the internal joint_state_broadcaster
    # to a safe namespace so it doesn't clash with BodyROS2Item!
    choreonoid_process = ExecuteProcess(
        cmd=['ros2', 'run', 'choreonoid_ros', 'choreonoid', project_file, 
             '--ros-args', 
             '--params-file', params_file,
             '-r', '/joint_states:=/AizuSpider/moveit_joint_states'],
        output='screen'
    )

    return LaunchDescription([rsp_node, choreonoid_process])