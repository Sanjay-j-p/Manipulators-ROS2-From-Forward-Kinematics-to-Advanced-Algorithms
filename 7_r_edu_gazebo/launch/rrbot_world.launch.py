import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Paths
    rrbot_description_path = get_package_share_directory('7_r_edu_description')
    xacro_file = os.path.join(rrbot_description_path, 'urdf', 'seven_r.urdf.xacro')

    # Process Xacro file
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn Entity in Ignition Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(), '-name', 'rrbot', '-allow_renaming', 'true']
    )

    # Load controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_position_controller'],
        output='screen'
    )
    # load_gripper_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
    #     output='screen'
    # )

    # Bridge for ROS-Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen'
    )

    return LaunchDescription([
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')]
            ),
            launch_arguments=[('gz_args', ' -r -v 4 empty.sdf')]
        ),
        # Register controller loading after entity spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_forward_velocity_controller]
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_forward_velocity_controller,
        #         on_exit=[load_gripper_controller]
        #     )
        # ),
        # Nodes
        node_robot_state_publisher,
        ignition_spawn_entity,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        bridge,
    ])
