from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")

    declared_arguments = [
        DeclareLaunchArgument("description_file", default_value="handrobot.urdf.xacro"),
        DeclareLaunchArgument("gui", default_value="true")
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("handrobot_ros2_control"), "urdf", description_file])
    ])
    robot_description = {"robot_description": robot_description_content}

    controller_config = PathJoinSubstitution([
        FindPackageShare("handrobot_ros2_control"),
        "config",
        "handrobot_controllers.yaml"
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("handrobot_ros2_control"),
        "rviz",
        "handrobot_view.rviz"
    ])

    # Nodes
    # Note: joint states are published by hand_controller_node directly

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
        output="screen"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen"
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_fpc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_joint_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    hand_controller_node = Node(
        package="handrobot_ros2_control",
        executable="hand_controller_node.py",
        name="hand_controller_node",
        output="screen"
    )

    gesture_command_publisher_node = Node(
        package="handrobot_ros2_control",
        executable="gesture_command_publisher.py",
        name="gesture_command_publisher",
        output="screen"
    )

    # Add delays to ensure proper startup sequence
    delayed_load_jsb = TimerAction(
        period=2.0,
        actions=[load_jsb]
    )

    delayed_load_fpc = TimerAction(
        period=4.0,
        actions=[load_fpc]
    )

    delayed_hand_controller = TimerAction(
        period=6.0,
        actions=[hand_controller_node]
    )

    # Comment out automatic gesture publisher for manual control
    # delayed_gesture_publisher = TimerAction(
    #     period=8.0,
    #     actions=[gesture_command_publisher_node]
    # )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        rviz_node,
        ros2_control_node,
        delayed_load_jsb,
        delayed_load_fpc,
        delayed_hand_controller,
        # delayed_gesture_publisher  # Commented out for manual control
    ])
