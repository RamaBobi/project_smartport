"""
sim_truck_physics.launch.py
────────────────────────────────────────────────────────────────────────────
Physics-tuning session: empty world + copy_truck.urdf (tractor only).

Differences vs sim_truck.launch.py:
  World     : worlds/empty.sdf          (no port, no containers → faster sim)
  Model     : model/copy_truck.urdf     (physics-optimised tractor, no trailer)
  Spawn pos : origin (0, 0, 1.0)        (close to origin for easy measurement)
  Bridge    : world name "empty" → joint-state topic changes to
              /world/empty/model/copy_truck/joint_state

All 3 ROS2 control nodes are identical:
  ackermann_controller_node  (1000 Hz)
  wheel_state_node
  vehicle_state_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('smartport_description')

    urdf_file  = os.path.join(pkg_description, 'model',  'copy_truck.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'empty.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── Gazebo Harmonic ──────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── Robot description (URDF → /robot_description + TF) ──────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # ── Spawn copy_truck at origin ───────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'copy_truck',
            '-x', '0',
            '-y', '0',
            '-z', '1.0',
        ],
        output='screen',
    )

    # ── ROS ↔ Gazebo bridge ──────────────────────────────────────────────────
    # IMPORTANT: world name is "empty" (not "port"), so the joint-state
    # bridge topic is /world/empty/model/copy_truck/joint_state.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ── Sensors (Gz → ROS) ──────────────────────────────────────────
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',

            # ── Odometry (Gz → ROS) ─────────────────────────────────────────
            '/model/copy_truck/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # ── Joint states (Gz → ROS) ──────────────────────────────────────
            '/world/empty/model/copy_truck/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',

            # ── Steering commands (ROS → Gz) ─────────────────────────────────
            '/truck/fl_steer/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/truck/fr_steer/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',

            # ── Drive command (ROS → Gz) ─────────────────────────────────────
            '/truck/drive@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            # Remap joint-state topic to standard /joint_states
            ('/world/empty/model/copy_truck/joint_state', '/joint_states'),
            # Remap model odometry to the same topic as the port world
            ('/model/copy_truck/odometry', '/model/heavy_tractor/odometry'),
        ],
        output='screen',
    )

    # ── Ackermann controller @ 1000 Hz ──────────────────────────────────────
    ackermann_controller = Node(
        package='smartport_truckcontroller',
        executable='ackermann_controller_node',
        name='ackermann_controller',
        output='screen',
        parameters=[{
            'wheelbase':     3.0,
            'track_width':   2.4,
            'max_steer_rad': 0.5236,
            'loop_rate_hz':  1000.0,
        }],
    )

    # ── Wheel state node ─────────────────────────────────────────────────────
    wheel_state = Node(
        package='smartport_truckcontroller',
        executable='wheel_state_node',
        name='wheel_state',
        output='screen',
    )

    # ── Vehicle state node ───────────────────────────────────────────────────
    vehicle_state = Node(
        package='smartport_truckcontroller',
        executable='vehicle_state_node',
        name='vehicle_state',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        ackermann_controller,
        wheel_state,
        vehicle_state,
    ])
