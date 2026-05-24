"""
sim_truck.launch.py
────────────────────────────────────────────────────────────────────────────
Launches the full port-tractor simulation:

  Gazebo Harmonic
    └─ heavy_tractor model  (tractor.urdf)
         ├─ JointPositionController  → fl_steer, fr_steer
         ├─ JointController (vel)    → ml/mr/rl/rr drive wheels
         ├─ JointStatePublisher      → /joint_states  (via bridge)
         ├─ OdometryPublisher        → /model/heavy_tractor/odometry (via bridge)
         ├─ IMU sensor               → /imu            (via bridge)
         └─ GPS sensor               → /gps/fix         (via bridge)

  ROS2 Nodes
    ├─ robot_state_publisher         publishes /robot_description + TF
    ├─ ackermann_controller_node     1000 Hz: /truck/cmd_delta → fl/fr steer cmds
    ├─ wheel_state_node              /joint_states → /truck/vehicle_delta
    └─ vehicle_state_node            all sources → /truck/vehicle_state (27-state)

Topic flow:
  user/planner ──► /truck/cmd_delta ──► ackermann_controller_node
                                             ├─► /truck/fl_steer/cmd_pos ──► Gz
                                             └─► /truck/fr_steer/cmd_pos ──► Gz

  Gz joint_states ──► (bridge) ──► /joint_states
                                        ├─► wheel_state_node ──► /truck/vehicle_delta
                                        │                     └─► /truck/{fl,fr}_steer_angle
                                        └─► vehicle_state_node ──► /truck/vehicle_state
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('smartport_description')
    urdf_file  = os.path.join(pkg_description, 'model', 'tractor.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'port.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── Gazebo Harmonic simulator ────────────────────────────────────────────
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

    # ── Spawn tractor entity into Gazebo ─────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'heavy_tractor',
            '-x', '15',
            '-y', '10',
            '-z', '1.0',
        ],
        output='screen',
    )

    # ── ROS ↔ Gazebo bridge ──────────────────────────────────────────────────
    # Direction markers:
    #   [  = Gz → ROS   (subscribe on Gz, publish on ROS)
    #   ]  = ROS → Gz   (subscribe on ROS, publish on Gz)
    #   @  = bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ── Sensors (Gz → ROS) ─────────────────────────────────────────
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',

            # ── Odometry (Gz → ROS): pose + body-frame twist ───────────────
            '/model/heavy_tractor/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # ── Joint states (Gz → ROS): all joint positions + velocities ──
            '/world/port/model/heavy_tractor/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',

            # ── Steering joint position commands (ROS → Gz) ─────────────────
            '/truck/fl_steer/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/truck/fr_steer/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',

            # ── Rear-wheel drive command (ROS → Gz, rad/s) ──────────────────
            '/truck/drive@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            # Remap Gazebo joint_state topic to the standard ROS2 name
            ('/world/port/model/heavy_tractor/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    # ── Ackermann controller @ 1000 Hz ──────────────────────────────────────
    #    Reads /truck/cmd_delta → computes per-wheel Ackermann angles →
    #    publishes /truck/fl_steer/cmd_pos and /truck/fr_steer/cmd_pos
    ackermann_controller = Node(
        package='smartport_truckcontroller',
        executable='ackermann_controller_node',
        name='ackermann_controller',
        output='screen',
        parameters=[{
            'wheelbase':     3.0,    # L [m]  must match tractor.urdf
            'track_width':   2.4,    # W [m]  must match tractor.urdf
            'max_steer_rad': 0.5236, # 30°
            'loop_rate_hz':  1000.0,
        }],
    )

    # ── Wheel state node ─────────────────────────────────────────────────────
    #    Reads /joint_states → publishes /truck/vehicle_delta,
    #    /truck/fl_steer_angle, /truck/fr_steer_angle
    wheel_state = Node(
        package='smartport_truckcontroller',
        executable='wheel_state_node',
        name='wheel_state',
        output='screen',
    )

    # ── Vehicle state node ───────────────────────────────────────────────────
    #    Aggregates: odometry + IMU + joint_states + cmd_delta + drive →
    #    publishes /truck/vehicle_state (VehicleState, 27 states)
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
