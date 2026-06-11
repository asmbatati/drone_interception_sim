#!/usr/bin/env python3
"""Full pluggable interception pipeline with a selectable orchestration head.

Perception/estimation/planning always run; the orchestrator (and whether the
MPC->control cascade or a direct-drive controller flies the craft) is chosen
with orchestrator:= :

  detection -> estimation -> prediction -> planning ->        (always)
     |                                          |
     |  orchestrator:=fsm|offboard|none  -> MPC -> control -> PX4   (cascade)
     |  orchestrator:=bt                 -> d2dtracker_states BT -> PX4 (direct)
     |  orchestrator:=rl                 -> (future: d2dtracker_rl policy)

Heads:
  fsm       pipeline_fsm sequences arm/AUTO.TAKEOFF/OFFBOARD/AUTO.RTL on top of
            the cascade (the control stage streams the OFFBOARD setpoints). [default]
  offboard  minimal arm+OFFBOARD only (no phases) on top of the cascade.
  bt        the d2dtracker_states Behaviour Tree drives directly from the
            planner's intercept_point (MPC+control are NOT started).
  none      cascade runs but nothing arms/flies (topic inspection / RViz).
  rl        reserved for a d2dtracker_rl policy head (not yet wired).

Per-stage backends are still swappable: detector/estimator/predictor/planner/
mpc/controller. Sim launched separately (scripts/run_sim.sh, FastRTPS); same
RMW + ROS_DOMAIN_ID in this terminal.

  ros2 launch drone_interception_sim pipeline.launch.py                       # fsm + cascade
  ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=bt
  ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=offboard metrics:=true
  ros2 launch drone_interception_sim pipeline.launch.py predictor:=poly controller:=se3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    orch = LaunchConfiguration('orchestrator')

    # condition expressions over the orchestrator selection
    cascade_if = IfCondition(PythonExpression(
        ["'", orch, "' in ['fsm', 'offboard', 'none']"]))
    fsm_if = IfCondition(PythonExpression(["'", orch, "' == 'fsm'"]))
    offboard_if = IfCondition(PythonExpression(["'", orch, "' == 'offboard'"]))
    bt_if = IfCondition(PythonExpression(["'", orch, "' == 'bt'"]))

    def stage(pkg, exe, name, backend_param, backend_arg, params=None, remaps=None,
              condition=None):
        base = {'use_sim_time': use_sim_time}
        if backend_param:
            base[backend_param] = LaunchConfiguration(backend_arg)
        if params:
            base.update(params)
        return Node(package=pkg, executable=exe, name=name, namespace=ns, output='screen',
                    parameters=[base], remappings=remaps or [], condition=condition)

    # --- perception / estimation / planning (always) -----------------------
    detection = stage(
        'interception_detection', 'detection_node', 'detection_node',
        'backend', 'detector',
        params={'target_odom_topic': '/target/mavros/local_position/odom'})
    estimation = stage(
        'interception_estimation', 'tracker_node', 'tracker_ros',
        'model_type', 'estimator', params={'tracking_frame': 'map'},
        remaps=[('measurement/pose_array', '/interceptor/detection_node/detections_poses')])
    prediction = stage(
        'interception_prediction', 'prediction_node', 'prediction_node',
        'backend', 'predictor', params={'horizon': 20, 'dt': 0.1},
        remaps=[('target_estimate', '/interceptor/kf/good_tracks')])
    planning = stage(
        'interception_planning', 'planning_node', 'planning_node',
        'backend', 'planner',
        remaps=[('predicted_trajectory', '/interceptor/predicted_trajectory'),
                ('interceptor_odom', '/interceptor/mavros/local_position/odom')])

    # --- MPC -> control cascade (fsm | offboard | none) ---------------------
    mpc = stage(
        'interception_mpc', 'mpc_node', 'mpc_node', 'backend', 'mpc',
        remaps=[('reference', '/interceptor/reference'),
                ('interceptor_odom', '/interceptor/mavros/local_position/odom')],
        condition=cascade_if)
    control = stage(
        'interception_control', 'control_node', 'control_node', 'backend', 'controller',
        remaps=[('command_trajectory', '/interceptor/command_trajectory'),
                ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
                ('setpoint_raw_local', '/interceptor/mavros/setpoint_raw/local'),
                ('attitude_target', '/interceptor/mavros/setpoint_raw/attitude'),
                ('mavros_state', '/interceptor/mavros/state')],
        condition=cascade_if)

    # --- orchestration heads ------------------------------------------------
    # Orchestration heads run on WALL clock (use_sim_time=False): they are
    # timer-driven mission sequencers and must keep ticking even when /clock
    # delivery stalls (which it intermittently does under FastRTPS). Their logic
    # uses positions/ranges, not message timestamps, so the clock choice is safe.
    fsm = Node(
        package='drone_interception_sim', executable='pipeline_fsm',
        name='pipeline_fsm', namespace=ns, output='screen',
        parameters=[{'use_sim_time': False,
                     'takeoff_alt': LaunchConfiguration('takeoff_alt'),
                     'attack_distance': LaunchConfiguration('attack_distance'),
                     'capture_radius': LaunchConfiguration('capture_radius')}],
        remappings=[('interceptor_odom', '/interceptor/mavros/local_position/odom'),
                    ('target_estimate', '/interceptor/kf/good_tracks_pose_array')],
        condition=fsm_if)

    offboard = Node(
        package='drone_interception_sim', executable='offboard_manager',
        name='offboard_manager', namespace=ns, output='screen',
        parameters=[{'use_sim_time': False}], condition=offboard_if)

    bt = Node(
        package='d2dtracker_states', executable='bt_node',
        name='bt_node', namespace=ns, output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('intercept_point', '/interceptor/intercept_point'),
                    ('target_detection', '/interceptor/detection_node/detections_poses'),
                    ('target_odom', '/target/mavros/local_position/odom')],
        condition=bt_if)

    # Method string is self-labeling (the selected backends), so the summary
    # table rows are directly comparable across benchmark runs.
    metrics = Node(
        package='drone_interception_sim', executable='interception_metrics',
        name='interception_metrics', namespace=ns, output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'method': ['pipeline/', LaunchConfiguration('planner'),
                                '+', LaunchConfiguration('predictor'),
                                '+', LaunchConfiguration('mpc'),
                                '+', LaunchConfiguration('controller')],
                     'csv_path': LaunchConfiguration('csv_path'),
                     'summary_csv_path': LaunchConfiguration('summary_csv_path')}],
        remappings=[('interceptor_odom', '/interceptor/mavros/local_position/odom'),
                    ('target_odom', '/target/mavros/local_position/odom')],
        condition=IfCondition(LaunchConfiguration('metrics')))

    # Visualization: republish actual/estimated/interceptor poses, predicted +
    # planned paths, the intercept point and a live estimation-error readout as
    # one colour-coded MarkerArray in 'map'. The display lives in the SIM's RViz
    # (rviz/interception.rviz, launched by interception.launch.py) — no separate
    # RViz here. Wall clock so it keeps refreshing if /clock stalls.
    viz = Node(
        package='drone_interception_sim', executable='pipeline_viz',
        name='pipeline_viz', namespace=ns, output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[('target_odom', '/target/mavros/local_position/odom'),
                    ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
                    ('target_estimate', '/interceptor/kf/good_tracks_pose_array'),
                    ('predicted_path', '/interceptor/predicted_path'),
                    ('planned_path', '/interceptor/reference_path'),
                    ('intercept_point', '/interceptor/intercept_point'),
                    ('markers', '/interceptor/interception/markers')],
        condition=IfCondition(LaunchConfiguration('viz')))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='interceptor'),
        # Sim time for the cascade. The detector propagates the odom's
        # (Gazebo-clock) source stamp and the KF derives target velocity from the
        # deltas between those stamps + extrapolates tracks to them, so the
        # perception chain must run on the *same* clock or the time base is
        # inconsistent and the estimate drifts on a fast target. The orchestration
        # heads + viz run on wall clock (set explicitly above) so their mission
        # timers keep ticking even if /clock momentarily stalls. Launch only via
        # this file (single entry) so there is exactly one /clock bridge — the
        # duplicate bridges from hand-launching were what made /clock flaky.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('orchestrator', default_value='fsm',
                              description='fsm | offboard | bt | none | (rl: future)'),
        DeclareLaunchArgument('detector', default_value='ground_truth'),
        DeclareLaunchArgument('estimator', default_value='const_vel'),
        # gru = the OLD implementation's trained predictor (pos+vel GRUs,
        # velocity-path, weights shipped in interception_prediction/models);
        # falls back to const_vel automatically if torch/weights are missing.
        DeclareLaunchArgument('predictor', default_value='gru'),
        DeclareLaunchArgument('planner', default_value='rendezvous'),
        DeclareLaunchArgument('mpc', default_value='passthrough'),
        DeclareLaunchArgument('controller', default_value='px4_setpoint'),
        DeclareLaunchArgument('takeoff_alt', default_value='4.0'),
        DeclareLaunchArgument('attack_distance', default_value='3.0'),
        DeclareLaunchArgument('capture_radius', default_value='1.0'),
        DeclareLaunchArgument('metrics', default_value='false'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/pipeline_metrics.csv'),
        DeclareLaunchArgument('summary_csv_path',
                              default_value='/tmp/interception_results.csv',
                              description='per-run summary table (appended)'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='publish the interception MarkerArray '
                                          '(shown in the sim RViz)'),
        GroupAction([detection, estimation, prediction, planning,
                     mpc, control, fsm, offboard, bt, metrics, viz]),
    ])
