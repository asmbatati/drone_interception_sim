#!/usr/bin/env python3
# Copyright 2026 Abdulrahman S. Al-Batati
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Full pluggable interception pipeline with a selectable orchestration head.

Perception/estimation/planning always run; the orchestrator (and whether the
MPC->control cascade or a direct-drive controller flies the craft) is chosen
with orchestrator:= :

  detection -> estimation -> prediction -> planning ->        (always)
     |                                          |
     |  orchestrator:=fsm|offboard|none  -> MPC -> control -> PX4   (cascade)
     |  orchestrator:=bt                 -> shaheen_states BT -> PX4 (direct)
     |  orchestrator:=rl                 -> shaheen_rl policy_head (rl_model:=...)

Heads:
  fsm       pipeline_fsm sequences arm/AUTO.TAKEOFF/OFFBOARD/AUTO.RTL on top of
            the cascade (the control stage streams the OFFBOARD setpoints). [default]
  offboard  minimal arm+OFFBOARD only (no phases) on top of the cascade.
  bt        the shaheen_states Behaviour Tree drives directly from the
            planner's intercept_point (MPC+control are NOT started).
  none      cascade runs but nothing arms/flies (topic inspection / RViz).
  rl        shaheen_rl policy_head flies a trained SB3 policy directly
            (velocity setpoints; MPC+control are NOT started). rl_model:=<zip>.

Per-stage backends are still swappable: detector/estimator/predictor/planner/
mpc/controller. Sim launched separately (scripts/run_sim.sh, FastRTPS); same
RMW + ROS_DOMAIN_ID in this terminal.

  ros2 launch drone_interception_sim pipeline.launch.py                       # fsm + cascade
  ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=bt
  ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=offboard metrics:=true
  ros2 launch drone_interception_sim pipeline.launch.py predictor:=poly controller:=se3
  ros2 launch drone_interception_sim pipeline.launch.py orchestrator:=rl \
      rl_model:=$HOME/drone_interception_ws/ros2_ws/src/shaheen_rl/checkpoints/intercept/model_evasive_40k.zip
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
    cascade_if = IfCondition(PythonExpression(["'", orch, "' in ['fsm', 'offboard', 'none']"]))
    fsm_if = IfCondition(PythonExpression(["'", orch, "' == 'fsm'"]))
    offboard_if = IfCondition(PythonExpression(["'", orch, "' == 'offboard'"]))
    bt_if = IfCondition(PythonExpression(["'", orch, "' == 'bt'"]))
    rl_if = IfCondition(PythonExpression(["'", orch, "' == 'rl'"]))

    def stage(
        pkg, exe, name, backend_param, backend_arg, params=None, remaps=None, condition=None
    ):
        base = {'use_sim_time': use_sim_time}
        if backend_param:
            base[backend_param] = LaunchConfiguration(backend_arg)
        if params:
            base.update(params)
        return Node(
            package=pkg,
            executable=exe,
            name=name,
            namespace=ns,
            output='screen',
            parameters=[base],
            remappings=remaps or [],
            condition=condition,
        )

    # --- perception / estimation / planning (always) -----------------------
    detection = stage(
        'interception_detection',
        'detection_node',
        'detection_node',
        'backend',
        'detector',
        params={
            'target_odom_topic': '/target/mavros/local_position/odom',
            'cue': LaunchConfiguration('cue'),
            'cue_min_range': LaunchConfiguration('cue_min_range'),
            'cue_noise': LaunchConfiguration('cue_noise'),
        },
    )
    estimation = stage(
        'interception_estimation',
        'tracker_node',
        'tracker_ros',
        'model_type',
        'estimator',
        params={
            'tracking_frame': 'map',
            'sigma_a': LaunchConfiguration('kf_sigma_a'),
            'r_diag_x': LaunchConfiguration('kf_r'),
            'r_diag_y': LaunchConfiguration('kf_r'),
            'r_diag_z': LaunchConfiguration('kf_r'),
        },
        remaps=[('measurement/pose_array', '/interceptor/detection_node/detections_poses')],
    )
    prediction = stage(
        'interception_prediction',
        'prediction_node',
        'prediction_node',
        'backend',
        'predictor',
        params={'horizon': 20, 'dt': 0.1},
        remaps=[('target_estimate', '/interceptor/kf/good_tracks')],
    )
    planning = stage(
        'interception_planning',
        'planning_node',
        'planning_node',
        'backend',
        'planner',
        params={
            'yaw_mode': PythonExpression(
                ["'target' if '", LaunchConfiguration('gaze'), "' == 'true' else 'velocity'"]
            ),
            'min_altitude': LaunchConfiguration('min_altitude'),
        },
        remaps=[
            ('predicted_trajectory', '/interceptor/predicted_trajectory'),
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
        ],
    )

    # --- MPC -> control cascade (fsm | offboard | none) ---------------------
    mpc = stage(
        'interception_mpc',
        'mpc_node',
        'mpc_node',
        'backend',
        'mpc',
        remaps=[
            ('reference', '/interceptor/reference'),
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
        ],
        condition=cascade_if,
    )
    control = stage(
        'interception_control',
        'control_node',
        'control_node',
        'backend',
        'controller',
        # 'mass' is read by the se3 backend only. '' lookahead: se3 0.1 s;
        # 1.0 s behind mpc_12state, whose plan starts at the current pose, so
        # a position setpoint must lead the drone by about 1 / MPC_XY_P (0.95)
        # to hold the plan's speed (0.3 s hovers); others 0.3 s
        params={
            'mass': LaunchConfiguration('vehicle_mass'),
            'hover_thrust': LaunchConfiguration('hover_thrust'),
            'max_accel': LaunchConfiguration('max_accel'),
            'lookahead_time': PythonExpression(
                [
                    "float('",
                    LaunchConfiguration('control_lookahead'),
                    "') if '",
                    LaunchConfiguration('control_lookahead'),
                    "' else (0.1 if '",
                    LaunchConfiguration('controller'),
                    "' == 'se3' else (1.0 if '",
                    LaunchConfiguration('mpc'),
                    "' == 'mpc_12state' else 0.3))",
                ]
            ),
        },
        remaps=[
            ('command_trajectory', '/interceptor/command_trajectory'),
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
            ('setpoint_raw_local', '/interceptor/mavros/setpoint_raw/local'),
            ('attitude_target', '/interceptor/mavros/setpoint_raw/attitude'),
            ('mavros_state', '/interceptor/mavros/state'),
        ],
        condition=cascade_if,
    )

    # --- orchestration heads ------------------------------------------------
    # Orchestration heads run on WALL clock (use_sim_time=False): they are
    # timer-driven mission sequencers and must keep ticking even when /clock
    # delivery stalls (which it intermittently does under FastRTPS). Their logic
    # uses positions/ranges, not message timestamps, so the clock choice is safe.
    fsm = Node(
        package='drone_interception_sim',
        executable='pipeline_fsm',
        name='pipeline_fsm',
        namespace=ns,
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'takeoff_alt': LaunchConfiguration('takeoff_alt'),
                'attack_distance': LaunchConfiguration('attack_distance'),
                'capture_radius': LaunchConfiguration('capture_radius'),
                'capture_confirm': LaunchConfiguration('capture_confirm'),
                'follow_through': LaunchConfiguration('follow_through'),
            }
        ],
        remappings=[
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
            ('target_estimate', '/interceptor/kf/good_tracks_pose_array'),
        ],
        condition=fsm_if,
    )

    offboard = Node(
        package='drone_interception_sim',
        executable='offboard_manager',
        name='offboard_manager',
        namespace=ns,
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=offboard_if,
    )

    # Same takeoff/attack/capture settings as the FSM head, so the two heads
    # are compared on the same mission (the BT's standalone defaults are
    # 5.0 m / 1.5 m / 0.6 m).
    bt = Node(
        package='shaheen_states',
        executable='bt_node',
        name='bt_node',
        namespace=ns,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'takeoff_height': LaunchConfiguration('takeoff_alt'),
                'attack_distance': LaunchConfiguration('attack_distance'),
                'capture_radius': LaunchConfiguration('capture_radius'),
                'follow_through': LaunchConfiguration('follow_through'),
            }
        ],
        remappings=[
            ('intercept_point', '/interceptor/intercept_point'),
            ('target_detection', '/interceptor/detection_node/detections_poses'),
            ('target_odom', '/target/mavros/local_position/odom'),
        ],
        condition=bt_if,
    )

    # RL head: a shaheen_rl policy flies the interceptor directly (velocity
    # setpoints, like the BT it bypasses MPC/control). Same metrics as the rest.
    rl = Node(
        package='shaheen_rl',
        executable='policy_head',
        name='policy_head',
        namespace=ns,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'model': LaunchConfiguration('rl_model'),
                'velocity_frame': LaunchConfiguration('rl_velocity_frame'),
                'capture_radius': LaunchConfiguration('capture_radius'),
            }
        ],
        remappings=[
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
            ('target_odom', '/target/mavros/local_position/odom'),
        ],
        condition=rl_if,
    )

    # Method string is self-labeling (the selected backends, see the 'method'
    # argument default), so the summary rows are directly comparable across
    # benchmark runs. scenario/trial label the row for batch campaigns.
    metrics = Node(
        package='drone_interception_sim',
        executable='interception_metrics',
        name='interception_metrics',
        namespace=ns,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'method': LaunchConfiguration('method'),
                'capture_radius': LaunchConfiguration('capture_radius'),
                'scenario': LaunchConfiguration('scenario'),
                'trial': LaunchConfiguration('trial'),
                'csv_path': LaunchConfiguration('csv_path'),
                'summary_csv_path': LaunchConfiguration('summary_csv_path'),
                'referee': LaunchConfiguration('referee'),
            }
        ],
        remappings=[
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
            ('target_odom', '/target/mavros/local_position/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('metrics')),
    )

    # Visualization: republish actual/estimated/interceptor poses, predicted +
    # planned paths, the intercept point and a live estimation-error readout as
    # one colour-coded MarkerArray in 'map'. The display lives in the SIM's RViz
    # (rviz/interception.rviz, launched by interception.launch.py) — no separate
    # RViz here. Wall clock so it keeps refreshing if /clock stalls.
    viz = Node(
        package='drone_interception_sim',
        executable='pipeline_viz',
        name='pipeline_viz',
        namespace=ns,
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('target_odom', '/target/mavros/local_position/odom'),
            ('interceptor_odom', '/interceptor/mavros/local_position/odom'),
            ('target_estimate', '/interceptor/kf/good_tracks_pose_array'),
            ('predicted_path', '/interceptor/predicted_path'),
            ('planned_path', '/interceptor/reference_path'),
            ('intercept_point', '/interceptor/intercept_point'),
            ('markers', '/interceptor/interception/markers'),
        ],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                'orchestrator', default_value='fsm', description='fsm | offboard | bt | rl | none'
            ),
            DeclareLaunchArgument(
                'rl_model',
                default_value='',
                description='orchestrator:=rl: SB3 policy .zip '
                '(e.g. shaheen_rl/checkpoints/intercept/'
                'model_evasive_40k.zip)',
            ),
            DeclareLaunchArgument(
                'rl_velocity_frame',
                default_value='body',
                description='orchestrator:=rl: frame of the interceptor '
                'velocity the policy was trained on (body | world)',
            ),
            DeclareLaunchArgument(
                'detector',
                default_value='ground_truth',
                description='ground_truth | depth (ported blob search) | depth_cc (ground '
                'removal + connected components) | yolo',
            ),
            DeclareLaunchArgument(
                'gaze',
                default_value='false',
                description='yaw toward the target estimate instead of the direction of '
                'travel, so a forward-looking camera keeps it in view (vision detectors)',
            ),
            DeclareLaunchArgument(
                'min_altitude',
                default_value='-1000.0',
                description='planner: lowest reference altitude, m (the default leaves it '
                'as planned; vision runs, whose estimate can dip below ground: 2.0)',
            ),
            DeclareLaunchArgument(
                'cue',
                default_value='false',
                description='vision detectors: feed a noisy long-range target cue (radar / '
                'datalink stand-in) while the camera has no detection and the target is '
                'farther than cue_min_range',
            ),
            DeclareLaunchArgument(
                'cue_min_range',
                default_value='8.0',
                description='m: the cue is silent inside this range (keep it inside the '
                "camera's depth clip so the end game is vision only)",
            ),
            DeclareLaunchArgument('cue_noise', default_value='0.5', description='cue sigma, m'),
            DeclareLaunchArgument('estimator', default_value='const_vel'),
            DeclareLaunchArgument(
                'kf_sigma_a',
                default_value='10.0',
                description='estimator: std of the random acceleration, m/s^2 (process noise)',
            ),
            DeclareLaunchArgument(
                'kf_r',
                default_value='0.01',
                description='estimator: measurement variance per axis, m^2 (a vision or '
                'cue-fed run measures to a few tenths of a metre: 0.1)',
            ),
            # gru = the OLD implementation's trained predictor (pos+vel GRUs,
            # velocity-path, weights shipped in interception_prediction/models);
            # falls back to const_vel automatically if torch/weights are missing.
            DeclareLaunchArgument('predictor', default_value='gru'),
            DeclareLaunchArgument('planner', default_value='rendezvous'),
            DeclareLaunchArgument('mpc', default_value='passthrough'),
            DeclareLaunchArgument('controller', default_value='px4_setpoint'),
            DeclareLaunchArgument(
                'vehicle_mass',
                default_value='2.466',
                description='interceptor mass for the se3 controller, kg '
                '(x500_d435 incl. camera + gimbal per its SDF; '
                'camera-less x500: 2.064)',
            ),
            DeclareLaunchArgument(
                'hover_thrust',
                default_value='0.79',
                description="PX4's normalized hover thrust for the se3 "
                'controller (sets its max thrust = m g / hover; '
                'x500_d435 in SITL: 0.79, from hover_thrust_estimate)',
            ),
            DeclareLaunchArgument(
                'max_accel',
                default_value='12.0',
                description='se3 total acceleration limit, m/s^2 (x500_d435 '
                'tops out near 12.4 at full thrust)',
            ),
            DeclareLaunchArgument(
                'control_lookahead',
                default_value='',
                description='s into the command trajectory the control '
                "stage tracks ('' = se3 0.1; 1.0 after "
                'mpc_12state; px4_setpoint/pn_velocity 0.3)',
            ),
            DeclareLaunchArgument('takeoff_alt', default_value='4.0'),
            DeclareLaunchArgument('attack_distance', default_value='3.0'),
            DeclareLaunchArgument('capture_radius', default_value='1.0'),
            DeclareLaunchArgument(
                'referee',
                default_value='odom',
                description='metrics: odom | gazebo, judge range and capture on MAVROS '
                'odometry or on the poses Gazebo itself reports (the heads still fly on '
                'their estimates)',
            ),
            DeclareLaunchArgument(
                'capture_confirm',
                default_value='0.6',
                description='fsm head: s the estimated range must stay '
                'inside capture_radius to count as capture',
            ),
            DeclareLaunchArgument(
                'follow_through',
                default_value='0.0',
                description='fsm and bt heads: s to keep pursuing after their capture check '
                'passes before returning (the check uses odometry, up to about 1 m off)',
            ),
            DeclareLaunchArgument('metrics', default_value='false'),
            DeclareLaunchArgument('csv_path', default_value='/tmp/pipeline_metrics.csv'),
            DeclareLaunchArgument(
                'summary_csv_path',
                default_value='/tmp/interception_results.csv',
                description='per-run summary table (appended)',
            ),
            DeclareLaunchArgument(
                'method',
                default_value=[
                    'pipeline/',
                    LaunchConfiguration('planner'),
                    '+',
                    LaunchConfiguration('predictor'),
                    '+',
                    LaunchConfiguration('mpc'),
                    '+',
                    LaunchConfiguration('controller'),
                ],
                description='metrics label (default: the backends)',
            ),
            DeclareLaunchArgument(
                'scenario', default_value='', description='metrics label for the target scenario'
            ),
            DeclareLaunchArgument(
                'trial', default_value='-1', description='metrics label: trial index in a campaign'
            ),
            DeclareLaunchArgument(
                'viz',
                default_value='true',
                description='publish the interception MarkerArray ' '(shown in the sim RViz)',
            ),
            GroupAction(
                [
                    detection,
                    estimation,
                    prediction,
                    planning,
                    mpc,
                    control,
                    fsm,
                    offboard,
                    bt,
                    rl,
                    metrics,
                    viz,
                ]
            ),
        ]
    )
