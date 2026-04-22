clear;
close all;
clc;

%% ROS2 TURTLEBOT NAVIGATION=
% Subscribes to robot odometry, commands velocity via /cmd_vel, visualizes in real-time.


%% Mission Parameters
goal = [1; -0.2; 0];               % [x; y; theta] or [x; y]
tolerance = 0.05;                 % Position reached tolerance (m)
heading_tolerance = 0.08;         % Heading reached tolerance (rad), used if goal has theta
max_mission_time = 120;           % Mission timeout (s)
update_rate_hz = 10;              % Control loop frequency (Hz)
dt = 1 / update_rate_hz;


% Shared callback state
global g_pose g_pose_received g_scan g_scan_received g_last_odom_time
g_pose = [0; 0; 0];
g_pose_received = false;
g_scan = [];
g_scan_received = false;
g_last_odom_time = -inf;

%% ROS2 Initialization
fprintf('[INIT] Starting ROS2 node and topic connections...\n');

if isempty(getenv('ROS_DOMAIN_ID'))
    setenv('ROS_DOMAIN_ID', '30');
end
fprintf('[INIT] ROS_DOMAIN_ID=%s\n', getenv('ROS_DOMAIN_ID'));

% Select command topic for your TurtleBot setup.
% Most TurtleBot3 setups use /cmd_vel.
cmd_topic = "/cmd_vel";

% Create node
node = ros2node("turtlebot_navigator");

% Create subscriber to odometry
% TurtleBot3 publishes odometry on /odom topic
odom_sub = ros2subscriber(node, "/odom", @odomCallback);

% Create subscriber to LiDAR scan (best effort matches TB setup)
scan_sub = ros2subscriber(node, "/scan", @scanCallback, 'Reliability', 'besteffort');

% Create publisher for velocity commands
% TurtleBot3 expects commands on /cmd_vel topic
% Message type: geometry_msgs/Twist
vel_pub = ros2publisher(node, cmd_topic, "geometry_msgs/Twist");
vel_msg = ros2message('geometry_msgs/Twist');

% Give ROS time to discover topics
fprintf('[INIT] Waiting for topic discovery (5 seconds)...\n');
pause(5);

% Wait for first odometry message so we never drive blind.
wait_odom_timeout = 15;
t_wait = tic;
while ~g_pose_received && toc(t_wait) < wait_odom_timeout
    pause(0.05);
end
if ~g_pose_received
    error('[INIT] No odometry on /odom after %.1f s. Check bringup, network, and ROS_DOMAIN_ID.', wait_odom_timeout);
end
fprintf('[INIT] Odometry stream detected.\n');

if numel(goal) ~= 2 && numel(goal) ~= 3
    error('Goal must be [x; y] or [x; y; theta].');
end
goal = goal(:);

% Accept heading in degrees if user passes large magnitude (e.g., 180).
if numel(goal) == 3 && abs(goal(3)) > (2 * pi + 0.1)
    goal(3) = deg2rad(goal(3));
    fprintf('[INIT] Interpreting goal heading as degrees. Converted to %.3f rad (%.1f deg).\n', goal(3), rad2deg(goal(3)));
end

%% Initialize Visualization
plotter = plotTurtlebot('TurtleBot Real Navigation', [-0.5, 1.5], [-0.5, 1.5]);
plotter = plotter.updatePositionDesired(goal');

%% Initialize Controller
controller_state = [];  % Empty on first call triggers initialization in navigate()
avoid_state = [];       % Empty on first call triggers initialization in obstacleAvoidance()

%% Mission Clock
mission_start = tic;

%% Main Control Loop
fprintf('[START] Beginning mission. Driving to goal [%.3f, %.3f]\n', goal(1), goal(2));
if numel(goal) == 3
    fprintf('[START] Target heading: %.3f rad (%.1f deg)\n', goal(3), rad2deg(goal(3)));
end
fprintf('[INFO] Close the figure window or stop script execution in MATLAB to end.\n\n');

loop_count = 0;

try
    while toc(mission_start) < max_mission_time
        loop_count = loop_count + 1;
        t_elapsed = toc(mission_start);
        now_sec = now * 86400;

        if isempty(plotter.fig) || ~isvalid(plotter.fig)
            fprintf('\n[STOP] Figure closed by user. Stopping robot and ending script.\n');
            break;
        end

        % Detect stale odometry stream
        if (now_sec - g_last_odom_time) > 1.0
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            send(vel_pub, vel_msg);
            error('Odometry stream is stale (>1 s). Commanding stop for safety.');
        end
        
        % Read latest pose from callback state
        pose = g_pose;
        
        % Compute distance to goal
        dist_to_goal = hypot(goal(1) - pose(1), goal(2) - pose(2));
        
        % Check goal reached (position only or position+heading)
        if numel(goal) == 3
            heading_error_goal = wrapToPi(goal(3) - pose(3));
            reached_goal = (dist_to_goal <= tolerance) && (abs(heading_error_goal) <= heading_tolerance);
        else
            reached_goal = (dist_to_goal <= tolerance);
        end

        if reached_goal
            % Stop robot
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            send(vel_pub, vel_msg);
            
            fprintf('\n[SUCCESS] Goal reached at t=%.2f s\n', t_elapsed);
            fprintf('[FINAL] Pose: x=%.3f, y=%.3f, theta=%.3f\n', pose(1), pose(2), pose(3));
            break;
        end
        
        % Call PID navigation controller
        [v_cmd, w_cmd, controller_state] = navigate(pose, goal, controller_state, dt);

        % Reactive obstacle avoidance layer from LiDAR.
        [v_cmd, w_cmd, avoid_state, avoid_dbg] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state);

        % Publish velocity command to robot
        vel_msg.linear.x = v_cmd;      % Forward velocity (m/s)
        vel_msg.angular.z = w_cmd;     % Rotation velocity (rad/s)
        send(vel_pub, vel_msg);
        
        % Update visualization with real pose
        plotter = plotter.updatePose(pose(1:2)', pose(3));

        % Update LiDAR visualization (scan points transformed to world frame)
        if g_scan_received && ~isempty(g_scan)
            cart = rosReadCartesian(g_scan);
            if ~isempty(cart)
                R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
                cart_world = cart * R' + pose(1:2)';
                plotter = plotter.updateScan(cart_world);
            end
        end
        
        % Status output every 10 loops (~1 Hz at 10 Hz control)
        if mod(loop_count, 10) == 0
            fprintf('t=%6.2f s | pos=[%7.4f, %7.4f] | goal_dist=%6.3f m | v=%6.3f m/s, w=%6.3f rad/s | avoid=%d | front=%.2f\n', ...
                t_elapsed, pose(1), pose(2), dist_to_goal, v_cmd, w_cmd, avoid_dbg.avoid_mode, avoid_dbg.front_min);
        end

        % Wait to maintain control rate
        drawnow limitrate;
        pause(dt);
    end
    
    % Stop robot on exit
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    send(vel_pub, vel_msg);
    
catch ME
    % Emergency stop on error
    fprintf('\n[ERROR] %s\n', ME.message);
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    send(vel_pub, vel_msg);
    rethrow(ME);
end

function a = wrapToPi(a)
    a = mod(a + pi, 2 * pi) - pi;
end

if toc(mission_start) >= max_mission_time
    fprintf('\n[TIMEOUT] Mission exceeded %.1f seconds\n', max_mission_time);
end

fprintf('[END] Final stats: %d loops, %.2f s elapsed\n', loop_count, toc(mission_start));

% Cleanup
clear vel_pub odom_sub scan_sub node;

%% ========== CALLBACK FUNCTION ==========
function odomCallback(msg)
    % Callback for /odom subscriber
    % Extracts position (x, y) and orientation (theta) from odometry message
    
    global g_pose g_pose_received
    
    % Extract position from message
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    
    % Extract orientation (quaternion) and convert to yaw angle
    quat = msg.pose.pose.orientation;
    theta = quat2euler([quat.w, quat.x, quat.y, quat.z]);  % Returns [roll, pitch, yaw]
    theta = theta(3);  % Extract yaw
    
    g_pose = [x; y; theta];
    g_pose_received = true;
    global g_last_odom_time
    g_last_odom_time = now * 86400;
end

function scanCallback(msg)
    global g_scan g_scan_received
    g_scan = msg;
    g_scan_received = true;
end

%% ========== UTILITY: QUATERNION TO EULER ==========
function angles = quat2euler(q)
    % Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw]
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = pi/2 * sign(sinp);
    else
        pitch = asin(sinp);
    end
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    angles = [roll, pitch, yaw];
end
