clear;
close all;
clc;

%% ROS2 TURTLEBOT NAVIGATION (PRM ORCHESTRATOR)
% Loads a built SLAM map, plans PRM path, then tracks waypoints using PID
% plus reactive obstacle avoidance.

%% User Mission Parameters
map_input_file = 'slam_map_test_20260425_150348.mat';              % '' = latest slam_map_*.mat in Project/Maps
goal_input_xy = [2.8, 1.2];               % Goal input [x y]
goal_is_relative_to_start = true;         % true: goal = map_start + goal_input

% Simplest robust localization:
% - Put the robot at the physical map start pose before running this script.
% - The script snapshots odometry at startup and treats that as map_start_pose.
% - This avoids guessing x/y offsets and also handles heading offset.
map_start_pose = [0, 0, 0.0]; % [x y yaw] in map frame at script start

waypoint_tolerance = 0.12;        % Intermediate waypoint tolerance (m)
final_tolerance = 0.08;           % Final waypoint tolerance (m)

max_mission_time = 180;           % Mission timeout (s)
update_rate_hz = 10;              % Control loop frequency (Hz)
dt = 1 / update_rate_hz;

% Visualization (set to false/low for faster, less jittery control loop)
enable_visualization = true;
plot_scan_visualization = false;
viz_update_stride = 3;            % update plots every N loops

% Optional PRM planner overrides passed into navigatePRM.
prm_cfg = struct();
prm_cfg.numNodes = 350;
prm_cfg.connectionDistance = 0.55;
prm_cfg.retryNumNodes = 700;
prm_cfg.retryConnectionDistance = 0.75;
prm_cfg.inflateRadius = 0.08;
prm_cfg.simplifyEps = 0.08;


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

if numel(goal_input_xy) ~= 2
    error('goal_input_xy must be [x y].');
end
goal_input_xy = double(goal_input_xy(:)');

if numel(map_start_pose) ~= 3
    error('map_start_pose must be [x y yaw].');
end
map_start_pose = double(map_start_pose(:)');

if goal_is_relative_to_start
    goal_xy = map_start_pose(1:2) + goal_input_xy;
else
    goal_xy = goal_input_xy;
end

% Snapshot odometry reference at startup; this defines map frame origin.
odom_ref_pose = g_pose(:)';
start_xy = map_start_pose(1:2);
fprintf('[INIT] Startup odom reference: [%.3f, %.3f, %.3f]\n', odom_ref_pose(1), odom_ref_pose(2), odom_ref_pose(3));
fprintf('[INIT] PRM start in map frame: [%.3f, %.3f]\n', start_xy(1), start_xy(2));

%% PRM Planning from built SLAM map
prm_out = navigatePRM(map_input_file, start_xy, goal_xy, prm_cfg);
path = prm_out.path;
num_waypoints = size(path, 1);

if num_waypoints < 2
    error('PRM path has too few waypoints.');
end

fprintf('[PLAN] Map: %s\n', prm_out.mapFilePath);
fprintf('[PLAN] Waypoints: %d | Goal: [%.3f, %.3f]\n', num_waypoints, goal_xy(1), goal_xy(2));
fprintf('[PLAN] Localization mode: startup odom -> map_start_pose calibration\n');

%% Initialize Visualization
plotter = [];
if enable_visualization
    plotter = plotTurtlebot('TurtleBot Real Navigation', [-0.5, 1.5], [-0.5, 1.5]);
    plotter = plotter.updatePositionDesired(path(1, :));
end

%% Initialize Controller
controller_state = [];  % Empty on first call triggers initialization in navigatePID()
avoid_state = [];       % Empty on first call triggers initialization in obstacleAvoidance()

%% Mission Clock
mission_start = tic;
t_prev_loop = 0; % Track loop time for exact dt
mission_state = 'NAV_TO_B';

%% Main Control Loop
fprintf('[START] Beginning PRM mission from [%.3f, %.3f] to [%.3f, %.3f]\n', ...
    start_xy(1), start_xy(2), goal_xy(1), goal_xy(2));
fprintf('[INFO] Close the figure window or stop script execution in MATLAB to end.\n\n');

loop_count = 0;
waypoint_idx = 1;
goal_reached = false;

try
    while toc(mission_start) < max_mission_time
        loop_count = loop_count + 1;
        t_elapsed = toc(mission_start);
        
        % Extremely accurate, exact dt derivation (matching original loop completely)
        dt = t_elapsed - t_prev_loop;
        if dt <= 0; dt = 0.001; end
        t_prev_loop = t_elapsed;
        
        now_sec = now * 86400;

        if enable_visualization
            if isempty(plotter.fig) || ~isvalid(plotter.fig)
                fprintf('\n[STOP] Figure closed by user. Stopping robot and ending script.\n');
                break;
            end
        end

        % Detect stale odometry stream
        if (now_sec - g_last_odom_time) > 1.0
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            send(vel_pub, vel_msg);
            error('Odometry stream is stale (>1 s). Commanding stop for safety.');
        end
        
        % Read latest pose from callback state
        pose = odomToMapPose(g_pose(:)', odom_ref_pose, map_start_pose);

        % START PÅ STATE MACHINE
        switch mission_state
        
        % STATE 1: Kør fra Start (A) til Område B
        case 'NAV_TO_B'
            % Jeres eksisterende waypoint/PID logik her
            [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp, controller_state, dt);
            [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);
            
            if goal_reached % (Jeres logik fra før, når waypoint_idx > num_waypoints)
                fprintf('[STATE] Ankommet til område B. Skifter til SEARCH_TARGET.\n');
                
                % Stop robotten inden vi skifter state
                v_cmd = 0; w_cmd = 0;
                mission_state = 'SEARCH_TARGET';
            end
            
        % ==========================================
        % STATE 2: Roter og led efter cirkel
        % ==========================================
        case 'SEARCH_TARGET'
            % Drej robotten langsomt rundt om sig selv
            v_cmd = 0.0;
            w_cmd = 0.3; % Drej med 0.3 rad/s
            
            % Kald en NY funktion til billedbehandling
            % [circle_detected, pixel_offset] = detectTarget(g_camera_image);
            
            circle_detected = false; % Placeholder
            
            if circle_detected
                fprintf('[STATE] Cirkel fundet! Skifter til ALIGN_AND_PHOTO.\n');
                v_cmd = 0; w_cmd = 0;
                mission_state = 'ALIGN_AND_PHOTO';
            end
            
        % ==========================================
        % STATE 3: Placer 1 meter foran og tag billede
        % ==========================================
        case 'ALIGN_AND_PHOTO'
            % Finjuster position baseret på kamera-data og tag billedet
            % (Her skal I bygge logikken til at holde afstanden på 1m)
            
            photo_taken = true; % Placeholder
            
            if photo_taken
                fprintf('[STATE] Billede taget. Skifter til NAV_TO_C.\n');
                % Generer en NY PRM rute fra nuværende position til Område C
                % prm_out = navigatePRM(map_input_file, pose(1:2), goal_C_xy, prm_cfg);
                % path = prm_out.path;
                % waypoint_idx = 1;
                
                mission_state = 'NAV_TO_C';
            end
            
        % ==========================================
        % STATE 4: Kør til Område C
        % ==========================================
        case 'NAV_TO_C'
             % Genbrug PID/Obstacle logikken fra NAV_TO_B
             % Følg den nye sti
             
             if goal_reached
                 fprintf('[SUCCESS] Mission fuldført. Robot i område C.\n');
                 break; % Stop programmet
             end
             
    end % Slut på switch-case
    
    % Send v_cmd og w_cmd til robotten (som I gør nu)
    vel_msg.linear.x = v_cmd;
    vel_msg.angular.z = w_cmd;
    send(vel_pub, vel_msg);














        % Determine current target waypoint and tolerance.
        target_wp = path(waypoint_idx, :);
        if enable_visualization && mod(loop_count, viz_update_stride) == 0
            plotter = plotter.updatePositionDesired(target_wp);
        end

        dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
        if waypoint_idx == num_waypoints
            active_tol = final_tolerance;
        else
            active_tol = waypoint_tolerance;
        end

        if dist_to_wp <= active_tol
            waypoint_idx = waypoint_idx + 1;

            % Reset PID memory at waypoint transitions to reduce jitter from
            % carried integral/derivative state between sharp segment changes.
            controller_state = [];

            if waypoint_idx > num_waypoints
                goal_reached = true;
                vel_msg.linear.x = 0;
                vel_msg.angular.z = 0;
                send(vel_pub, vel_msg);

                fprintf('\n[SUCCESS] Final waypoint reached at t=%.2f s\n', t_elapsed);
                fprintf('[FINAL] Pose(map): x=%.3f, y=%.3f, theta=%.3f\n', pose(1), pose(2), pose(3));
                break;
            end

            target_wp = path(waypoint_idx, :);
            dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
        end

        % PID waypoint tracker (position-only target).
        [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);

        % Reactive obstacle avoidance layer.
        [v_cmd, w_cmd, avoid_state, avoid_dbg] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);

        % Publish velocity command to robot
        vel_msg.linear.x = v_cmd;      % Forward velocity (m/s)
        vel_msg.angular.z = w_cmd;     % Rotation velocity (rad/s)
        send(vel_pub, vel_msg);
        
        % Update visualization with throttling to reduce control-loop jitter.
        if enable_visualization && mod(loop_count, viz_update_stride) == 0
            plotter = plotter.updatePose(pose(1:2)', pose(3));

            if plot_scan_visualization && g_scan_received && ~isempty(g_scan)
                cart = rosReadCartesian(g_scan);
                if ~isempty(cart)
                    R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
                    cart_world = cart * R' + pose(1:2)';
                    plotter = plotter.updateScan(cart_world);
                end
            end
        end
        
        % Status output every 10 loops (~1 Hz at 10 Hz control)
        if mod(loop_count, 10) == 0
            fprintf('t=%6.2f s | wp=%d/%d | pos=[%7.4f, %7.4f] | wp_dist=%6.3f m | v=%6.3f m/s, w=%6.3f rad/s | avoid=%d | front=%.2f\n', ...
                t_elapsed, waypoint_idx, num_waypoints, pose(1), pose(2), dist_to_wp, v_cmd, w_cmd, avoid_dbg.avoid_mode, avoid_dbg.front_min);
        end

        % Wait to maintain configured control rate.
        if enable_visualization && mod(loop_count, viz_update_stride) == 0
            drawnow limitrate;
        end
        loop_elapsed = toc(mission_start) - t_elapsed;
        pause(max(0, (1 / update_rate_hz) - loop_elapsed));
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

if toc(mission_start) >= max_mission_time
    fprintf('\n[TIMEOUT] Mission exceeded %.1f seconds\n', max_mission_time);
end

if ~goal_reached
    fprintf('[END] Mission ended without reaching final waypoint.\n');
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

function poseMap = odomToMapPose(poseOdom, odomRef, mapStart)
% Convert live odom pose into map frame using startup calibration.
% 1) Compute delta pose in odom frame from startup.
dx = poseOdom(1) - odomRef(1);
dy = poseOdom(2) - odomRef(2);
dth = wrapToPiLocal(poseOdom(3) - odomRef(3));

% 2) Rotate delta into map start orientation and add map start translation.
c = cos(mapStart(3));
s = sin(mapStart(3));
dxyMap = [c, -s; s, c] * [dx; dy];

poseMap = [mapStart(1) + dxyMap(1), mapStart(2) + dxyMap(2), wrapToPiLocal(mapStart(3) + dth)];
poseMap = poseMap(:);
end

function a = wrapToPiLocal(a)
a = mod(a + pi, 2 * pi) - pi;
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
