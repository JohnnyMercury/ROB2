clear;
close all;
clc;

%% ROS2 TURTLEBOT NAVIGATION (PRM ORCHESTRATOR)
% Loads a built SLAM map, plans PRM path, then tracks waypoints using PID
% plus reactive obstacle avoidance.

%% 1. User Mission Parameters
map_input_file = 'slam_map_test_20260428_150643.mat'; % '' = latest slam_map_*.mat in Project/Maps
goal_B_xy = [1, 1.2];               % !!! Koordinater for område B
goal_C_xy = [-1.0, -1.0];              % !!! Koordinater for område C
map_start_pose = [0, 0, 0.0];         % [x y yaw] robotten starter i (Map frame)

% Tolerance (m)
waypoint_tolerance = 0.12;        % Intermediate waypoint tolerance (m)
final_tolerance = 0.08;           % Final waypoint tolerance (m)
max_mission_time = 300;           % Mission timeout (s)
update_rate_hz = 10;              % Control loop frequency (Hz)
dt = 1 / update_rate_hz;

% Visualization
enable_visualization = true;
viz_update_stride = 3;            % Update plots every N loops

% Mission State
mission_state = 'PLAN_TO_B';      % Start

% Shared callback state
global g_pose g_pose_received g_scan g_scan_received g_last_odom_time
g_pose = [0; 0; 0];
g_pose_received = false;
g_scan = [];
g_scan_received = false;
g_last_odom_time = -inf;

%% 2. ROS2 Initialization
fprintf('[INIT] Starting ROS2 node and topic connections...\n');

if isempty(getenv('ROS_DOMAIN_ID'))
    setenv('ROS_DOMAIN_ID', '30');
end
fprintf('[INIT] ROS_DOMAIN_ID=%s\n', getenv('ROS_DOMAIN_ID'));

node = ros2node("turtlebot_navigator"); % Create node
odom_sub = ros2subscriber(node, "/odom", @odomCallback); % Create subscriber to odometry
scan_sub = ros2subscriber(node, "/scan", @scanCallback, 'Reliability', 'besteffort'); % Create subscriber to LiDAR scan
% !!! Husk at tilføje kamera subscriber her senere:
% img_sub = ros2subscriber(node, "/camera/image_raw");

vel_pub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
vel_msg = ros2message('geometry_msgs/Twist');

% Give ROS time to discover topics
fprintf('[INIT] Waiting for topic discovery (5 seconds)...\n');
pause(5);

% Wait for first odometry message
t_wait = tic;
while ~g_pose_received && toc(t_wait) < 15
    pause(0.1);
end
if ~g_pose_received
    error('[INIT] No odometry');
end

% Snapshot odometry reference at startup for map calibration.
odom_ref_pose = g_pose(:)';
fprintf('[INIT] Calibration done. Ready.\n');


%% 4. Main Control Loop
mission_start = tic;
t_prev_loop = 0;
loop_count = 0;
path = [];
waypoint_idx = 1;
controller_state = [];
avoid_state = [];

fprintf('[START] Mission started. STATE: %s\n', mission_state);

try
    while toc(mission_start) < max_mission_time
        loop_count = loop_count + 1;
        t_elapsed = toc(mission_start);
        dt = t_elapsed - t_prev_loop; % Extremely accurate, exact dt derivation
        if dt <= 0; dt = 0.001; end
        t_prev_loop = t_elapsed;

        if (now * 86400 - g_last_odom_time) > 1.0
            stopRobot(vel_pub, vel_msg);
            error('Odometry stream is stale (>1 s). Commanding stop for safety.');
        end
        
        % ===================
        % STATE MACHINE START
        % ===================
        switch mission_state
        
        % STATE 1: Planning route from area A to area B
        case 'PLAN_TO_B'
            fprintf('[PLAN] Planning route to area B...\n');
                prm_out = navigatePRM(map_input_file, pose(1:2)', goal_B_xy, struct('numNodes', 350));
                path = prm_out.path;
                waypoint_idx = 1;
                mission_state = 'NAVIGATING_TO_B';
                
                if enable_visualization
                    plotter = plotTurtlebot('Mission Monitor', [-1, 6], [-3, 3]);
                end

            % STATE 2: Driving from area A to area B 
            case 'NAVIGATING_TO_B'
                % Standard navigation towards the goal
                [v_cmd, w_cmd, waypoint_idx, reached] = followPath(pose, path, waypoint_idx, dt, waypoint_tolerance, final_tolerance);
                
                if reached
                    fprintf('[STATE] Reached area B! Searching for orange circle.\n');
                    mission_state = 'SEARCH_TARGET';
                    v_cmd = 0; w_cmd = 0;
                end
            
            % STATE 3: Searching for orange circle
            case 'SEARCH_TARGET'
                % The robot rotates slowly to find the circle
                v_cmd = 0;
                w_cmd = 0.3; % rad/s
                
                % !!! HER SKAL VI KALDE BILLED-FUNKTION
                % [found, offset] = detectCircle(latest_image);
                found = false; % Placeholder
                
                if found
                    fprintf('[STATE] Circle detected! Changing posision...\n');
                    mission_state = 'ALIGN_AND_PHOTO';
                end
                
            % STATE 4: Align 1 meter from target and take photo
            case 'ALIGN_AND_PHOTO'
                v_cmd = 0; w_cmd = 0;
                fprintf('[IMAGE] Taking picture of circle...\n');
                
                mission_state = 'PLAN_TO_C';

            % STATE 5: Planning route from area B to area C
            case 'PLAN_TO_C'
                fprintf('[PLAN] Planning route to area C...\n');
                prm_out = navigatePRM(map_input_file, pose(1:2)', goal_C_xy, struct('numNodes', 350));
                path = prm_out.path;
                waypoint_idx = 1;
                mission_state = 'NAVIGATING_TO_C';

            % STATE 6: Driving from area B to area C
            case 'NAVIGATING_TO_C'
                [v_cmd, w_cmd, waypoint_idx, reached] = followPath(pose, path, waypoint_idx, dt, waypoint_tolerance, final_tolerance);
                
                if reached
                    fprintf('[SUCCESS] Mission completed! Robot is in area C.\n');
                    stopRobot(vel_pub, vel_msg);
                    break; 
                end
        end

      
        % Reactive obstacle avoidance layer.
        if ~strcmp(mission_state, 'SEARCH_TARGET') && ~strcmp(mission_state, 'ALIGN_AND_PHOTO')
             [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);
        end

        % Publish velocity command to robot
        vel_msg.linear.x = v_cmd;      % Forward velocity (m/s)
        vel_msg.angular.z = w_cmd;     % Rotation velocity (rad/s)
        send(vel_pub, vel_msg);
        
        % Update visualization with throttling to reduce control-loop jitter.
        if enable_visualization && mod(loop_count, viz_update_stride) == 0
            plotter = plotter.updatePose(pose(1:2)', pose(3));
            if ~isempty(path) && waypoint_idx <= size(path, 1)
                plotter = plotter.updatePositionDesired(path(waypoint_idx, :));
            end

            if g_scan_received && ~isempty(g_scan)
                cart = rosReadCartesian(g_scan);
                if ~isempty(cart)
                    R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
                    cart_world = cart * R' + pose(1:2)';
                    plotter = plotter.updateScan(cart_world);
                end
            end 
            drawnow limitrate;
        end

        pause(max(0, (1/update_rate_hz) - (toc(mission_start) - t_elapsed))); 
    end
    
catch ME
    stopRobot(vel_pub, vel_msg);
    rethrow(ME);
end

stopRobot(vel_pub, vel_msg);

%% ========== FUNCTIONS ==========

function [v, w, idx, reached] = followPath(pose, path, idx, dt, tol, final_tol)
    reached = false;
    target = path(idx, :);
    dist = hypot(target(1)-pose(1), target(2)-pose(2));

    active_tol = tol;
    if idx == size(path, 1); active_tol = final_tol; end
    
    if dist <= active_tol
        if idx >= size(path, 1)
            reached = true;
            v = 0; w = 0; return;
        end
        idx = idx + 1;
        target = path(idx, :);
    end

    persistent controller_state; 
    [v, w, controller_state] = navigatePID(pose, target(:), controller_state, dt);
end


function stopRobot(pub, msg)
    msg.linear.x = 0;
    msg.angular.z = 0;
    send(pub, msg);
    fprintf('[STOP] Robot stoppet.\n');
end


function odomCallback(msg)
    global g_pose g_pose_received g_last_odom_time
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    quat = msg.pose.pose.orientation;
    % Simpel quat2yaw
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    cosy_cosp = 1 - 2 * (quat.y^2 + quat.z^2);
    theta = atan2(siny_cosp, cosy_cosp);
    g_pose = [x; y; theta];
    g_pose_received = true;
    g_last_odom_time = now * 86400;
end

function scanCallback(msg)
    global g_scan g_scan_received
    g_scan = msg;
    g_scan_received = true;
end

function poseMap = odomToMapPose(poseOdom, odomRef, mapStart)
dx = poseOdom(1) - odomRef(1);
dy = poseOdom(2) - odomRef(2);
dth = wrapToPiLocal(poseOdom(3) - odomRef(3));
c = cos(mapStart(3)); 
s = sin(mapStart(3));
dxyMap = [c, -s; s, c] * [dx; dy];
poseMap = [mapStart(1) + dxyMap(1), mapStart(2) + dxyMap(2), mapStart(3) + dth];
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2 * pi) - pi;
end