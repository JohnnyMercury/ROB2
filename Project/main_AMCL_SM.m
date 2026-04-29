clear;
close all;
clc;

%% ROS2 TURTLEBOT NAVIGATION (PRM ORCHESTRATOR)
% Loads a built SLAM map, plans PRM path, then tracks waypoints using PID
% plus reactive obstacle avoidance, now backed by AMCL Drift Correction.
% Tasks: 
% 1. Navigate from Area A to B
% 2. Search for and align 1m in front of an orange and blue circle.
% 3. Take a picture.
% 4. Navigate from Area B to Area C.

%% User Mission Parameters
map_input_file = 'slam_map_test_20260429_091016.mat';  % '' = latest slam_map_*.mat in Project/Maps
map_start_pose = [0, 0, 0.0]; % [x y yaw] in map frame at script start

% FILL THESE IN WITH COORDINATES FOR AREA B AND C
goal_B = [5.0, 2.0]; % Goal in Area B
goal_C = [8.0, -1.0]; % Goal in Area C

% Navigation
enable_amcl = true;               % Enable LiDAR Scan Matching to eliminate Odometry drift
waypoint_tolerance = 0.12;        % Intermediate waypoint tolerance (m)
final_tolerance = 0.08;           % Final waypoint tolerance (m)
max_mission_time = 180;           % Mission timeout (s)
update_rate_hz = 10;              % Control loop frequency (Hz)
dt = 1 / update_rate_hz;

% Visualization 
enable_visualization = true;
plot_scan_visualization = true;   % Turn on to see the scan matching the map
viz_update_stride = 3;            % update plots every N loops

% PRM planner overrides
prm_cfg = struct();
prm_cfg.numNodes = 350;
prm_cfg.connectionDistance = 0.55;
prm_cfg.retryNumNodes = 700;
prm_cfg.retryConnectionDistance = 0.75;
prm_cfg.inflateRadius = 0.08;
prm_cfg.simplifyEps = 0.08;

% Vision 
H = 0.10;           % Real circle diameter (meters)
f = 1226.5;         % Camera focal length (pixels)
target_distance = 1.0; 
Kp_dist = 0.5;      
Kp_angle = 0.002;

% Shared callback state
global g_pose g_pose_received g_scan g_scan_received g_last_odom_time
global g_image g_image_received
g_pose = [0; 0; 0];
g_pose_received = false;
g_scan = [];
g_scan_received = false;
g_image = [];
g_image_received = false;
g_last_odom_time = -inf;

%% ROS2 Initialization
fprintf('[INIT] Starting ROS2 node and topic connections...\n');
if isempty(getenv('ROS_DOMAIN_ID'))
    setenv('ROS_DOMAIN_ID', '30');
end

cmd_topic = "/cmd_vel";

% Create node and topics
node = ros2node("turtlebot_navigator");
odom_sub = ros2subscriber(node, "/odom", @odomCallback);
scan_sub = ros2subscriber(node, "/scan", @scanCallback, 'Reliability', 'besteffort');
image_sub = ros2subscriber(node, '/camera/image_raw/compressed', @imageCallback);
vel_pub = ros2publisher(node, cmd_topic, "geometry_msgs/Twist");
vel_msg = ros2message('geometry_msgs/Twist');

fprintf('[INIT] Waiting for topic discovery (5 seconds)...\n');
pause(5);

% Odometry timeout validation
wait_odom_timeout = 15;
t_wait = tic;
while ~g_pose_received && toc(t_wait) < wait_odom_timeout
    pause(0.05);
end
if ~g_pose_received
    error('[INIT] No odometry on /odom after %.1f s. Check bringup, network, and ROS_DOMAIN_ID.', wait_odom_timeout);
end
fprintf('[INIT] Odometry stream detected.\n');

% Input validation
if numel(goal_B) ~= 2 || numel(goal_C) ~= 2
    error('Goals must be [x y].');
end
goal_B = double(goal_B(:)');
goal_C = double(goal_C(:)');
map_start_pose = double(map_start_pose(:)');

% Snapshot odometry reference at startup
odom_ref_pose = g_pose(:)';
start_xy = map_start_pose(1:2);
fprintf('[INIT] Startup odom reference: [%.3f, %.3f, %.3f]\n', odom_ref_pose(1), odom_ref_pose(2), odom_ref_pose(3));

%% PRM Planning from built SLAM map
% Initial PRM planning to Area B
fprintf('[PLAN] Planning initial path to Area B...\n');
prm_out = navigatePRM(map_input_file, start_xy, goal_B, prm_cfg);
path = prm_out.path;
num_waypoints = size(path, 1);
waypoint_idx = 1;

if num_waypoints < 2
    error('PRM path has too few waypoints.');
end

%% AMCL Localization Initialization
use_amcl = false;
if enable_amcl
    fprintf('[INIT] Setting up AMCL for drift correction...\n');
    try
        % Load uninflated map for localization matching
        S_map = load(prm_out.mapFilePath);
        if isfield(S_map.output, 'mapCleanProb')
            locMap = S_map.output.mapCleanProb;
        else
            locMap = S_map.output.rawOccMap;
        end
        
        mcl = monteCarloLocalization;
        mcl.UseLidarScan = true;
        
        odomModel = odometryMotionModel;
        odomModel.Noise = [0.1 0.1 0.1 0.1];
        mcl.MotionModel = odomModel;
        
        sensorModel = likelihoodFieldSensorModel;
        sensorModel.SensorLimits = [0.08 3.5];
        sensorModel.Map = locMap;
        mcl.SensorModel = sensorModel;
        
        % Only run heavy update math when moving to save loop time (10cm or ~5.7 deg)
        mcl.UpdateThresholds = [0.10, 0.10, 0.10]; 
        mcl.ResamplingInterval = 1;
        mcl.ParticleLimits = [200 1000];
        mcl.GlobalLocalization = false;
        mcl.InitialPose = map_start_pose;
        mcl.InitialCovariance = diag([0.02, 0.02, 0.01]);
        
        use_amcl = true;
        fprintf('[INIT] AMCL initialized successfully.\n');
    catch ME
        fprintf('[WARN] Failed to initialize AMCL. Using pure odometry. Error: %s\n', ME.message);
    end
end

% Set up persistent transforms to track the drift. 
% T_M_O stores the rigid transform to map raw Odometry frame to the True Map frame.
T_M_R_init = pose2tform2D(map_start_pose);
T_O_R_init = pose2tform2D(odom_ref_pose);
T_M_O = T_M_R_init / T_O_R_init; % Matrix Right Division (T_M_R_init * inv(T_O_R_init))


%% Initialize Visualization
plotter = [];
if enable_visualization
    plotter = plotTurtlebot('TurtleBot Real Navigation', [-0.5, 1.5], [-0.5, 1.5]);
    plotter = plotter.updatePositionDesired(path(1, :));
end

%% MAIN CONTROL LOOP (STATE MACHINE)
state = 'NAV_TO_B';
controller_state = [];  
avoid_state = [];       

% Mission Clock
mission_start = tic;
t_prev_loop = 0;
loop_count = 0;

fprintf('[START] Beginning state machine mission...\n');

try
    while toc(mission_start) < max_mission_time
        loop_count = loop_count + 1;
        t_elapsed = toc(mission_start);
        dt = max(0.001, t_elapsed - t_prev_loop);
        t_prev_loop = t_elapsed;
        now_sec = now * 86400;

        if enable_visualization
            if isempty(plotter.fig) || ~isvalid(plotter.fig)
                fprintf('\n[STOP] Figure closed by user. Stopping robot.\n');
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
        
        % ----------------------------------------------------
        % 1. LOCALIZATION (Drift-Corrected Pose Tracking)
        % ----------------------------------------------------
        % Read live Odometry
        odom_pose = g_pose(:)';
        T_O_R = pose2tform2D(odom_pose);
        
        % Process Scan Matching dynamically to align Map 
        if use_amcl && g_scan_received && ~isempty(g_scan)
            try
                scan_obj = rosReadLidarScan(g_scan);
                ranges = double(scan_obj.Ranges(:));
                angles = double(scan_obj.Angles(:));
                valid = isfinite(ranges) & ranges > 0.08 & ranges < 3.48;
                
                if sum(valid) > 20
                    scan_clean = lidarScan(ranges(valid), angles(valid));
                    % Update Particle Filter
                    [isUpdated, estPose, ~] = mcl(odom_pose, scan_clean);
                    if isUpdated
                        % AMCL provided a correction! Calculate the new Map-to-Odom relationship
                        T_M_R_mcl = pose2tform2D(estPose);
                        T_M_O = T_M_R_mcl / T_O_R; 
                    end
                end
            catch
                % Silently ignore bad frames to keep control loop running uninterrupted
            end
        end
        
        % Calculate smoothed robot pose from drift transform + live odometry 
        % This guarantees the loop operates at 10Hz smoothly even when MCL pauses
        T_M_R = T_M_O * T_O_R;
        pose = tform2pose2D(T_M_R);
        v_cmd = 0; w_cmd = 0;

        % ----------------------------------------------------
        % 2. WAYPOINT TRACKING
        % ----------------------------------------------------
        switch state
            case 'NAV_TO_B'
                %Task: Drive from area A to area B
                target_wp = path(waypoint_idx, :);
                dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                
                active_tol = waypoint_tolerance;

                if enable_visualization && mod(loop_count, viz_update_stride) == 0
                    plotter = plotter.updatePositionDesired(target_wp);
                end

                if waypoint_idx == num_waypoints; active_tol = final_tolerance; end

                if dist_to_wp <= active_tol
                    waypoint_idx = waypoint_idx + 1;
                    controller_state = [];

                if dist_to_wp <= active_tol
                    waypoint_idx = waypoint_idx + 1;
                    controller_state = [];

                    if waypoint_idx > num_waypoints
                        fprintf('[STATE] Area B reached. Switching to SEARCH_CIRCLE.\n');
                        state = 'SEARCH_CIRCLE';
                    else
                        target_wp = path(waypoint_idx, :);
                        dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                    end
                end

                if strcmp(state, 'NAV_TO_B')
                    [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
                    [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);
                end

            case 'SEARCH_CIRCLE'
                % Task: Rotate to find the targets
                v_cmd = 0.0; w_cmd = 0.3; % Slow rotation
                
                if g_image_received && ~isempty(g_image)
                    image_rgb = flip(g_image, 1);
                    [centersO, ~, centersB, ~] = detectCircles(image_rgb);
                    if ~isempty(centersO) || ~isempty(centersB)
                        fprintf('[STATE] Target detected! Switching to ALIGN_AND_PICTURE.\n');
                        state = 'ALIGN_AND_PICTURE';
                    end
                end

            case 'ALIGN_AND_PICTURE'
                % Task: Position exactly 1m in front of target and snap photo
                if g_image_received && ~isempty(g_image)
                    image_rgb = flip(g_image, 1);
                    [centersO, radiiO, centersB, radiiB] = detectCircles(image_rgb);
                    img_width = size(image_rgb, 2); 
                    
                    active_dist = []; active_center_x = [];
                    if ~isempty(centersO)
                        active_dist = f * H / (2 * radiiO(1));
                        active_center_x = centersO(1, 1);
                    elseif ~isempty(centersB)
                        active_dist = f * H / (2 * radiiB(1));
                        active_center_x = centersB(1, 1);
                    end
                    
                    if ~isempty(active_dist)
                        err_dist = active_dist - target_distance; 
                        err_angle = (img_width / 2) - active_center_x; 
                        
                        v_cmd = Kp_dist * err_dist;
                        w_cmd = Kp_angle * err_angle;
                        
                        % Velocity constraints for safety
                        v_cmd = max(min(v_cmd, 0.15), -0.15); 
                        w_cmd = max(min(w_cmd, 0.5), -0.5); 
                        
                        % Threshold for taking the picture
                        if abs(err_dist) < 0.05 && abs(err_angle) < 20
                            fprintf('[STATE] In position. Taking picture...\n');
                            vel_msg.linear.x = 0; vel_msg.angular.z = 0; send(vel_pub, vel_msg);
                            imwrite(image_rgb, 'TB_Mission_Success.png');
                            pause(1.5);
                            
                            fprintf('[PLAN] Re-planning for Area C...\n');
                            prm_out = navigatePRM(map_input_file, pose(1:2)', goal_C, prm_cfg);
                            path = prm_out.path;
                            num_waypoints = size(path, 1);
                            waypoint_idx = 1;
                            if enable_visualization; plotter = plotter.updatePositionDesired(path(1,:)); end
                            state = 'NAV_TO_C';
                        end
                    else
                        state = 'SEARCH_CIRCLE'; % Target lost, search again
                    end
                end

            case 'NAV_TO_C'
                % Task: Follow path to final destination
                target_wp = path(waypoint_idx, :);
                dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                
                active_tol = waypoint_tolerance;
                if waypoint_idx == num_waypoints; active_tol = final_tolerance; end
                
                if dist_to_wp <= active_tol
                    waypoint_idx = waypoint_idx + 1;
                    controller_state = [];
                    if waypoint_idx > num_waypoints
                        fprintf('[STATE] Mission Complete! Arrived at Area C.\n');
                        state = 'DONE';
                    else
                        target_wp = path(waypoint_idx, :);
                        if enable_visualization; plotter = plotter.updatePositionDesired(target_wp); end
                    end
                end
                
                if strcmp(state, 'NAV_TO_C')
                    [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
                    [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);
                end

            case 'DONE'
                v_cmd = 0; w_cmd = 0;
                break;
        end

        % Publish
        vel_msg.linear.x = v_cmd;      
        vel_msg.angular.z = w_cmd;     
        send(vel_pub, vel_msg);
        
        % ----------------------------------------------------
        % 3. VISUALIZATION & THROTTLING
        % ----------------------------------------------------
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
            
            % Draw AMCL Cloud to verify drift correction convergence
            if use_amcl
                try plotter = plotter.updateParticles(mcl.Particles);
                catch
                end
            end
            
            drawnow limitrate;
        end
        
        if mod(loop_count, 10) == 0
            fprintf('t=%6.2f s | wp=%d/%d | pos=[%7.4f, %7.4f] | wp_dist=%6.3f m | v=%6.3f, w=%6.3f\n', ...
                t_elapsed, waypoint_idx, num_waypoints, pose(1), pose(2), dist_to_wp, v_cmd, w_cmd);
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
clear vel_pub odom_sub scan_sub node;

%% ========== CALLBACK FUNCTIONS ==========
function odomCallback(msg)
    global g_pose g_pose_received
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    
    quat = msg.pose.pose.orientation;
    theta = quat2euler([quat.w, quat.x, quat.y, quat.z]); 
    theta = theta(3);  
    
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

function imageCallback(msg)
    global g_image g_image_received
    g_image = rosReadImage(msg); g_image_received = true;
end

%% FUNCTION FOR VISION
function [centersO, radiiO, centersB, radiiB] = detectCircles(rgb)
    hsv = rgb2hsv(imgaussfilt(rgb, 1));
    % Thresholds for Orange and Blue
    maskO = (hsv(:,:,1) >= 0.05 & hsv(:,:,1) <= 0.15) & hsv(:,:,2) >= 0.5 & hsv(:,:,3) >= 0.2;
    maskB = (hsv(:,:,1) >= 0.55 & hsv(:,:,1) <= 0.65) & hsv(:,:,2) >= 0.5 & hsv(:,:,3) >= 0.2;
    se = strel('disk', 5);
    [centersO, radiiO] = imfindcircles(imclose(imopen(maskO, se), se), [20 200], 'ObjectPolarity', 'bright');
    [centersB, radiiB] = imfindcircles(imclose(imopen(maskB, se), se), [20 200], 'ObjectPolarity', 'bright');
end

%% ========== UTILITY: QUATERNION TO EULER ==========
function angles = quat2euler(q)
    w = q(1); x = q(2); y = q(3); z = q(4);
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = pi/2 * sign(sinp);
    else
        pitch = asin(sinp);
    end
    
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    angles = [roll, pitch, yaw];
end

%% ========== UTILITY: TRANSFORMS ==========
% Calculates Rigid Transformations to handle Frame Tracking
function T = pose2tform2D(p)
    c = cos(p(3));
    s = sin(p(3));
    T = [c, -s, p(1);
         s,  c, p(2);
         0,  0, 1];
end

function p = tform2pose2D(T)
    p = [T(1,3); T(2,3); atan2(T(2,1), T(1,1))];
    p = p(:);
end