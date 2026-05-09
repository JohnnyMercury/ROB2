clear;
close all;
clc;

%% ROS2 TURTLEBOT NAVIGATION (PRM + AMCL ORCHESTRATOR)
% Loads a built SLAM map, localizes via Monte Carlo Localization (AMCL),
% plans PRM path, then tracks waypoints using PID + Obstacle Avoidance.

%% User Mission Parameters
map_input_file = 'edited_custom_map.mat';  
map_start_pose = [0, 0, 0.0]; % [x y yaw] in map frame at script start
goal_B = [20.35, 9.24];  % Goal in Area B 
goal_C = [17.31, 17.26];  % Goal in Area C 

% Navigation
waypoint_tolerance = 0.12;        
final_tolerance = 0.08;           
max_mission_time = 3000;           
update_rate_hz = 10;              
dt = 1 / update_rate_hz;

% Visualization 
enable_visualization = true;
plot_scan_visualization = true;   
viz_update_stride = 3;            

% PRM planner overrides
prm_cfg = struct();
prm_cfg.numNodes = 1000;
prm_cfg.connectionDistance = 2.00;
prm_cfg.retryNumNodes = 700;
prm_cfg.retryConnectionDistance = 0.75;
prm_cfg.inflateRadius = 0.20;
prm_cfg.simplifyEps = 0.40; 

% Vision 
H = 0.10;           % Real circle diameter (meters)
f = 1226.5;         % Camera focal length (pixels)
target_distance = 1.0; 
Kp_dist = 0.5;      
Kp_angle = 0.002;

found_orange = false; 
found_blue = false;  
target_color_focus =''; 
search_waypoints = [20.405, 11.125; 20.348, 14.104; 20.348, 18.171]; % Area B 
current_search_idx = 1;
spin_progress = 0;  

% Shared callback state
global g_pose g_pose_received g_scan g_scan_received
global g_image g_image_received
g_pose = [0; 0; 0];
g_pose_received = false;
g_scan = [];
g_scan_received = false;
g_image = [];
g_image_received = false;

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
goal_B = double(goal_B(:)');
goal_C = double(goal_C(:)');
map_start_pose = double(map_start_pose(:)');
start_xy = map_start_pose(1:2);

%% Load Map and Initialize AMCL
fprintf('[INIT] Loading Map for AMCL from %s...\n', map_input_file);
mapData = load(map_input_file);
if isfield(mapData, 'output') && isfield(mapData.output, 'mapCleanBinary')
    amcl_map = mapData.output.mapCleanBinary;
else
    error('Could not find output.mapCleanBinary in the specified map file.');
end

fprintf('[INIT] Initializing Monte Carlo Localization...\n');
amcl = monteCarloLocalization;

% Configure Sensor Model for TB3 Burger
sm = likelihoodFieldSensorModel;
sm.Map = amcl_map;
sm.SensorLimits = [0.12 3.5]; % TB3 Lidar min/max range
amcl.SensorModel = sm;

% Configure Motion Model
amcl.MotionModel = odometryMotionModel; % Automatically defaults to differential drive

% Configure Particle Filter
amcl.GlobalLocalization = false;
amcl.InitialPose = map_start_pose;
amcl.InitialCovariance = diag([0.05, 0.05, 0.05]); % Low initial uncertainty
amcl.UpdateThresholds = [0.05, 0.05, 0.05]; % [x, y, yaw] minimum motion to trigger update
amcl.ResamplingInterval = 1;

% Tracking variables
pose = map_start_pose(:);
particles = [];

%% PRM Planning
fprintf('[PLAN] Planning initial path to Area B...\n');
prm_out = navigatePRM(map_input_file, start_xy, goal_B, prm_cfg);
path = prm_out.path;
num_waypoints = size(path, 1);
waypoint_idx = 1;

if num_waypoints < 2
    error('PRM path has too few waypoints.');
end

%% Initialize Visualization
plotter = [];
if enable_visualization
    plotter = plotTurtlebot('TurtleBot AMCL Navigation', [-0.5, 1.5], [-0.5, 1.5]);
    plotter = plotter.updatePositionDesired(path(1, :));
end

%% MAIN CONTROL LOOP (STATE MACHINE)
state = 'NAV_TO_B';
controller_state = [];  
avoid_state = [];       

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

        if enable_visualization
            if isempty(plotter.fig) || ~isvalid(plotter.fig)
                fprintf('\n[STOP] Figure closed by user. Stopping robot.\n');
                break;
            end
        end
        
        % ----------------------------------------------------
        % 1. LOCALIZATION (AMCL Update)
        % ----------------------------------------------------
        odom_pose = g_pose(:)'; % Raw odometry from ROS [x, y, theta]
        scan_cache = [];
        
        if g_scan_received && ~isempty(g_scan)
            scan_obj = rosReadLidarScan(g_scan);
            
            % Cache for obstacle avoidance to prevent double-parsing
            scan_cache.ranges = double(scan_obj.Ranges(:));
            scan_cache.angles = double(scan_obj.Angles(:));
            
            % Update AMCL with odometry diff + LiDAR (passing ranges and angles separately)
            [isUpdated, estimatedPose, ~] = amcl(odom_pose, scan_cache.ranges, scan_cache.angles);
            
            if isUpdated
                pose = estimatedPose(:); % Update global pose safely
                if enable_visualization
                    [particles, ~] = getParticles(amcl);
                end
            end
        end

        v_cmd = 0; w_cmd = 0;
        dist_to_wp = 0; 

        % ----------------------------------------------------
        % 2. WAYPOINT TRACKING
        % ----------------------------------------------------
        switch state
            case 'NAV_TO_B'
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

                    if waypoint_idx > num_waypoints
                        fprintf('[STATE] Area B reached. Switching to SEARCH_CIRCLE.\n');
                        state = 'SEARCH_DRIVE';
                    else
                        target_wp = path(waypoint_idx, :);
                        dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                        fprintf('[STATE] Waypoint reached! Targeting Waypoint %d/%d.\n', waypoint_idx, num_waypoints);
                    end
                end

                if strcmp(state, 'NAV_TO_B')
                    [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
                    [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false, scan_cache);
                end

            case 'SEARCH_DRIVE'
                target_wp = search_waypoints(current_search_idx, :);
                dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                
                if g_image_received && ~isempty(g_image)
                    image_rgb = flip(g_image, 1);
                    [centersO, radiiO, centersB, radiiB] = detectCircles(image_rgb);
                    
                    if ~isempty(centersO) && ~found_orange
                        fprintf('[STATE] Orange target detected! Switching to ALIGN_AND_PICTURE.\n');
                        target_color_focus = 'Orange';
                        state = 'ALIGN_AND_PICTURE';
                    elseif ~isempty(centersB) && ~found_blue
                        fprintf('[STATE] Blue target detected! Switching to ALIGN_AND_PICTURE.\n');
                        target_color_focus = 'Blue';
                        state = 'ALIGN_AND_PICTURE';
                    end
                end

                if strcmp(state, 'SEARCH_DRIVE')
                    if dist_to_wp <= waypoint_tolerance
                        fprintf('[STATE] Reached searchpoint. %d. Starts to rotate 360 degrees.\n', current_search_idx);
                        state = 'SEARCH_SPIN';
                        spin_progress = 0; 
                        controller_state = [];
                    else
                        [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
                        [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false, scan_cache);
                    end
                end
                
            case 'SEARCH_SPIN'
                v_cmd = 0.0; w_cmd = 0.4; % Slow rotation
                
                if g_image_received && ~isempty(g_image)
                    image_rgb = flip(g_image, 1);
                    [centersO, ~, centersB, ~] = detectCircles(image_rgb);

                    if ~isempty(centersO) && ~found_orange
                        fprintf('[STATE] Orange target detected! Switching to ALIGN_AND_PICTURE.\n');
                        target_color_focus = 'Orange';
                        state = 'ALIGN_AND_PICTURE';
                    elseif ~isempty(centersB) && ~found_blue
                        fprintf('[STATE] Blue target detected! Switching to ALIGN_AND_PICTURE.\n');
                        target_color_focus = 'Blue';
                        state = 'ALIGN_AND_PICTURE';
                    end
                end

                if strcmp(state, 'SEARCH_SPIN')
                    spin_progress = spin_progress + (abs(w_cmd) * dt); 
                    
                    if spin_progress >= (2 * pi)
                        current_search_idx = current_search_idx + 1;
                        
                        if current_search_idx > size(search_waypoints, 1)
                            fprintf('[STATE] Every searchpoint is scanned, but found no circles. Starting over.\n');
                            current_search_idx = 1;
                        else
                            fprintf('[STATE] Scanning finished. Driving to next searchpoint.\n');
                        end
                        state = 'SEARCH_DRIVE';
                    end
                end

            case 'ALIGN_AND_PICTURE'
                if g_image_received && ~isempty(g_image)
                    image_rgb = flip(g_image, 1);
                    [centersO, radiiO, centersB, radiiB] = detectCircles(image_rgb);
                    img_width = size(image_rgb, 2); 
                    
                    active_dist = []; active_center_x = [];

                    if strcmp(target_color_focus, 'Orange') && ~isempty(centersO)
                        active_dist = f * H / (2 * radiiO(1));
                        active_center_x = centersO(1, 1);
                    elseif strcmp(target_color_focus, 'Blue') && ~isempty(centersB)
                        active_dist = f * H / (2 * radiiB(1));
                        active_center_x = centersB(1, 1);
                    end
                    
                    if ~isempty(active_dist)
                        err_dist = active_dist - target_distance; 
                        err_angle = (img_width / 2) - active_center_x; 
                        
                        v_cmd = Kp_dist * err_dist;
                        w_cmd = Kp_angle * err_angle;
                        
                        v_cmd = max(min(v_cmd, 0.15), -0.15); 
                        w_cmd = max(min(w_cmd, 0.5), -0.5); 
                        
                        if abs(err_dist) < 0.05 && abs(err_angle) < 20
                            fprintf('[STATE] In position. Taking picture...\n');
                            vel_msg.linear.x = 0; vel_msg.angular.z = 0; send(vel_pub, vel_msg);

                            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                            if ~exist('circle_images', 'dir'), mkdir('circle_images'); end
                            img_filename = fullfile('circle_images', sprintf('%s_Circle_%s.png', target_color_focus, timestamp));
                            imwrite(image_rgb, img_filename);
                            fprintf('[INFO] Picture saved as: %s\n', img_filename);
                            pause(1.5);

                            if strcmp(target_color_focus, 'Orange')
                                found_orange = true;
                            elseif strcmp(target_color_focus, 'Blue')
                                found_blue = true;
                            end

                            if found_orange && found_blue
                                fprintf('[PLAN] Both cirles found! Re-planning for Area C...\n');
                                prm_out = navigatePRM(map_input_file, pose(1:2)', goal_C, prm_cfg);
                                path = prm_out.path;
                                num_waypoints = size(path, 1);
                                waypoint_idx = 1;
                                if enable_visualization; plotter = plotter.updatePositionDesired(path(1,:)); end
                                state = 'NAV_TO_C';
                            else
                                fprintf('[STATE] Still missing one circles. Returning to search...\n');
                                state = 'SEARCH_DRIVE';
                            end
                        end
                    else
                        state = 'SEARCH_SPIN'; 
                    end
                end

            case 'NAV_TO_C'
                target_wp = path(waypoint_idx, :);
                dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                
                active_tol = waypoint_tolerance;

                if enable_visualization; plotter = plotter.updatePositionDesired(target_wp); end

                if waypoint_idx == num_waypoints; active_tol = final_tolerance; end
                
                if dist_to_wp <= active_tol
                    waypoint_idx = waypoint_idx + 1;
                    controller_state = [];

                    if waypoint_idx > num_waypoints
                        fprintf('[STATE] Mission Complete! Arrived at Area C.\n');
                        state = 'DONE';
                    else
                        target_wp = path(waypoint_idx, :);
                        dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                        fprintf('[STATE] Waypoint reached! Targeting Waypoint %d/%d.\n', waypoint_idx, num_waypoints);
                    end
                end
                
                if strcmp(state, 'NAV_TO_C')
                    [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
                    [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false, scan_cache);
                end

            case 'DONE'
                v_cmd = 0; w_cmd = 0;
                break;
        end

        % Publish Velocity
        vel_msg.linear.x = v_cmd;      
        vel_msg.angular.z = w_cmd;     
        send(vel_pub, vel_msg);
        
        % ----------------------------------------------------
        % 3. VISUALIZATION & THROTTLING
        % ----------------------------------------------------
        if enable_visualization && mod(loop_count, viz_update_stride) == 0
            plotter = plotter.updatePose(pose(1:2)', pose(3));

            if plot_scan_visualization && g_scan_received && ~isempty(g_scan) && ~isempty(scan_cache)
                % Read efficiently from our cache instead of re-parsing the msg
                cart_raw = pol2cart_local(scan_cache.angles, scan_cache.ranges);
                R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
                cart_world = cart_raw * R' + pose(1:2)';
                plotter = plotter.updateScan(cart_world);
            end
            
            % Draw AMCL particle cloud
            if ~isempty(particles)
                plotter = plotter.updateParticles(particles(:, 1:2));
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
    
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    send(vel_pub, vel_msg);
    
catch ME
    fprintf('\n[ERROR] %s\n', ME.message);
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    send(vel_pub, vel_msg);
    rethrow(ME);
end

if toc(mission_start) >= max_mission_time
    fprintf('\n[TIMEOUT] Mission exceeded %.1f seconds\n', max_mission_time);
end
clear vel_pub odom_sub scan_sub image_sub node amcl;

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

%% ========== UTILITY FUNCTIONS ==========
function [centersO, radiiO, centersB, radiiB] = detectCircles(rgb)
    hsv = rgb2hsv(imgaussfilt(rgb, 1));
    maskO = (hsv(:,:,1) >= 0.05 & hsv(:,:,1) <= 0.15) & hsv(:,:,2) >= 0.5 & hsv(:,:,3) >= 0.2;
    maskB = (hsv(:,:,1) >= 0.55 & hsv(:,:,1) <= 0.65) & hsv(:,:,2) >= 0.5 & hsv(:,:,3) >= 0.2;
    se = strel('disk', 5);
    [centersO, radiiO] = imfindcircles(imclose(imopen(maskO, se), se), [20 200], 'ObjectPolarity', 'bright');
    [centersB, radiiB] = imfindcircles(imclose(imopen(maskB, se), se), [20 200], 'ObjectPolarity', 'bright');
end

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

function cart = pol2cart_local(theta, rho)
    % Helper function to convert Lidar ranges to cartesian purely using math
    [x, y] = pol2cart(theta, rho);
    cart = [x, y];
end