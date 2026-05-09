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
map_input_file = 'edited_custom_map.mat';  % '' = latest slam_map_*.mat in Project/Maps
map_start_pose = [0, 0, 0.0]; % [x y yaw] in map frame at script start
goal_B = [20.35, 9.24];  % Goal in Area B 
goal_C = [17.31, 17.26];  % Goal in Area C 

% Navigation
enable_amcl = true;               % Enable LiDAR Scan Matching to eliminate Odometry drift
waypoint_tolerance = 0.12;        % Intermediate waypoint tolerance (m)
final_tolerance = 0.08;           % Final waypoint tolerance (m)
max_mission_time = 300;           % Mission timeout (s)
update_rate_hz = 10;              % Control loop frequency (Hz)
dt = 1 / update_rate_hz;

% Visualization 
enable_visualization = true;
plot_scan_visualization = true;   % Turn on to see the scan matching the map
viz_update_stride = 3;            % update plots every N loops

% PRM planner overrides
prm_cfg = struct();
prm_cfg.numNodes = 500;
prm_cfg.connectionDistance = 3.00;
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

found_orange = false; % Variable for orange circle
found_blue = false;  % Variable for blue circle
target_color_focus =''; % Keeps track on which color we are working on
search_waypoints = [18.2, 7.8; 18.4, 9.5; 18.9, 11.4]; % Search points for area B FILL IN!!!!!!!!!!
current_search_idx = 1;
spin_progress = 0;  % Keeps track on spinning

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

% DEBUG: Print all waypoints for verification
fprintf('[PLAN] Planned %d waypoints:\n', num_waypoints);
for i = 1:num_waypoints
    fprintf('  wp_%d: [%.4f, %.4f]\n', i, path(i,1), path(i,2));
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
        mcl.InitialCovariance = diag([0.1, 0.1, 0.05]);  % Relaxed to allow faster convergence if startup pose is incorrect
        
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

% AMCL correction tracking
correctionsLog = {};
last_T_M_O = T_M_O;  % store last transform to detect large corrections
amcl_update_count = 0;


fprintf('[START] Beginning AMCL warmup phase (10 seconds)...\n');

% Warmup Phase: Let AMCL converge while robot is stationary
if use_amcl
    warmup_duration = 10;  % seconds
    warmup_start = tic;
    while toc(warmup_start) < warmup_duration
        if ~g_scan_received || isempty(g_scan)
            pause(0.05);
            continue;
        end
        
        % Read odometry and scan
        odom_pose = g_pose(:)';
        T_O_R = pose2tform2D(odom_pose);
        
        % Force AMCL to update every loop during warmup (use large finite value instead of inf)
        mcl.UpdateThresholds = [999, 999, 999];
        
        try
            scan_obj = rosReadLidarScan(g_scan);
            ranges = double(scan_obj.Ranges(:));
            angles = double(scan_obj.Angles(:));
            valid = isfinite(ranges) & ranges > 0.08 & ranges < 3.48;
            
            if sum(valid) > 20
                scan_clean = lidarScan(ranges(valid), angles(valid));
                [isUpdated, estPose, ~] = mcl(odom_pose, scan_clean);
                if isUpdated
                    T_M_R_mcl = pose2tform2D(estPose);
                    T_M_O = T_M_R_mcl / T_O_R;
                end
            end
        catch
        end
        
        % Small delays to keep loop from spinning too fast
        pause(0.05);
        if mod(round(toc(warmup_start)*10), 10) == 0  % Every 1 second
            fprintf('  Warmup: %.1f / %.1f seconds\n', toc(warmup_start), warmup_duration);
        end
    end
    fprintf('[INIT] AMCL warmup complete. Particles converged.\n');
end

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
        % if (now_sec - g_last_odom_time) > 1.0
        %     vel_msg.linear.x = 0;
        %     vel_msg.angular.z = 0;
        %     send(vel_pub, vel_msg);
        %     error('Odometry stream is stale (>1 s). Commanding stop for safety.');
        % end
        
        % ----------------------------------------------------
        % 1. LOCALIZATION (Drift-Corrected Pose Tracking)
        % ----------------------------------------------------
        % Read live Odometry
        odom_pose = g_pose(:)';
        T_O_R = pose2tform2D(odom_pose);
        
        % Process Scan Matching dynamically to align Map 
        if use_amcl && g_scan_received && ~isempty(g_scan)
            % At startup, always update AMCL to converge quickly; after 30 loops, use normal thresholds
            if loop_count < 30
                mcl.UpdateThresholds = [999, 999, 999];  % Always update during warmup (large finite value)
            else
                mcl.UpdateThresholds = [0.10, 0.10, 0.10];  % Normal decimation after startup
            end
            
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
                        amcl_update_count = amcl_update_count + 1;
                        if mod(amcl_update_count, 10) == 0
                            fprintf('[AMCL] Update #%d at t=%.1f s | estPose=[%.3f, %.3f, %.3f]\n', amcl_update_count, toc(mission_start), estPose(1), estPose(2), estPose(3));
                        end
                        % AMCL provided a correction! Calculate the new Map-to-Odom relationship
                        T_M_R_mcl = pose2tform2D(estPose);
                        T_M_O_new = T_M_R_mcl / T_O_R;

                        % Compute correction relative to last applied transform
                        try
                            tf_delta = T_M_O_new / last_T_M_O;
                            delta_pose = tform2pose2D(tf_delta);
                        catch
                            delta_pose = [0,0,0];
                        end

                        corr_dist = hypot(delta_pose(1), delta_pose(2));
                        corr_ang = delta_pose(3);

                        % Log large corrections for debugging
                        if corr_dist > 0.05 || abs(corr_ang) > 0.1
                            numParticles = 0;
                            try numParticles = size(mcl.Particles,1); catch; end
                            fprintf('[AMCL] Large correction at t=%.1f s | est=[%.3f,%.3f,%.3f] | delta=[%.4f,%.4f,%.4f] | particles=%d\n', ...
                                toc(mission_start), estPose(1), estPose(2), estPose(3), delta_pose(1), delta_pose(2), delta_pose(3), numParticles);
                            correctionsLog{end+1} = struct('time', toc(mission_start), 'estPose', estPose, 'delta', delta_pose, 'particles', numParticles);
                        end

                        % Apply the new transform and remember it
                        T_M_O = T_M_O_new;
                        last_T_M_O = T_M_O;

                        % If correction is large, replan from current estimated pose
                        if corr_dist > 0.20 || abs(corr_ang) > 0.5
                            try
                                pose_now = tform2pose2D(T_M_O * T_O_R);
                                fprintf('[AMCL] Significant correction detected (%.3fm, %.3frad). Replanning from [%.3f, %.3f].\n', corr_dist, corr_ang, pose_now(1), pose_now(2));
                                prm_out = navigatePRM(map_input_file, pose_now(1:2)', goal_B, prm_cfg);
                                path = prm_out.path;
                                num_waypoints = size(path, 1);
                                % choose nearest waypoint index to continue
                                dists = hypot(path(:,1) - pose_now(1), path(:,2) - pose_now(2));
                                [~, idx] = min(dists);
                                waypoint_idx = max(1, idx);
                                if enable_visualization
                                    plotter = plotter.updatePositionDesired(path(waypoint_idx, :));
                                end
                                fprintf('[AMCL] Replan complete: new path has %d waypoints, continuing at wp_%d.\n', num_waypoints, waypoint_idx);
                            catch ME
                                fprintf('[AMCL] Replan failed: %s\n', ME.message);
                            end
                        end
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
                % Task: Drive from area A to area B
                target_wp = path(waypoint_idx, :);
                dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                
                active_tol = waypoint_tolerance;

                if enable_visualization && mod(loop_count, viz_update_stride) == 0
                    plotter = plotter.updatePositionDesired(target_wp);
                end

                if waypoint_idx == num_waypoints; active_tol = final_tolerance; end

                if dist_to_wp <= active_tol
                    % Log waypoint transition for diagnostics
                    fprintf('[NAV] Reached waypoint %d at pose [%.3f, %.3f, %.3f].\n', ...
                        waypoint_idx, pose(1), pose(2), pose(3));

                    % Advance a single waypoint and handle end-of-path
                    waypoint_idx = waypoint_idx + 1;
                    controller_state = [];

                    if waypoint_idx > num_waypoints
                        fprintf('[STATE] Area B reached. Switching to SEARCH_CIRCLE.\n');
                        state = 'SEARCH_DRIVE';
                    else
                        target_wp = path(waypoint_idx, :);
                        dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                        if enable_visualization && mod(loop_count, viz_update_stride) == 0
                            plotter = plotter.updatePositionDesired(target_wp);
                        end
                        fprintf('[NAV] Next target: wp_%d [%.3f, %.3f]\n', waypoint_idx, target_wp(1), target_wp(2));
                    end
                end

                if strcmp(state, 'NAV_TO_B')
                    [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
                    [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);
                end

            case 'SEARCH_DRIVE'
                % Task: Drive to next searchpoint to scan for circles
                target_wp = search_waypoints(current_search_idx, :);
                dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
                
                % Search for circles while driving
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
                        [v_cmd, w_cmd, avoid_state, ~] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);
                    end
                end
                
            case 'SEARCH_SPIN'
                % Task: Rotate to find the targets
                v_cmd = 0.0; w_cmd = 0.4; % Slow rotation
                
                % Search for circles
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
                    spin_progress = spin_progress + (abs(w_cmd) * dt); % Checks how much we have rotated
                    
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
                % Task: Position exactly 1m in front of target and snap photo
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
                        
                        % Velocity constraints for safety
                        v_cmd = max(min(v_cmd, 0.15), -0.15); 
                        w_cmd = max(min(w_cmd, 0.5), -0.5); 
                        
                        % Threshold for taking the picture
                        if abs(err_dist) < 0.05 && abs(err_angle) < 20
                            fprintf('[STATE] In position. Taking picture...\n', target_color_focus);
                            vel_msg.linear.x = 0; vel_msg.angular.z = 0; send(vel_pub, vel_msg);

                            % Save picture 
                            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                            img_filename = fullfile('circle_images', sprintf('%s_Circle_%s.png', target_color_focus, timestamp));
                            imwrite(image_rgb, img_filename);
                            fprintf('[INFO] Picture saved as: %s\n', img_filename);
                            pause(1.5);

                            % Updates boolean variables
                            if strcmp(target_color_focus, 'Orange')
                                found_orange = true;
                            elseif strcmp(target_color_focus, 'Blue')
                                found_blue = true;
                            end

                            % Checks if we're done with both circles 
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
                        % Target lost, sarching againg
                        state = 'SEARCH_SPIN'; % Rotating to see if we can find it 
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