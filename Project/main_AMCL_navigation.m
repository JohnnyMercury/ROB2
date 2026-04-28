clear;
close all;
clc;

%% ROS2 TURTLEBOT NAVIGATION (PRM + AMCL ORCHESTRATOR)
% Loads a built SLAM map, localizes using AMCL (Particle Filter), 
% plans PRM path, and tracks waypoints while continuously correcting drift.

%% User Mission Parameters
map_input_file = 'slam_map_test_20260428_150643.mat';                      % '' = latest slam_map_*.mat in Project/Maps
goal_input_xy = [2.0, 1.5];               % Goal input [x y]

% With AMCL, you usually want goals to be absolute map coordinates.
goal_is_relative_to_start = false;        % false: goal_input_xy is the exact map coordinate

% AMCL Initialization Guess
% Place the robot within ~2 meters of this guess, orientation doesn't matter.
map_start_pose = [0, 0, 0.0];             % Guess [x y yaw] in map frame
amcl_search_radius = 2.0;                 % meters of uncertainty
amcl_search_yaw = pi;                     % radians of uncertainty (pi = 180 deg)

waypoint_tolerance = 0.15;        % Intermediate waypoint tolerance (m)
final_tolerance = 0.08;           % Final waypoint tolerance (m)

max_mission_time = 180;           % Mission timeout (s)
update_rate_hz = 10;              % Control loop frequency (Hz)
dt = 1 / update_rate_hz;

% Visualization
enable_visualization = true;
plot_scan_visualization = true;   % Changed to true to show actual LiDAR points (blue)
viz_update_stride = 2;            % update plots every N loops

% PRM Planner Settings
prm_cfg = struct();
prm_cfg.numNodes = 350;
prm_cfg.connectionDistance = 0.55;
prm_cfg.retryNumNodes = 700;
prm_cfg.retryConnectionDistance = 0.75;
prm_cfg.inflateRadius = 0.12;     % Slightly inflated for safer paths
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

cmd_topic = "/cmd_vel";
node = ros2node("turtlebot_navigator");
odom_sub = ros2subscriber(node, "/odom", @odomCallback);
scan_sub = ros2subscriber(node, "/scan", @scanCallback, 'Reliability', 'besteffort');
vel_pub = ros2publisher(node, cmd_topic, "geometry_msgs/Twist");
vel_msg = ros2message('geometry_msgs/Twist');

fprintf('[INIT] Waiting for topic discovery (5 seconds)...\n');
pause(5);

wait_odom_timeout = 15;
t_wait = tic;
while ~g_pose_received && toc(t_wait) < wait_odom_timeout
    pause(0.05);
end
if ~g_pose_received
    error('[INIT] No odometry on /odom after %.1f s.', wait_odom_timeout);
end

%% Load Map for AMCL Localization
maps_dir = fullfile(fileparts(mfilename('fullpath')), 'Maps');
if isempty(map_input_file)
    files = dir(fullfile(maps_dir, 'slam_map_*.mat'));
    if isempty(files), error('No maps found.'); end
    [~, idx] = max([files.datenum]);
    map_file_path = fullfile(files(idx).folder, files(idx).name);
else
    map_file_path = fullfile(maps_dir, map_input_file);
    if ~isfile(map_file_path), map_file_path = map_input_file; end
end

fprintf('[INIT] Loading map: %s\n', map_file_path);
map_data = load(map_file_path);

% Use probabilistic map for best AMCL results, fallback to binary
if isfield(map_data.output, 'mapCleanProb') && ~isempty(map_data.output.mapCleanProb)
    amcl_map = occupancyMap(map_data.output.mapCleanProb);
else
    occ = occupancyMatrix(map_data.output.rawOccMap);
    amcl_map = occupancyMap(occ, map_data.output.rawOccMap.Resolution);
end

%% Configure AMCL
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

% Setup Sensor Model (Robust cross-version method)
sm = likelihoodFieldSensorModel();
sm.Map = amcl_map;
sm.SensorLimits = [0.1 3.5];
amcl.SensorModel = sm;

% Setup Motion Model (Robust cross-version method)
% TurtleBot diff-drive noise: [rot1 rot2 trans1 trans2]
mm = odometryMotionModel();
mm.Noise = [0.05 0.05 0.05 0.05]; 
amcl.MotionModel = mm;

% Trigger update every [distance(m), angle(rad), time(s)]
amcl.UpdateThresholds = [0.05, 0.05, 0.2]; 
amcl.ParticleLimits = [100 1500];

amcl.GlobalLocalization = false;
amcl.InitialPose = map_start_pose;
amcl.InitialCovariance = diag([amcl_search_radius, amcl_search_radius, amcl_search_yaw].^2);

%% Spin to Localize (AMCL Convergence)
fprintf('\n[LOC] AMCL Initialization started.\n');
fprintf('[LOC] Robot will drive in a circle to scan surroundings and locate itself.\n');

if enable_visualization
    plotter = plotTurtlebot('TurtleBot AMCL Navigation', [-2, 3], [-2, 3]);
    
    % The magenta dots are the AMCL particles (robot pose guesses), NOT the LiDAR data!
    h_particles = scatter(plotter.h_ax, 0, 0, 8, 'magenta', 'filled', 'MarkerFaceAlpha', 0.5);
end

localized = false;
last_odom = g_pose(:)';
estPose = map_start_pose;
t_loc = tic;

while ~localized && toc(t_loc) < 25
    % Command a circular path (forward + turning)
    vel_msg.linear.x = 0.15;
    vel_msg.angular.z = 0.4;
    send(vel_pub, vel_msg);
    
    if g_scan_received && ~isempty(g_scan)
        scan_obj = rosReadLidarScan(g_scan);
        curr_odom = g_pose(:)';
        odom_delta = getOdomDelta(last_odom, curr_odom);
        last_odom = curr_odom;
        
        [isUpdated, pose, cov] = amcl(odom_delta, scan_obj);
        if isUpdated
            estPose = pose;
            [parts, ~] = amcl.getParticles();
            
            if enable_visualization
                set(h_particles, 'XData', parts(:,1), 'YData', parts(:,2));
                plotter = plotter.updatePose(estPose(1:2)', estPose(3));
                
                % Show actual LiDAR points (blue) during initialization spin too!
                if plot_scan_visualization
                    cart = rosReadCartesian(scan_obj);
                    if ~isempty(cart)
                        R = [cos(estPose(3)), -sin(estPose(3)); sin(estPose(3)), cos(estPose(3))];
                        cart_world = cart * R' + estPose(1:2)';
                        plotter = plotter.updateScan(cart_world);
                    end
                end
                
                drawnow limitrate;
            end
            
            % Check if particles have clustered (converged)
            pos_std = sqrt(max(eig(cov(1:2, 1:2))));
            yaw_std = sqrt(cov(3,3));
            
            if pos_std < 0.15 && yaw_std < 0.15
                localized = true;
                fprintf('[LOC] AMCL Converged! (pos_std=%.2fm, yaw_std=%.2frad)\n', pos_std, yaw_std);
            end
        end
    end
    pause(0.05);
end

% Stop moving
vel_msg.linear.x = 0;
vel_msg.angular.z = 0;
send(vel_pub, vel_msg);
pause(0.5);

if ~localized
    fprintf('[LOC] Warning: AMCL did not fully converge within timeout. Proceeding with best guess.\n');
end
fprintf('[LOC] Current Map Pose: [%.3f, %.3f, %.3f]\n\n', estPose(1), estPose(2), estPose(3));

start_xy = estPose(1:2);
if goal_is_relative_to_start
    goal_xy = start_xy + goal_input_xy(:)';
else
    goal_xy = goal_input_xy(:)';
end

%% PRM Planning
fprintf('[PLAN] Planning path from [%.2f, %.2f] to [%.2f, %.2f]\n', start_xy(1), start_xy(2), goal_xy(1), goal_xy(2));
prm_out = navigatePRM(map_file_path, start_xy, goal_xy, prm_cfg);
path = prm_out.path;
num_waypoints = size(path, 1);

if enable_visualization
    plotter = plotter.updatePositionDesired(path(1, :));
end

%% Controller State
controller_state = [];
avoid_state = [];

%% Main Control Loop
mission_start = tic;
t_prev_loop = 0;
loop_count = 0;
waypoint_idx = 1;
goal_reached = false;

try
    while toc(mission_start) < max_mission_time
        loop_count = loop_count + 1;
        t_elapsed = toc(mission_start);
        
        dt = t_elapsed - t_prev_loop;
        if dt <= 0; dt = 0.001; end
        t_prev_loop = t_elapsed;
        
        if enable_visualization && (~isvalid(plotter.fig) || isempty(plotter.h_ax))
            fprintf('\n[STOP] Figure closed by user.\n');
            break;
        end

        % Detect stale odometry
        if ((now * 86400) - g_last_odom_time) > 1.0
            vel_msg.linear.x = 0; vel_msg.angular.z = 0; send(vel_pub, vel_msg);
            error('Odometry stream is stale (>1 s).');
        end
        
        % -------------------------------------------------------------
        % LOCALIZATION UPDATE (Drift Correction)
        % -------------------------------------------------------------
        curr_odom = g_pose(:)';
        odom_delta = getOdomDelta(last_odom, curr_odom);
        last_odom = curr_odom;
        
        if g_scan_received && ~isempty(g_scan)
            scan_obj = rosReadLidarScan(g_scan);
            [isUpdated, amclPose, ~] = amcl(odom_delta, scan_obj);
            if isUpdated
                estPose = amclPose;
                if enable_visualization && mod(loop_count, viz_update_stride) == 0
                    [parts, ~] = amcl.getParticles();
                    set(h_particles, 'XData', parts(:,1), 'YData', parts(:,2));
                end
            else
                estPose = integrateOdom(estPose, odom_delta);
            end
        else
            estPose = integrateOdom(estPose, odom_delta);
        end
        
        pose = estPose; % This is our new, drift-free robot pose
        
        % -------------------------------------------------------------
        % WAYPOINT TRACKING
        % -------------------------------------------------------------
        target_wp = path(waypoint_idx, :);
        dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
        
        active_tol = waypoint_tolerance;
        if waypoint_idx == num_waypoints, active_tol = final_tolerance; end

        if dist_to_wp <= active_tol
            waypoint_idx = waypoint_idx + 1;
            controller_state = []; % Reset PID integrals
            
            if waypoint_idx > num_waypoints
                goal_reached = true;
                break;
            end
            
            target_wp = path(waypoint_idx, :);
            dist_to_wp = hypot(target_wp(1) - pose(1), target_wp(2) - pose(2));
            if enable_visualization
                plotter = plotter.updatePositionDesired(target_wp);
            end
        end

        % Get PID and APF commands
        [v_cmd, w_cmd, controller_state] = navigatePID(pose, target_wp(:), controller_state, dt);
        [v_cmd, w_cmd, avoid_state, avoid_dbg] = obstacleAvoidance(v_cmd, w_cmd, g_scan, avoid_state, false);

        vel_msg.linear.x = v_cmd;
        vel_msg.angular.z = w_cmd;
        send(vel_pub, vel_msg);
        
        % Visualization
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
        
        if mod(loop_count, 10) == 0
            fprintf('wp=%d/%d | pos=[%5.2f, %5.2f] | wp_dist=%5.2f m | v=%5.2f, w=%5.2f | avoid=%d\n', ...
                waypoint_idx, num_waypoints, pose(1), pose(2), dist_to_wp, v_cmd, w_cmd, avoid_dbg.avoid_mode);
        end

        loop_elapsed = toc(mission_start) - t_elapsed;
        pause(max(0, (1 / update_rate_hz) - loop_elapsed));
    end
    
    % Stop robot safely
    vel_msg.linear.x = 0; vel_msg.angular.z = 0; send(vel_pub, vel_msg);
    
    if goal_reached
        fprintf('\n[SUCCESS] Goal reached at t=%.2f s. Final Pose: [%.3f, %.3f]\n', toc(mission_start), pose(1), pose(2));
    end
    
catch ME
    vel_msg.linear.x = 0; vel_msg.angular.z = 0; send(vel_pub, vel_msg);
    rethrow(ME);
end

clear vel_pub odom_sub scan_sub node;

%% ========== AMCL ODOMETRY HELPERS ==========

function delta = getOdomDelta(prev, curr)
    % Calculates the local robot movement [dx, dy, dtheta] between two odometry states
    dx = curr(1) - prev(1);
    dy = curr(2) - prev(2);
    dth = wrapToPiLocal(curr(3) - prev(3));
    
    % Rotate world delta into the robot's previous local frame
    c = cos(-prev(3)); 
    s = sin(-prev(3));
    loc = [c, -s; s, c] * [dx; dy];
    delta = [loc(1), loc(2), dth];
end

function newPose = integrateOdom(prev, delta)
    % Adds local delta to global map pose (used when AMCL skips an update)
    c = cos(prev(3)); 
    s = sin(prev(3));
    g_xy = [c, -s; s, c] * [delta(1); delta(2)];
    newPose = [prev(1) + g_xy(1), prev(2) + g_xy(2), wrapToPiLocal(prev(3) + delta(3))];
end

%% ========== ROS CALLBACKS ==========
function odomCallback(msg)
    global g_pose g_pose_received g_last_odom_time
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    quat = msg.pose.pose.orientation;
    
    % Manual Quat to Yaw
    w = quat.w; qx = quat.x; qy = quat.y; qz = quat.z;
    siny_cosp = 2 * (w * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy^2 + qz^2);
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

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2 * pi) - pi;
end