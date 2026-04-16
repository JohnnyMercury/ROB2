%% Clear workspace, command window, and close all figures
clear all
clc
close all
 
%% Declare global variables for robot pose and laser scan data
global pose scan
 
%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');
 
%% Display available ROS2 topics (for debug)
ros2 topic list
 
%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');
 
% Wait for topics to be available
%pause(0.5);
 
%% Wait for required topics to appear before creating subscribers
maxWaitSec = 120;
pollInterval = 0.5;
requiredTopics = ["/odom", "/scan"];
startTime = tic;

%% Initialize plot display toggles
plot_headings = false;    % Enable/disable heading vs time plot
plot_distance = false;    % Enable/disable distance to goal plot
plot_scan = true;        % Enable/disable LIDAR scan visualization
 
disp('Waiting for required ROS2 topics...');
while toc(startTime) < maxWaitSec
    try
        topicOutput = evalc("ros2 topic list");
        topics = string(split(string(topicOutput)));
        if all(ismember(requiredTopics, topics))
            disp(['Topics found after ' num2str(toc(startTime), '%.1f') ' seconds.']);
            break;
        end
    catch
        % Ignore errors during discovery
    end
    pause(pollInterval);
end
 
if toc(startTime) >= maxWaitSec
    error('Timeout waiting for topics. Check TurtleBot bringup.');
end
 
%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic
 
% Pause to allow ROS subscriptions to initialize
pause(0.5);
   
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
   
    %% Pre-allocate command message (reuse every loop instead of creating new message)
    cmdMsg = ros2message('geometry_msgs/Twist');
   
    %% Create figure for TurtleBot's data (conditionally based on toggles)
    % Create mode calls: each function returns a handles struct used for fast updates.
    if plot_headings
        hHead = plotHeadings();
    else
        hHead = [];  % Empty handle if disabled
    end
    
    if plot_distance
        hDist = plotDistance();
    else
        hDist = [];  % Empty handle if disabled
    end
    
    visualise = TurtleBotVisualise();
 
 
    %% Initialize goal position (fixed target)
    goal_position = [1.0, -0.1]; % [x, y] coordinates of the goal position
    position_desired = goal_position; % show goal in the visualizer
 
    % History buffers for live time-series plotting.
    % These arrays grow each loop and are passed into plot update calls.
    % Cap history to rolling window to prevent unbounded memory growth.
    maxHistoryLength = 1000;  % Keep last ~10 seconds of data
    loopStartTime = tic;  % Record real wall-clock time for accurate timing
    t = 0;
    t_hist = [];
    heading_desired_hist = [];
    heading_actual_hist = [];
    distance_hist = [];
   
    %% Initialize PD controller for heading
    % PD: angularVelocity = Kp_h * error + Kd_h * d(error)/dt
    Kp_h = 4;  % Proportional gain for heading (tune this)
    Kd_h = 2;  % Derivative gain for heading (tune this)
    heading_error_prev = 0;  % Track previous error for derivative
    t_prev = 0;  % Track previous time for accurate dt
   
    %% Initialize PI controller for position/distance
    % PI: linearVelocity = Kp_p * distance + Ki_p * integral(distance)
    Kp_p = 8;  % Proportional gain for position (tune this)
    Ki_p = 0.02;  % Integral gain for position (tune this)
    integral_distance_error = 0;  % Accumulated integral of distance error
    
    %% Track previous heading_desired for continuity (avoid 180° heading flips)
    heading_desired_prev = 0;
 
 
 
    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Wait for scan and pose data to arrive
        if isempty(scan) || ~isstruct(scan) || isempty(pose) || ~isstruct(pose)
            pause(0.001);  % Very brief pause to prevent CPU spinlock
            continue;  % Skip this iteration, wait for callbacks
        end
       
        %% USE REAL ELAPSED TIME (not fake t = t + 0.01)
        t = toc(loopStartTime);
       
        %% Visualise desired position (circle center)
        visualise = updatePositionDesired(visualise, position_desired);
 
        %% Get the robot's current position and heading
        position = [pose.position.x pose.position.y];
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading = orientation(3); % Extract heading (yaw)
        visualise = updatePose(visualise, position, heading);
   
        %% Process and plot laser scan data (if enabled)
        %heading_y = heading+pi/2; % LIDAR is attached properly now, pointing forward
        if plot_scan
            cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
            %cart = cart * [cos(heading_y), -sin(heading_y); sin(heading_y), cos(heading_y)]' + position; % Transform based on robot position and heading
            cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position; % Transform based on robot position and heading
            visualise = updateScan(visualise, cart);
        end
        
        %% Compute goal position and tracking errors
        target_desired = goal_position;  % Use fixed goal position
        x_error = target_desired(1) - position(1);  % Error in x direction
        y_error = target_desired(2) - position(2);  % Error in y direction
        distance_error = sqrt(x_error^2 + y_error^2);  % Euclidean distance to goal
        
        % Calculate raw desired heading
        heading_desired_raw = atan2(y_error, x_error);  % Desired heading angle to face goal
        
        % Apply continuity check: if heading jumped by ~π, use previous heading
        % This prevents the robot from deciding to turn 180° when very close to goal
        heading_error_raw = heading_desired_raw - heading_desired_prev;
        while heading_error_raw > pi
            heading_error_raw = heading_error_raw - 2*pi;
        end
        while heading_error_raw < -pi
            heading_error_raw = heading_error_raw + 2*pi;
        end
        
        % If a large shift detected near goal, keep previous heading to avoid oscillation
        if distance_error < 0.3 && abs(heading_error_raw) > pi/2
            heading_desired = heading_desired_prev;  % Stick with previous heading
        else
            heading_desired = heading_desired_raw;  % Use freshly calculated heading
            heading_desired_prev = heading_desired_raw;  % Update for next iteration
        end

        %% Plot actual and desired heading
        heading_actual  = heading;   % example actual heading (rad)
 
        % Append latest samples to history arrays (time + values).
        t_hist(end+1) = t;
        heading_desired_hist(end+1) = heading_desired;
        heading_actual_hist(end+1) = heading_actual;
       
        % Cap heading history to rolling window (prevent unbounded memory growth).
        if length(t_hist) > maxHistoryLength
            t_hist = t_hist(end-maxHistoryLength+1:end);
            heading_desired_hist = heading_desired_hist(end-maxHistoryLength+1:end);
            heading_actual_hist = heading_actual_hist(end-maxHistoryLength+1:end);
        end
 
        %% Visualise desired heading (if enabled)
        % Update mode call: reuse existing line handles, only refresh X/Y data.
        if plot_headings
            hHead = plotHeadings(hHead, t_hist, heading_desired_hist, heading_actual_hist);
        end
 
        %% Visualise distance to desired position (if enabled)
        % Compute Euclidean distance to goal and append to history.
        distance_to_target = norm(target_desired - position);
        distance_hist(end+1) = distance_to_target;
       
        % Cap distance history to rolling window.
        if length(distance_hist) > maxHistoryLength
            distance_hist = distance_hist(end-maxHistoryLength+1:end);
        end
        % Update mode call for distance-vs-time plot.
        if plot_distance
            hDist = plotDistance(hDist, t_hist, distance_hist);
        end

        %% Error calculation
        heading_error = heading_desired - heading;

        %% Threshold for error (deadzone to prevent oscillation)
        if abs(distance_error) < 0.05
            distance_error = 0;
        end
        
        % Larger heading deadzone when very close to goal to reduce circling
        heading_threshold = 0.1;  % Radians
        if distance_error < 0.2  % When within 20cm of goal
            heading_threshold = 0.3;  % Relax heading requirement (0.3 rad ≈ 17°)
        end
        if abs(heading_error) < heading_threshold
            heading_error = 0;
        end
        
        %% Check if robot has reached the goal (combined distance and heading threshold)
        goal_reached_threshold_distance = 0.1;  % 10cm from goal
        goal_reached_threshold_heading = 0.2;   % 11.5° heading tolerance
        if distance_error < goal_reached_threshold_distance && abs(heading_error) < goal_reached_threshold_heading
            % Robot is at goal - stop all movement
            linearVelocity = 0;
            angularVelocity = 0;
        else
            %% PID controller for heading (only when not at goal)
            % Wrap heading error to [-pi, pi] to avoid discontinuity jumps
            while heading_error > pi
                heading_error = heading_error - 2*pi;
            end
            while heading_error < -pi
                heading_error = heading_error + 2*pi;
            end
           
            % Compute time delta for accurate derivative
            dt = t - t_prev;
            if dt > 0
                heading_error_dot = (heading_error - heading_error_prev) / dt;
            else
                heading_error_dot = 0;
            end
           
            % PD control law: angularVelocity = Kp * error + Kd * d(error)/dt
            angularVelocity = Kp_h * heading_error + Kd_h * heading_error_dot;
           
            % Store state for next iteration
            heading_error_prev = heading_error;
            t_prev = t;
            
            %% PI controller for position/distance
            % Accumulate integral of distance error over time
            integral_distance_error = integral_distance_error + distance_error * dt;
            if integral_distance_error > 1
                integral_distance_error = 0;
            end
           
            % PI control law: linearVelocity = Kp * distance + Ki * integral(distance)
            linearVelocity = Kp_p * distance_error + Ki_p * integral_distance_error;
        end
   
        %% Publish velocity commands (reuse pre-allocated cmdMsg)
        cmdMsg.linear.x = clip(linearVelocity, -0.1, 0.1);
        cmdMsg.angular.z = clip(angularVelocity, -1.0, 1.0);
        send(cmdPub, cmdMsg);
       
        %% Single drawnow for all figures (consolidated from individual plot functions)
        drawnow limitrate;
   
        %% Exit the loop if the figure is closed
        if size(findobj(visualise.fig)) == 0
            ME = MException('NonException:EndProgram', 'The program was closed.');
            throw(ME)
        end
    end
catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.X = 0;
    cmdMsg.Angular.Z = 0;
    send(cmdPub, cmdMsg);
   
    % Clean up ROS subscriptions
    clear odomSub scanSub
 
    % Show the error
    if ~strcmp(ME.identifier, 'NonException:EndProgram')
        rethrow(ME)
    end
end
 
%% Callback functions
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose
 
    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;
end
 
function scanCallback(message)
    % Use global variable to store laser scan data
    global scan
 
    % Save the laser scan message
    scan = message;
end