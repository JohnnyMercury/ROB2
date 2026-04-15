%% Dead Reckoning: IMU-based position estimation (ROS2)
% Exercise 1:
% 1. Get acceleration and orientation from /imu topic
% 2. Transform acceleration from body frame to world frame using quatrotate()
% 3. Integrate twice to get displacement
% 4. Compare with odometry from /tf topic

clear all
clc
close all

%% Global variables for callbacks
global imuMsg tfMsg
imuMsg = [];
tfMsg = [];

%% ROS2 setup
setenv('ROS_DOMAIN_ID', '30');
node = ros2node('/base_station_dead_reckoning');

%% Subscribers
imuSub = ros2subscriber(node, '/imu', @imuCallback);
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');

% Wait for first messages
disp('Waiting for IMU and TF messages...');
while isempty(imuMsg) || isempty(tfMsg)
    pause(0.1);
end
disp('Messages received. Starting dead reckoning...');

%% Dead reckoning state
position = [0, 0, 0]; % Estimated position in world frame [x y z]
velocity = [0, 0, 0]; % Estimated velocity in world frame
gravity  = [0, 0, 9.81]; % Gravity vector in world frame (Z up)

%% TF reference (for comparison)
tfPosInit = [];
tfPosRel = [0, 0, 0];

%% Logging for plotting
drHistory = [];
tfHistory = [];
timeHistory = [];

%% Figure for trajectory comparison
figure('Name', 'Dead Reckoning vs TF Odometry', 'NumberTitle', 'off');
hold on; grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory: Dead Reckoning (blue) vs TF Odometry (red)');
hDR = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
hTF = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
legend('Dead Reckoning (IMU)', 'TF Odometry', 'Location', 'best');

%% Main loop
tStart = tic;
tPrev = 0;
lastImuTime = -1;
runDuration = 60; % seconds

try
    while toc(tStart) < runDuration
        % Use IMU timestamp to avoid integrating same sample twice
        imuTime = double(imuMsg.header.stamp.sec) + double(imuMsg.header.stamp.nanosec) * 1e-9;
        if imuTime == lastImuTime
            pause(0.005);
            continue;
        end
        lastImuTime = imuTime;

        tNow = toc(tStart);
        dt = max(tNow - tPrev, 1e-3);
        tPrev = tNow;

        %% Step 1: Get acceleration and orientation from IMU
        aBody = [imuMsg.linear_acceleration.x, ...
                 imuMsg.linear_acceleration.y, ...
                 imuMsg.linear_acceleration.z];

        % Quaternion from ROS [x y z w] -> MATLAB [w x y z]
        qRos = [imuMsg.orientation.x, imuMsg.orientation.y, ...
                imuMsg.orientation.z, imuMsg.orientation.w];
        qMatlab = [qRos(4), qRos(1), qRos(2), qRos(3)];

        %% Step 2: Transform acceleration from body to world frame
        aWorld = quatrotate(qMatlab, aBody);

        % Remove gravity component (IMU reads gravity when stationary)
        aWorld = aWorld - gravity;

        %% Step 3: Double integration (acceleration -> velocity -> position)
        % p[k+1] = p[k] + v[k]*dt + 0.5*a[k]*dt^2
        % v[k+1] = v[k] + a[k]*dt
        position = position + velocity * dt + 0.5 * aWorld * dt^2;
        velocity = velocity + aWorld * dt;

        %% Step 5: Get TF odometry for comparison
        tfPose = extractTfPose(tfMsg);
        if ~isempty(tfPose)
            if isempty(tfPosInit)
                tfPosInit = tfPose;
            end
            tfPosRel = tfPose - tfPosInit;
        end

        %% Log data
        drHistory(end+1, :) = position(1:2);
        tfHistory(end+1, :) = tfPosRel(1:2);
        timeHistory(end+1) = tNow;

        %% Update plot (throttled)
        if mod(size(drHistory,1), 10) == 0
            set(hDR, 'XData', drHistory(:,1), 'YData', drHistory(:,2));
            set(hTF, 'XData', tfHistory(:,1), 'YData', tfHistory(:,2));
            drawnow limitrate;
        end

        pause(0.01);
    end

catch ME
    clear imuSub tfSub
    rethrow(ME);
end

% Cleanup
clear imuSub tfSub

%% Final comparison plot: position error over time
if ~isempty(drHistory) && ~isempty(tfHistory)
    err = vecnorm(drHistory - tfHistory, 2, 2);
    figure('Name', 'Position Error', 'NumberTitle', 'off');
    plot(timeHistory, err, 'k-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('||p_{DR} - p_{TF}|| [m]');
    title('Dead Reckoning drift vs TF odometry');
end

disp('Dead reckoning completed.');

%% Callback functions
function imuCallback(message)
    global imuMsg
    imuMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end

%% Helper: extract [x y yaw] from /tf transform (odom -> base_link)
function pose = extractTfPose(tfMessage)
    pose = [];
    if isempty(tfMessage)
        return;
    end

    try
        transforms = tfMessage.transforms;
    catch
        return;
    end

    for i = 1:numel(transforms)
        t = transforms(i);
        parent = string(t.header.frame_id);
        child = string(t.child_frame_id);

        if contains(parent, 'odom') && ...
           (contains(child, 'base_link') || contains(child, 'base_footprint'))
            x = t.transform.translation.x;
            y = t.transform.translation.y;
            qx = t.transform.rotation.x;
            qy = t.transform.rotation.y;
            qz = t.transform.rotation.z;
            qw = t.transform.rotation.w;
            yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
            pose = [x, y, yaw];
            return;
        end
    end
end
