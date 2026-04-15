%% Dead Reckoning: IMU-based position estimation (ROS2)
% Exercise 1:
% 1. Get acceleration and orientation from /imu topic
% 2. Transform acceleration from body frame to world frame using quatrotate()
% 3. Integrate twice to get displacement
% 4. Compare with odometry from /tf topic
%
% Noise mitigation:
% - IMU bias calibration (first 2 seconds averaged, then subtracted)
% - Deadband filter (zero out small accelerations)
% - Zero-Velocity Update (ZUPT): reset velocity when robot is stationary

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
disp('Messages received.');

%% Configuration
calibrationDuration = 2.0;   % seconds to estimate bias at start
deadband = 0.15;             % m/s^2, zero small accelerations (noise floor)
zuptThreshold = 0.20;        % m/s^2, below this we assume robot is stationary
zuptCounter = 0;             % consecutive "stationary" samples needed
zuptRequired = 5;            % samples
runDuration = 60;            % seconds total run time

%% Step 1: IMU calibration in BODY frame
% When stationary, IMU reads gravity + bias in body frame.
% Since TurtleBot stays flat (no pitch/roll change), gravity in body frame is constant.
% So: subtract averaged stationary reading BEFORE rotating to world frame.
disp('Calibrating IMU (keep robot still)...');
aBodySum = [0, 0, 0];
calibCount = 0;
tCalibStart = tic;
lastImuTimeCal = -1;

while toc(tCalibStart) < calibrationDuration
    imuTime = double(imuMsg.header.stamp.sec) + double(imuMsg.header.stamp.nanosec) * 1e-9;
    if imuTime == lastImuTimeCal
        pause(0.005);
        continue;
    end
    lastImuTimeCal = imuTime;

    aBody = [imuMsg.linear_acceleration.x, ...
             imuMsg.linear_acceleration.y, ...
             imuMsg.linear_acceleration.z];

    aBodySum = aBodySum + aBody;
    calibCount = calibCount + 1;
end

% Stationary body-frame reading (gravity + bias). Subtract from future readings
% to get true body-frame acceleration (zero when stationary, nonzero when moving).
aBodyStationary = aBodySum / calibCount;
fprintf('Stationary reading (body frame): [%.3f, %.3f, %.3f] m/s^2\n', ...
    aBodyStationary(1), aBodyStationary(2), aBodyStationary(3));

%% Dead reckoning state
position = [0, 0, 0];
velocity = [0, 0, 0];

%% TF reference
tfPosInit = [];
tfPosRel = [0, 0, 0];

%% Logging
drHistory = [];
tfHistory = [];
timeHistory = [];

%% Figure
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

disp('Running dead reckoning. Move the robot now...');

try
    while toc(tStart) < runDuration
        imuTime = double(imuMsg.header.stamp.sec) + double(imuMsg.header.stamp.nanosec) * 1e-9;
        if imuTime == lastImuTime
            pause(0.005);
            continue;
        end
        lastImuTime = imuTime;

        tNow = toc(tStart);
        dt = max(tNow - tPrev, 1e-3);
        tPrev = tNow;

        %% Step 1: Get acceleration and orientation
        aBody = [imuMsg.linear_acceleration.x, ...
                 imuMsg.linear_acceleration.y, ...
                 imuMsg.linear_acceleration.z];

        qRos = [imuMsg.orientation.x, imuMsg.orientation.y, ...
                imuMsg.orientation.z, imuMsg.orientation.w];
        qMatlab = [qRos(4), qRos(1), qRos(2), qRos(3)];

        %% Step 2: Remove bias+gravity in body frame, then transform to world
        aBodyClean = aBody - aBodyStationary;
        aWorld = quatrotate(qMatlab, aBodyClean);

        % Deadband filter: treat small acceleration as zero (noise)
        if norm(aWorld) < deadband
            aWorld = [0, 0, 0];
        end

        %% Step 3: Double integration
        position = position + velocity * dt + 0.5 * aWorld * dt^2;
        velocity = velocity + aWorld * dt;

        %% Zero Velocity Update (ZUPT)
        % If robot appears stationary for several consecutive samples,
        % reset velocity to prevent drift accumulation.
        if norm(aWorld) < zuptThreshold
            zuptCounter = zuptCounter + 1;
            if zuptCounter >= zuptRequired
                velocity = [0, 0, 0];
            end
        else
            zuptCounter = 0;
        end

        %% Step 5: TF odometry for comparison
        tfPose = extractTfPose(tfMsg);
        if ~isempty(tfPose)
            if isempty(tfPosInit)
                tfPosInit = tfPose;
            end
            tfPosRel = tfPose - tfPosInit;
        end

        %% Log
        drHistory(end+1, :) = position(1:2);
        tfHistory(end+1, :) = tfPosRel(1:2);
        timeHistory(end+1) = tNow;

        %% Plot update (throttled)
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

clear imuSub tfSub

%% Final error plot
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

%% Callbacks
function imuCallback(message)
    global imuMsg
    imuMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end

%% Helper: extract [x y yaw] from /tf transform
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
