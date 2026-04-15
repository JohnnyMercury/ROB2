%% Lidar SLAM: 2D map building from lidar scans (ROS2)
% Exercise 2:
% 1. Create lidarSLAM object
% 2. Subscribe to /scan topic
% 3. Build lidarScan objects from messages
% 4. Add scans to SLAM map using addScan()
% 5. Extract robot poses from PoseGraph
% 6. Continuously add scans as robot moves
% 7. Compare SLAM trajectory with TF odometry
% 8. [Optional] Generate occupancy map
% 9. [Optional] Extract optimized poses from PoseGraph
%
% To control the robot, run in a separate SSH terminal:
%   ros2 run turtlebot3_teleop teleop_keyboard

clear all
clc
close all

%% Global variables for callbacks
global scanMsg tfMsg
scanMsg = [];
tfMsg = [];

%% ROS2 setup
setenv('ROS_DOMAIN_ID', '30');
node = ros2node('/base_station_lidar_slam');

%% Subscribers
scanSub = ros2subscriber(node, '/scan', @scanCallback, 'Reliability', 'besteffort');
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');

disp('Waiting for /scan and /tf messages...');
while isempty(scanMsg) || isempty(tfMsg)
    pause(0.1);
end
disp('Messages received. Starting Lidar SLAM...');

%% Step 1: Create lidarSLAM object
% Parameters:
%   maxLidarRange - maximum lidar range in meters (LDS-01: 3.5, LDS-02: 8)
%   mapResolution - cells per meter (higher = finer map)
maxLidarRange = 3.5;
mapResolution = 20;

slamObj = lidarSLAM(mapResolution, maxLidarRange);
slamObj.LoopClosureThreshold = 150;      % Lower = detect more loops
slamObj.LoopClosureSearchRadius = 4;     % Smaller room = smaller radius
slamObj.MovementThreshold = [0.1 0.1];   % Add scan every 10cm / 0.1rad
slamObj.OptimizationInterval = 2;        % Optimize every 2 loop closures

%% Previous TF pose for relative pose computation
prevTfPose = [];

%% TF reference (for comparison)
tfPosInit = [];
tfHistory = [];
timeHistory = [];

%% Figures
% Figure 1: live SLAM map and trajectory
figSLAM = figure('Name', 'Lidar SLAM', 'NumberTitle', 'off');

% Figure 2: trajectory comparison (SLAM vs TF odometry)
figCompare = figure('Name', 'SLAM vs TF Odometry', 'NumberTitle', 'off');
hold on; grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory: SLAM (blue) vs TF Odometry (red)');
hSLAM = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
hTF = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
legend('Lidar SLAM', 'TF Odometry', 'Location', 'best');

%% Main loop
tStart = tic;
runDuration = 120;  % seconds
lastScanTime = -1;
scanCount = 0;

disp('Building map. Drive the robot around using teleop_keyboard...');

try
    while toc(tStart) < runDuration
        % Process only new scan messages
        scanTime = double(scanMsg.header.stamp.sec) + double(scanMsg.header.stamp.nanosec) * 1e-9;
        if scanTime == lastScanTime
            pause(0.05);
            continue;
        end
        lastScanTime = scanTime;

        %% Step 2-3: Build lidarScan object from ROS scan message
        ranges = double(scanMsg.ranges);
        angleMin = double(scanMsg.angle_min);
        angleIncrement = double(scanMsg.angle_increment);
        angles = angleMin + (0:length(ranges)-1)' * angleIncrement;

        % Filter out invalid ranges (inf, NaN, out-of-range)
        validIdx = isfinite(ranges) & ranges > 0.1 & ranges < maxLidarRange;
        rangesValid = ranges(validIdx);
        anglesValid = angles(validIdx);

        if numel(rangesValid) < 10
            % Skip nearly empty scans
            pause(0.05);
            continue;
        end

        currentScan = lidarScan(rangesValid, anglesValid);

        %% Step 4: Add scan to SLAM with relative pose from odometry
        % Using odometry as initial guess greatly improves scan matching.
        currentTfPose = extractTfPose(tfMsg);
        if ~isempty(currentTfPose) && ~isempty(prevTfPose)
            % Compute relative pose [dx dy dtheta] from previous TF
            dPose = currentTfPose - prevTfPose;
            % Wrap angle to [-pi, pi]
            dPose(3) = atan2(sin(dPose(3)), cos(dPose(3)));
            [isAccepted, loopClosureInfo, optimInfo] = addScan(slamObj, currentScan, dPose);
        else
            [isAccepted, loopClosureInfo, optimInfo] = addScan(slamObj, currentScan);
        end

        if isAccepted && ~isempty(currentTfPose)
            prevTfPose = currentTfPose;
        end

        if isAccepted
            scanCount = scanCount + 1;
        end

        %% Step 5: Get current pose from PoseGraph
        [scans, poses] = scansAndPoses(slamObj);

        %% Step 7: Get TF odometry for comparison
        tfPose = extractTfPose(tfMsg);
        if ~isempty(tfPose)
            if isempty(tfPosInit)
                tfPosInit = tfPose;
            end
            tfRel = tfPose - tfPosInit;
            tfHistory(end+1, :) = tfRel(1:2);
            timeHistory(end+1) = toc(tStart);
        end

        %% Update visualisations (throttled by scan rate)
        if isAccepted && mod(scanCount, 5) == 0
            % SLAM map + trajectory
            figure(figSLAM);
            show(slamObj);
            title(sprintf('Lidar SLAM (scans: %d)', scanCount));

            % Comparison plot
            if ~isempty(poses)
                set(hSLAM, 'XData', poses(:,1), 'YData', poses(:,2));
            end
            if ~isempty(tfHistory)
                set(hTF, 'XData', tfHistory(:,1), 'YData', tfHistory(:,2));
            end
            drawnow limitrate;
        end

        pause(0.02);
    end

catch ME
    clear scanSub tfSub
    rethrow(ME);
end

clear scanSub tfSub

%% Step 8 [Optional]: Generate occupancy map
[scans, poses] = scansAndPoses(slamObj);
if ~isempty(scans)
    occMap = buildMap(scans, poses, mapResolution, maxLidarRange);

    figure('Name', 'Occupancy Map', 'NumberTitle', 'off');
    show(occMap);
    hold on;
    plot(poses(:,1), poses(:,2), 'b-', 'LineWidth', 1.5);
    title('Final Occupancy Map with SLAM trajectory');
end

%% Step 9 [Optional]: Extract optimized poses from pose graph
poseGraph = slamObj.PoseGraph;
if poseGraph.NumNodes > 0
    optimizedPoses = nodeEstimates(poseGraph);
    fprintf('Pose graph: %d nodes, %d edges\n', ...
        poseGraph.NumNodes, poseGraph.NumEdges);

    figure('Name', 'Optimized Poses', 'NumberTitle', 'off');
    plot(optimizedPoses(:,1), optimizedPoses(:,2), 'g.-', 'LineWidth', 1.5);
    hold on; grid on; axis equal;
    xlabel('x [m]'); ylabel('y [m]');
    title('Optimized robot trajectory from pose graph');
end

disp('Lidar SLAM completed.');

%% Callbacks
function scanCallback(message)
    global scanMsg
    scanMsg = message;
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
