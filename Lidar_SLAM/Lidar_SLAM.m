%% Lidar SLAM: 2D map building from lidar scans (ROS2)
% Exercise 2 - minimal version following MATLAB documentation example.
%
% To control the robot, in a separate SSH terminal on the robot, run:
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

scanSub = ros2subscriber(node, '/scan', @scanCallback, 'Reliability', 'besteffort');
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');

disp('Waiting for /scan...');
while isempty(scanMsg)
    pause(0.1);
end
disp('Scan received. Starting SLAM...');

%% Step 1: Create lidarSLAM object (LDS-01 has 3.5m range)
maxLidarRange = 3.5;
mapResolution = 20;

slamObj = lidarSLAM(mapResolution, maxLidarRange);
slamObj.LoopClosureThreshold = 210;
slamObj.LoopClosureSearchRadius = 8;

%% Live SLAM figure
figSLAM = figure('Name', 'Lidar SLAM (live)', 'NumberTitle', 'off');

%% Main loop
tStart = tic;
runDuration = 180;  % 3 minutes
lastScanTime = -1;
scanCount = 0;

disp('Drive the robot around using teleop_keyboard...');

try
    while toc(tStart) < runDuration
        % Only process new scans
        scanTime = double(scanMsg.header.stamp.sec) + ...
                   double(scanMsg.header.stamp.nanosec) * 1e-9;
        if scanTime == lastScanTime
            pause(0.05);
            continue;
        end
        lastScanTime = scanTime;

        %% Steps 2-3: Build lidarScan from ROS message
        ranges = double(scanMsg.ranges);
        angleMin = double(scanMsg.angle_min);
        angleInc = double(scanMsg.angle_increment);
        angles = angleMin + (0:length(ranges)-1)' * angleInc;

        % Create lidarScan and remove invalid points
        currentScan = lidarScan(ranges, angles);
        currentScan = removeInvalidData(currentScan, ...
            'RangeLimits', [0.1 maxLidarRange]);

        if currentScan.Count < 10
            pause(0.05);
            continue;
        end

        %% Step 4: Add scan (let SLAM do scan matching on its own)
        [isAccepted, ~, ~] = addScan(slamObj, currentScan);

        if isAccepted
            scanCount = scanCount + 1;
            fprintf('Added scan %d\n', scanCount);
        end

        %% Live visualization every 5 accepted scans
        if isAccepted && mod(scanCount, 5) == 0
            figure(figSLAM);
            show(slamObj);
            title(sprintf('Lidar SLAM - %d scans', scanCount));
            drawnow limitrate;
        end

        pause(0.02);
    end

catch ME
    clear scanSub tfSub
    rethrow(ME);
end

clear scanSub tfSub

%% Final visualization
fprintf('Total scans added: %d\n', scanCount);

% Final SLAM plot
figure('Name', 'Final SLAM Map', 'NumberTitle', 'off');
show(slamObj);
title(sprintf('Final SLAM map - %d scans', scanCount));

% Step 8 [Optional]: Build occupancy map
[scans, poses] = scansAndPoses(slamObj);
if ~isempty(scans)
    occMap = buildMap(scans, poses, mapResolution, maxLidarRange);
    figure('Name', 'Occupancy Map', 'NumberTitle', 'off');
    show(occMap);
    hold on;
    plot(poses(:,1), poses(:,2), 'b-', 'LineWidth', 1.5);
    title('Occupancy Map');
end

disp('Done.');

%% Callbacks
function scanCallback(message)
    global scanMsg
    scanMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end
