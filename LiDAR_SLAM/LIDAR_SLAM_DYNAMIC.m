%% L7 Exercise 2: LiDAR SLAM (Steps 1-5 only) - Improved with Dynamic Clearing
% 1) Create lidarSLAM object
% 2) Subscribe to /scan
% 3) Convert LaserScan message to lidarScan
% 4) Add scans with addScan using relative pose between scans from /tf odometry
% 5) Read robot pose from scansAndPoses
% 6) Update live Occupancy Map to clear out dynamic obstacles

clear all
clc
close all

global tfMsg scanMsg
tfMsg = [];
scanMsg = [];

%% User configuration
cfg.rosDomainId = '30';
cfg.maxWaitSec = 120;
cfg.pollInterval = 0.5;
cfg.firstMsgTimeoutSec = 25;
cfg.maxLidarRange = 3.5;  % LDS-02: 8.0, LDS-01: 3.5
cfg.mapResolution = 20;    % cells per meter
cfg.maxNumScans = 3000;
cfg.minValidRange = 0.08;      % reject very near noisy returns
cfg.maxRangeMargin = 0.02;     % drop ranges near max range (no return)
cfg.rangeMedianWindow = 15;     % odd window for simple spike suppression

% SLAM tuning
cfg.loopClosureThreshold = 210;
cfg.loopClosureSearchRadius = 6;

%% ROS2 setup
setenv('ROS_DOMAIN_ID', cfg.rosDomainId);
node = ros2node('/base_station_l7_lidar_slam_steps_1_5');

requiredTopics = ["/scan", "/tf"];
startTime = tic;

disp('Waiting for required ROS2 topics...');
while toc(startTime) < cfg.maxWaitSec
    try
        topicOutput = evalc("ros2 topic list");
        topics = strtrim(splitlines(string(topicOutput)));
        topics = topics(topics ~= "");
        if all(ismember(requiredTopics, topics))
            disp(['Topics found after ' num2str(toc(startTime), '%.1f') ' seconds.']);
            break;
        end
    catch
        % Ignore transient ROS discovery errors.
    end
    pause(cfg.pollInterval);
end

if toc(startTime) >= cfg.maxWaitSec
    error('Timeout waiting for /scan and /tf topics.');
end

%% Subscribers
scanSub = ros2subscriber(node, '/scan', @scanCallback, 'Reliability', 'besteffort');
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');
pause(0.5);

%% Wait for first /scan and /tf
disp('Waiting for first /scan and /tf messages...');
firstMsgStart = tic;
while isempty(scanMsg) || isempty(tfMsg)
    if toc(firstMsgStart) > cfg.firstMsgTimeoutSec
        error('Timeout waiting for first /scan and /tf messages.');
    end
    pause(0.01);
end

%% Step 1: Create lidarSLAM object
slamObj = lidarSLAM(cfg.mapResolution, cfg.maxLidarRange, cfg.maxNumScans);
slamObj.LoopClosureThreshold = cfg.loopClosureThreshold;
slamObj.LoopClosureSearchRadius = cfg.loopClosureSearchRadius;

% --- IMPROVEMENT: Create a live Occupancy Map ---
% This map will handle ray-casting to clear out dynamic/moved obstacles
occMap = occupancyMap(20, 20, cfg.mapResolution);
occMap.GridLocationInWorld = [-10, -10]; % Center the 20x20m map around the starting point (0,0)

%% Runtime state
lastScanStamp = '';
tfInitPose = [];
prevAcceptedTfPose = [];
acceptedCount = 0;

% Visuals
mapFig = figure('Name', 'LiDAR SLAM Map (Live)', 'NumberTitle', 'off');
mapAx = axes('Parent', mapFig);
show(occMap, 'Parent', mapAx);
title(mapAx, 'Live Occupancy Map (Clearing Dynamic Obstacles)');

visualise = TurtleBotVisualise();
title(visualise.h_ax, 'TurtleBot Pose from /tf (heading shown)');

disp('Running LiDAR SLAM (Steps 1-5). Move TurtleBot to collect scans...');

try
    while true
        if ~isgraphics(mapFig) || ~isgraphics(visualise.fig)
            error('L7SLAM:UserClosedFigure', 'A visualization figure was closed by user.');
        end

        % Process each scan once using message timestamp.
        scanStamp = readScanStamp(scanMsg);
        if strcmp(scanStamp, lastScanStamp) % STRing CoMPare
            tfPoseLive = extractOdomPoseFromTf(tfMsg);
            if ~isempty(tfPoseLive)
                if isempty(tfInitPose)
                    tfInitPose = tfPoseLive;
                end
                tfYawLive = wrapToPiLocal(tfPoseLive(3) - tfInitPose(3));
                tfPoseLiveRel = [tfPoseLive(1) - tfInitPose(1), tfPoseLive(2) - tfInitPose(2), tfYawLive];
                visualise = updatePose(visualise, tfPoseLiveRel(1:2), tfPoseLiveRel(3));
                drawnow limitrate;
            end
            pause(0.001);
            continue;
        end
        lastScanStamp = scanStamp;

        % Step 3: Build lidarScan object from /scan message.
        [scan, nValid] = lidarScanFromRos(scanMsg, cfg.maxLidarRange, cfg.minValidRange, cfg.maxRangeMargin, cfg.rangeMedianWindow);
        if nValid < 10
            pause(0.001);
            continue;
        end

        % Read odometry from /tf and build relative pose increment.
        tfPose = extractOdomPoseFromTf(tfMsg);
        if isempty(tfPose)
            % If no TF at this moment, skip this scan.
            pause(0.001);
            continue;
        end

        if isempty(tfInitPose)
            tfInitPose = tfPose;
        end

        tfYawRel = wrapToPiLocal(tfPose(3) - tfInitPose(3));
        tfPoseRel = [tfPose(1) - tfInitPose(1), tfPose(2) - tfInitPose(2), tfYawRel];

        if isempty(prevAcceptedTfPose)
            relPose = [0, 0, 0];
        else
            relPose = poseDiff2D(prevAcceptedTfPose, tfPoseRel);
        end

        % Step 4: Add scan to map with relative pose.
        [isAccepted, ~, ~] = addScan(slamObj, scan, relPose);

        if isAccepted
            acceptedCount = acceptedCount + 1;
            prevAcceptedTfPose = tfPoseRel;

            % Step 5: Read current robot pose estimate from scansAndPoses.
            [~, poses] = scansAndPoses(slamObj);
            currentPose = poses(end, :);
            disp(['Accepted scan #' num2str(acceptedCount) ...
                  ', SLAM pose [x y yaw]=[' ...
                  num2str(currentPose(1), '%.3f') ' ' ...
                  num2str(currentPose(2), '%.3f') ' ' ...
                  num2str(currentPose(3), '%.3f') ']']);

            % --- IMPROVEMENT: Update Occupancy Map with insertRay ---
            % Ray tracing lowers the occupancy probability along the beam path
            % and increases it at the hit point. This clears out moved obstacles!
            insertRay(occMap, currentPose, scan, cfg.maxLidarRange);

            % Live map refresh on each accepted scan showing the occupancy grid
            cla(mapAx);
            show(occMap, 'Parent', mapAx);
            title(mapAx, 'Live Occupancy Map (Clearing Dynamic Obstacles)');
            
            % Overlay the current robot pose as a red dot
            hold(mapAx, 'on');
            plot(mapAx, currentPose(1), currentPose(2), 'r.', 'MarkerSize', 15);
            hold(mapAx, 'off');
        end

        % Continuously show robot heading in TurtleBotVisualise.
        visualise = updatePose(visualise, tfPoseRel(1:2), tfPoseRel(3));
        if exist('scan', 'var')
            cart = scan.Cartesian;
            if ~isempty(cart)
                visualise = updateScan(visualise, cart, tfPoseRel(1:2), tfPoseRel(3));
            end
        end

        drawnow limitrate;

        pause(0.001);
    end

catch ME
    clear scanSub tfSub
    if ~strcmp(ME.identifier, 'L7SLAM:UserClosedFigure')
        rethrow(ME)
    end
end

clear scanSub tfSub

%% Final outputs
disp(['Accepted scans: ' num2str(acceptedCount)]);

[scans, poses] = scansAndPoses(slamObj);
disp(['Pose graph nodes: ' num2str(size(poses, 1))]);

% Optional: Rebuild a highly optimized map at the very end
disp('Building final optimized occupancy map from SLAM graph...');
finalMap = buildMap(scans, poses, cfg.mapResolution, cfg.maxLidarRange);
figure('Name', 'Final Optimized SLAM Map');
show(finalMap);
title('Final Optimized Occupancy Map');

%% Callbacks and helpers
function scanCallback(message)
    global scanMsg
    scanMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end

function stampStr = readScanStamp(scanMessage)
    sec = 0;
    nanosec = 0;

    try
        sec = double(scanMessage.header.stamp.sec);
        nanosec = double(scanMessage.header.stamp.nanosec);
    catch
        try
            sec = double(scanMessage.Header.Stamp.Sec);
            nanosec = double(scanMessage.Header.Stamp.Nanosec);
        catch
            stampStr = sprintf('fallback_%f', now);
            return;
        end
    end

    stampStr = sprintf('%d_%d', sec, nanosec);
end

function [scan, nValid] = lidarScanFromRos(scanMessage, maxRange, minValidRange, maxRangeMargin, medianWindow)
    try
        ranges = double(scanMessage.ranges);
        angleMin = double(scanMessage.angle_min);
        angleInc = double(scanMessage.angle_increment);
    catch
        ranges = double(scanMessage.Ranges);
        angleMin = double(scanMessage.AngleMin);
        angleInc = double(scanMessage.AngleIncrement);
    end

    ranges = ranges(:);
    angles = angleMin + (0:numel(ranges)-1)' * angleInc;

    % Remove invalid and max-range/no-return samples before SLAM.
    isFinite = isfinite(ranges);
    isAboveMin = ranges > minValidRange;
    isBelowMax = ranges < (maxRange - maxRangeMargin);
    keep = isFinite & isAboveMin & isBelowMax;

    ranges = ranges(keep);
    angles = angles(keep);

    if isempty(ranges)
        nValid = 0;
        scan = lidarScan(0.1, 0);
        return;
    end

    % Simple denoising: moving median removes isolated spikes.
    w = max(1, 2 * floor(medianWindow / 2) + 1); % force odd window
    if numel(ranges) >= w
        ranges = movmedian(ranges, w);
    end

    nValid = numel(ranges);

    scan = lidarScan(ranges, angles);
end

function pose = extractOdomPoseFromTf(tfMessage)
    pose = [];

    if isempty(tfMessage)
        return;
    end

    transforms = [];
    try
        transforms = tfMessage.transforms;
    catch
        try
            transforms = tfMessage.Transforms;
        catch
            return;
        end
    end

    for i = 1:numel(transforms)
        t = transforms(i);

        try
            parentFrame = string(t.header.frame_id);
            childFrame = string(t.child_frame_id);
            x = t.transform.translation.x;
            y = t.transform.translation.y;
            qx = t.transform.rotation.x;
            qy = t.transform.rotation.y;
            qz = t.transform.rotation.z;
            qw = t.transform.rotation.w;
        catch
            try
                parentFrame = string(t.Header.FrameId);
                childFrame = string(t.ChildFrameId);
                x = t.Transform.Translation.X;
                y = t.Transform.Translation.Y;
                qx = t.Transform.Rotation.X;
                qy = t.Transform.Rotation.Y;
                qz = t.Transform.Rotation.Z;
                qw = t.Transform.Rotation.W;
            catch
                continue;
            end
        end

        isOdomParent = contains(parentFrame, 'odom');
        isBaseChild = contains(childFrame, 'base_link') || contains(childFrame, 'base_footprint');

        if isOdomParent && isBaseChild
            yaw = quatRosToYaw(qx, qy, qz, qw);
            pose = [x, y, yaw];
            return;
        end
    end
end

function yaw = quatRosToYaw(x, y, z, w)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
end

function d = poseDiff2D(pFrom, pTo)
    dx = pTo(1) - pFrom(1);
    dy = pTo(2) - pFrom(2);
    dth = wrapToPiLocal(pTo(3) - pFrom(3));

    c = cos(-pFrom(3));
    s = sin(-pFrom(3));
    dxyLocal = [c, -s; s, c] * [dx; dy];

    d = [dxyLocal(1), dxyLocal(2), dth];
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end