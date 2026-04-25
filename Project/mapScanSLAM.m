%% mapScanSLAM
% Stage 1 mapping script.
% Runs SLAM while you drive the robot externally (teleop), then saves a
% reusable SLAM session file into Project/Maps.
%
% Usage:
% 1) Start TurtleBot bringup and teleop.
% 2) Run this script in MATLAB.
% 3) Drive around the full environment.
% 4) Close the figure (or press Ctrl+C) to stop and save.

clear;
clc;
close all;

global g_scan_msg g_tf_msg
g_scan_msg = [];
g_tf_msg = [];

%% User config
cfg.rosDomainId = '30';
cfg.maxWaitTopicsSec = 60;
cfg.firstMsgTimeoutSec = 20;
cfg.pollIntervalSec = 0.5;

cfg.mapResolution = 20;       % cells/m
cfg.maxLidarRange = 3.5;      % LDS-01 default
cfg.maxNumScans = 6000;

cfg.loopClosureThreshold = 210;
cfg.loopClosureSearchRadius = 6;

cfg.minValidRange = 0.08;
cfg.maxRangeMargin = 0.02;
cfg.onlineMedianWindow = 9;
cfg.minValidPointsPerScan = 12;

cfg.saveFolder = fullfile(fileparts(mfilename('fullpath')), 'Maps');
cfg.sessionFilePrefix = 'slam_session_test_';

%% ROS setup
if isempty(getenv('ROS_DOMAIN_ID')) || ~strcmp(getenv('ROS_DOMAIN_ID'), cfg.rosDomainId)
    setenv('ROS_DOMAIN_ID', cfg.rosDomainId);
end

node = ros2node('/project_map_scan_slam');

requiredTopics = ["/scan", "/tf"];
startWait = tic;
fprintf('[INIT] Waiting for /scan and /tf...\n');
while toc(startWait) < cfg.maxWaitTopicsSec
    try
        topicOutput = evalc('ros2 topic list');
        topics = strtrim(splitlines(string(topicOutput)));
        topics = topics(topics ~= "");
        if all(ismember(requiredTopics, topics))
            fprintf('[INIT] Required topics discovered in %.1f s\n', toc(startWait));
            break;
        end
    catch
        % Ignore transient discovery failures.
    end
    pause(cfg.pollIntervalSec);
end
if toc(startWait) >= cfg.maxWaitTopicsSec
    error('Timed out waiting for /scan and /tf topics.');
end

scanSub = ros2subscriber(node, '/scan', @scanCallback, 'Reliability', 'besteffort');
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');
pause(0.5);

fprintf('[INIT] Waiting for first /scan and /tf messages...\n');
firstWait = tic;
while isempty(g_scan_msg) || isempty(g_tf_msg)
    if toc(firstWait) > cfg.firstMsgTimeoutSec
        error('Timed out waiting for first /scan and /tf messages.');
    end
    pause(0.01);
end

%% SLAM object
slamObj = lidarSLAM(cfg.mapResolution, cfg.maxLidarRange, cfg.maxNumScans);
slamObj.LoopClosureThreshold = cfg.loopClosureThreshold;
slamObj.LoopClosureSearchRadius = cfg.loopClosureSearchRadius;

%% Runtime state
acceptedCount = 0;
lastScanStamp = "";
tfInitPose = [];
prevAcceptedTfPose = [];

scanArchive = struct('ranges', {}, 'angles', {}, 'relPose', {}, 'tfPoseRel', {}, 'stamp', {});

% Visualization
fig = figure('Name', 'Project SLAM Scan (teleop)', 'NumberTitle', 'off');
ax = axes('Parent', fig);
show(slamObj, 'Parent', ax);
title(ax, 'Live SLAM Map (close figure to stop + save)');

fprintf('[RUN] Mapping started. Drive with teleop.\n');

try
    while true
        if ~isgraphics(fig)
            error('MapScanSLAM:UserStop', 'User closed figure.');
        end

        scanStamp = readScanStamp(g_scan_msg);
        if scanStamp == lastScanStamp
            drawnow limitrate;
            pause(0.001);
            continue;
        end
        lastScanStamp = scanStamp;

        [scan, nValid] = lidarScanFromRos(g_scan_msg, cfg.maxLidarRange, cfg.minValidRange, cfg.maxRangeMargin, cfg.onlineMedianWindow);
        if nValid < cfg.minValidPointsPerScan
            pause(0.001);
            continue;
        end

        tfPose = extractOdomPoseFromTf(g_tf_msg);
        if isempty(tfPose)
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

        [isAccepted, ~, ~] = addScan(slamObj, scan, relPose);
        if isAccepted
            acceptedCount = acceptedCount + 1;
            prevAcceptedTfPose = tfPoseRel;

            scanArchive(acceptedCount).ranges = scan.Ranges;
            scanArchive(acceptedCount).angles = scan.Angles;
            scanArchive(acceptedCount).relPose = relPose;
            scanArchive(acceptedCount).tfPoseRel = tfPoseRel;
            scanArchive(acceptedCount).stamp = char(scanStamp);

            if mod(acceptedCount, 10) == 0
                [~, poses] = scansAndPoses(slamObj);
                p = poses(end, :);
                fprintf('Accepted scan #%d | pose=[%.3f %.3f %.3f]\n', acceptedCount, p(1), p(2), p(3));
            end

            cla(ax);
            show(slamObj, 'Parent', ax);
            title(ax, 'Live SLAM Map (close figure to stop + save)');
        end

        drawnow limitrate;
        pause(0.001);
    end
catch ME
    clear scanSub tfSub
    if ~strcmp(ME.identifier, 'MapScanSLAM:UserStop')
        rethrow(ME);
    end
end

%% Save session
if ~exist(cfg.saveFolder, 'dir')
    mkdir(cfg.saveFolder);
end

session = struct();
session.cfg = cfg;
session.createdAt = datestr(now, 30);
session.acceptedCount = acceptedCount;
session.scanArchive = scanArchive;
[session.scans, session.poses] = scansAndPoses(slamObj);
session.mapResolution = cfg.mapResolution;
session.maxLidarRange = cfg.maxLidarRange;

try
    session.rawOccupancyMap = getOccupancyMapCompat(slamObj, cfg.mapResolution, cfg.maxLidarRange);
catch
    session.rawOccupancyMap = [];
end

session.slamObj = slamObj;

saveName = [cfg.sessionFilePrefix datestr(now, 'yyyymmdd_HHMMSS') '.mat'];
savePath = fullfile(cfg.saveFolder, saveName);
save(savePath, 'session', '-v7.3');

fprintf('[SAVE] Accepted scans: %d\n', acceptedCount);
fprintf('[SAVE] Session file: %s\n', savePath);

%% Local helpers
function scanCallback(message)
    global g_scan_msg
    g_scan_msg = message;
end

function tfCallback(message)
    global g_tf_msg
    g_tf_msg = message;
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
            stampStr = string(sprintf('fallback_%0.6f', now));
            return;
        end
    end
    stampStr = string(sprintf('%d_%d', sec, nanosec));
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

    keep = isfinite(ranges) & ranges > minValidRange & ranges < (maxRange - maxRangeMargin);
    ranges = ranges(keep);
    angles = angles(keep);

    if isempty(ranges)
        nValid = 0;
        scan = lidarScan(0.1, 0);
        return;
    end

    w = max(1, 2 * floor(medianWindow / 2) + 1);
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

        if contains(parentFrame, 'odom') && (contains(childFrame, 'base_link') || contains(childFrame, 'base_footprint'))
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
    a = mod(a + pi, 2 * pi) - pi;
end

function occMap = getOccupancyMapCompat(slamObj, mapResolution, maxLidarRange)
    try
        occMap = getOccupancyGrid(slamObj);
        return;
    catch
        % Fall back for MATLAB versions where getOccupancyGrid(lidarSLAM)
        % is unavailable.
    end

    [scans, poses] = scansAndPoses(slamObj);
    if isempty(scans) || isempty(poses)
        error('Cannot build occupancy map: no scans/poses in SLAM object.');
    end

    occMap = buildMap(scans, poses, mapResolution, maxLidarRange);
end
