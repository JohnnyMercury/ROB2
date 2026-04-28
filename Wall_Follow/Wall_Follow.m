%% Wall Follow (minimal working example)
% This script follows a wall using only LiDAR and /cmd_vel.
% Main idea:
% 1) Keep only one side sector of scan data (left or right wall side)
% 2) Fit a line to those points (with one inlier-refit step for robustness)
% 3) Extract wall distance and wall angle in robot frame
% 4) Drive forward with fixed speed and correct heading using angular control

clear all
clc
close all

global scanMsg tfMsg
scanMsg = [];
tfMsg = [];

%% User configuration (keep this simple)
cfg.rosDomainId = '30';
cfg.wallSide = 'left';      % 'left' or 'right'
cfg.desiredWallDistance = 0.50;  % m
cfg.forwardSpeed = 0.10;    % m/s
cfg.maxAngularSpeed = 0.80; % rad/s

% LiDAR filtering
cfg.minValidRange = 0.08;
cfg.maxValidRange = 3.40;
cfg.sideCenterDeg = 90;     % left wall uses +90 deg, right uses -90 deg
cfg.sideHalfWidthDeg = 60;  % wider sector makes acquisition easier when far from wall
cfg.minSidePoints = 12;
cfg.lineInlierThreshold = 0.08; % looser inlier gate improves fit stability on sparse returns
cfg.useIndexBasedAngles = false; % keep ROS-reported angle mapping

% Control gains (simple PI for distance + P for angle)
cfg.kpDistance = 1.8;
cfg.kiDistance = 0.25;
cfg.kpAngle = 1.1;
cfg.integralClamp = 0.5;

% Front safety override
cfg.frontHalfWidthDeg = 20;
cfg.frontStopDistance = 0.35;
cfg.turnAwayRate = 0.60;

% ROS startup timing
cfg.maxWaitSec = 120;
cfg.pollInterval = 0.5;
cfg.firstMsgTimeoutSec = 25;

if strcmpi(cfg.wallSide, 'left')
    sideSign = 1;
else
    sideSign = -1;
    cfg.sideCenterDeg = -abs(cfg.sideCenterDeg);
end

%% ROS2 setup
setenv('ROS_DOMAIN_ID', cfg.rosDomainId);
node = ros2node('/base_station_wall_follow');

requiredTopics = ["/scan", "/tf"];
startTime = tic;

disp('Waiting for required ROS2 topics /scan and /tf ...');
while toc(startTime) < cfg.maxWaitSec
    try
        topicOutput = evalc("ros2 topic list");
        topics = strtrim(splitlines(string(topicOutput)));
        topics = topics(topics ~= "");
        if all(ismember(requiredTopics, topics))
            disp(['Topic found after ' num2str(toc(startTime), '%.1f') ' seconds.']);
            break;
        end
    catch
        % Ignore transient discovery errors.
    end
    pause(cfg.pollInterval);
end

if toc(startTime) >= cfg.maxWaitSec
    error('Timeout waiting for /scan and /tf topics.');
end

%% Subscriber and publisher
scanSub = ros2subscriber(node, '/scan', @scanCallback, 'Reliability', 'besteffort');
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');
cmdPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
cmdMsg = ros2message('geometry_msgs/Twist');
pause(0.5);

%% Wait for first scan and tf
firstMsgStart = tic;
disp('Waiting for first /scan and /tf messages...');
while isempty(scanMsg) || isempty(tfMsg)
    if toc(firstMsgStart) > cfg.firstMsgTimeoutSec
        error('Timeout waiting for first /scan and /tf messages.');
    end
    pause(0.01);
end

%% Minimal live visual (robot frame)
fig = figure('Name', 'Wall Follow (robot frame)', 'NumberTitle', 'off');
ax = axes('Parent', fig);
hPts = scatter(ax, nan, nan, 8, 'b', 'filled'); hold(ax, 'on');
hLine = plot(ax, nan, nan, 'r-', 'LineWidth', 2);
hText = text(ax, -1.8, 1.6, '', 'FontSize', 10);
grid(ax, 'on'); axis(ax, 'equal');
xlim(ax, [-2 2]); ylim(ax, [-2 2]);
xlabel(ax, 'x (forward) [m]'); ylabel(ax, 'y (left) [m]');
title(ax, 'Blue: side points, Red: fitted wall');

% Second figure: robot pose/path and LiDAR points in world frame.
visualise = TurtleBotVisualise();
title(visualise.h_ax, 'TurtleBot pose, trajectory and LiDAR points');

%% Control state
lastScanStamp = '';
loopStart = tic;
prevTime = 0;
integralDistance = 0;
printCounter = 0;
tfInitPose = [];
lastWall = [];
lastWallTime = -inf;
cfg.wallHoldSec = 0.35; % hold last wall estimate briefly to prevent jitter on dropped fits

disp('Running wall follow. Close figure to stop.');

try
    while true
        if ~isgraphics(fig) || ~isgraphics(visualise.fig)
            error('WallFollow:UserClosedFigure', 'Figure closed by user.');
        end

        % Process each scan once.
        stamp = readScanStamp(scanMsg);
        if strcmp(stamp, lastScanStamp)
            pause(0.001);
            continue;
        end
        lastScanStamp = stamp;

        tNow = toc(loopStart);
        dt = max(1e-3, tNow - prevTime);
        prevTime = tNow;

        [ranges, angles] = readRangesAngles(scanMsg, cfg.useIndexBasedAngles);
        tfPose = extractOdomPoseFromTf(tfMsg);

        if ~isempty(tfPose)
            if isempty(tfInitPose)
                tfInitPose = tfPose;
            end
            tfYawRel = wrapToPiLocal(tfPose(3) - tfInitPose(3));
            tfPoseRel = [tfPose(1) - tfInitPose(1), tfPose(2) - tfInitPose(2), tfYawRel];
        else
            tfPoseRel = [0, 0, 0];
        end

        % Front safety check: if obstacle is too close in front, rotate away.
        frontMin = minRangeInSector(ranges, angles, -cfg.frontHalfWidthDeg, cfg.frontHalfWidthDeg, cfg.minValidRange, cfg.maxValidRange);

        % Estimate wall line from side sector.
        [ok, wall] = estimateWallFromSideSector(ranges, angles, cfg, sideSign);

        if ok
            lastWall = wall;
            lastWallTime = tNow;
        elseif ~isempty(lastWall) && (tNow - lastWallTime) < cfg.wallHoldSec
            wall = lastWall;
            ok = true;
        end

        if ~ok
            % If wall is not visible, do a gentle biased search instead of spinning in place.
            cmdMsg.linear.x = 0.07;
            cmdMsg.angular.z = 0.12 * sideSign;
            send(cmdPub, cmdMsg);

            cartAll = scanToCartesian(ranges, angles, cfg.minValidRange, cfg.maxValidRange);
            if ~isempty(cartAll)
                visualise = updateScan(visualise, cartAll, tfPoseRel(1:2), tfPoseRel(3));
            end
            visualise = updatePose(visualise, tfPoseRel(1:2), tfPoseRel(3));
            drawnow limitrate;
            continue;
        end

        % Distance error > 0 means we are farther from wall than desired.
        distanceError = wall.distance - cfg.desiredWallDistance;
        integralDistance = integralDistance + distanceError * dt;
        integralDistance = clip(integralDistance, -cfg.integralClamp, cfg.integralClamp);

        % Wall angle is the angle between robot x-axis and wall direction.
        % Target is zero (robot parallel to wall).
        angleError = wall.angle;

         % Distance correction depends on side; angle alignment sign is shared.
        angularCmd = sideSign * (cfg.kpDistance * distanceError + cfg.kiDistance * integralDistance) ...
             + cfg.kpAngle * angleError;
        linearCmd = cfg.forwardSpeed;

         % Reduce forward speed during large heading mismatch.
         if abs(angleError) > deg2rad(25)
             linearCmd = 0.04;
         end

        if frontMin < cfg.frontStopDistance
            linearCmd = 0.0;
            angularCmd = -sideSign * cfg.turnAwayRate;
        end

        cmdMsg.linear.x = clip(linearCmd, -0.15, 0.15);
        cmdMsg.angular.z = clip(angularCmd, -cfg.maxAngularSpeed, cfg.maxAngularSpeed);
        send(cmdPub, cmdMsg);

        % Update plot with points and fitted line segment.
        set(hPts, 'XData', wall.points(:,1), 'YData', wall.points(:,2));
        set(hLine, 'XData', wall.lineXY(:,1), 'YData', wall.lineXY(:,2));
        set(hText, 'String', sprintf('d=%.3f m (target %.2f), angle=%.3f rad', wall.distance, cfg.desiredWallDistance, wall.angle));

        cartAll = scanToCartesian(ranges, angles, cfg.minValidRange, cfg.maxValidRange);
        if ~isempty(cartAll)
            visualise = updateScan(visualise, cartAll, tfPoseRel(1:2), tfPoseRel(3));
        end
        visualise = updatePose(visualise, tfPoseRel(1:2), tfPoseRel(3));
        drawnow limitrate;

        printCounter = printCounter + 1;
        if mod(printCounter, 10) == 0
            disp(sprintf('d=%.3f m, angle=%.3f rad, v=%.2f, w=%.2f', wall.distance, wall.angle, cmdMsg.linear.x, cmdMsg.angular.z));
        end
    end

catch ME
    % Always stop robot when exiting.
    cmdMsg.linear.x = 0.0;
    cmdMsg.angular.z = 0.0;
    send(cmdPub, cmdMsg);

    clear scanSub tfSub

    if ~strcmp(ME.identifier, 'WallFollow:UserClosedFigure')
        rethrow(ME)
    end
end

clear scanSub tfSub

%% Callback
function scanCallback(message)
    global scanMsg
    scanMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end

%% Helpers
function stampStr = readScanStamp(scanMessage)
    % Read scan timestamp robustly across lowercase/uppercase field names.
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

function [ranges, angles] = readRangesAngles(scanMessage, useIndexBasedAngles)
    % Extract ranges and corresponding angles from ROS LaserScan.
    try
        ranges = double(scanMessage.ranges(:));
        angleMin = double(scanMessage.angle_min);
        angleInc = double(scanMessage.angle_increment);
    catch
        ranges = double(scanMessage.Ranges(:));
        angleMin = double(scanMessage.AngleMin);
        angleInc = double(scanMessage.AngleIncrement);
    end

    if useIndexBasedAngles
        % TurtleBot LDS convention observed in workspace:
        % index 0: front, 90: right, 180: back, 270: left.
        % Convert to robot-frame math angles (x forward, y left):
        % right should be -pi/2 and left should be +pi/2.
        idxDeg = (0:numel(ranges)-1)';
        angles = wrapToPiLocal(-deg2rad(idxDeg));
    else
        angles = angleMin + (0:numel(ranges)-1)' * angleInc;
        angles = wrapToPiLocal(angles);
    end
end

function cart = scanToCartesian(ranges, angles, minR, maxR)
    keep = isfinite(ranges) & ranges > minR & ranges < maxR;
    if ~any(keep)
        cart = zeros(0, 2);
        return;
    end

    r = ranges(keep);
    a = angles(keep);
    cart = [r .* cos(a), r .* sin(a)];
end

function m = minRangeInSector(ranges, angles, aMinDeg, aMaxDeg, minR, maxR)
    aMin = deg2rad(aMinDeg);
    aMax = deg2rad(aMaxDeg);

    keep = isfinite(ranges) & ranges > minR & ranges < maxR & angles >= aMin & angles <= aMax;
    if any(keep)
        m = min(ranges(keep));
    else
        m = inf;
    end
end

function [ok, wall] = estimateWallFromSideSector(ranges, angles, cfg, sideSign)
    ok = false;
    wall = struct('distance', nan, 'angle', nan, 'points', zeros(0,2), 'lineXY', zeros(0,2));

    center = deg2rad(cfg.sideCenterDeg);
    halfW = deg2rad(cfg.sideHalfWidthDeg);

    keep = isfinite(ranges) & ...
           ranges > cfg.minValidRange & ranges < cfg.maxValidRange & ...
           angles >= (center - halfW) & angles <= (center + halfW);

    if nnz(keep) < cfg.minSidePoints
        return;
    end

    r = ranges(keep);
    a = angles(keep);

    % Robot frame: x forward, y left.
    x = r .* cos(a);
    y = r .* sin(a);
    pts = [x, y];

    % First line fit using total least squares (PCA on point cloud).
    [p0, v, n, residual] = fitLineTLS(pts);

    % One robust step: keep inliers near line and refit.
    inliers = residual < cfg.lineInlierThreshold;
    if nnz(inliers) < cfg.minSidePoints
        return;
    end
    ptsIn = pts(inliers, :);
    [p0, v, n, ~] = fitLineTLS(ptsIn);

    % Keep line direction mostly forward for a stable angle sign.
    if v(1) < 0
        v = -v;
        n = -n;
    end

    % Orient line normal toward selected side so distance is positive.
    if sideSign > 0 && n(2) < 0
        n = -n;
    end
    if sideSign < 0 && n(2) > 0
        n = -n;
    end

    distance = dot(n, p0);
    if distance < 0
        distance = abs(distance);
    end

    angle = atan2(v(2), v(1));
    angle = wrapToPiLocal(angle);
    if angle > pi/2
        angle = angle - pi;
    elseif angle < -pi/2
        angle = angle + pi;
    end

    % Create a short segment for plotting.
    s = linspace(-1.5, 1.5, 2)';
    lineXY = p0 + s .* v;

    wall.distance = distance;
    wall.angle = angle;
    wall.points = ptsIn;
    wall.lineXY = lineXY;

    ok = true;
end

function [p0, v, n, residual] = fitLineTLS(pts)
    % Total least squares line fit in 2D.
    p0 = mean(pts, 1);
    X = pts - p0;

    C = (X' * X) / size(X, 1);
    [V, D] = eig(C);
    [~, idx] = max(diag(D));

    v = V(:, idx)';
    v = v / norm(v);

    n = [-v(2), v(1)];
    n = n / norm(n);

    residual = abs(X * n');
end

function y = clip(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
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
