%% L7 Exercises: Dead Reckoning + LiDAR SLAM (ROS2)
% Exercise 1:
%   - IMU inertial dead reckoning in 2D.
% Exercise 2:
%   - Build a LiDAR SLAM map from /scan and compare against DR and /tf odometry.

clear all
clc
close all

global imuMsg tfMsg scanMsg
imuMsg = [];
tfMsg = [];
scanMsg = [];

%% User configuration
cfg.rosDomainId = '30';
cfg.maxWaitSec = 120;
cfg.pollInterval = 0.5;
cfg.firstMsgTimeoutSec = 25;
cfg.runDurationSec = 150;
cfg.plotUpdateInterval = 0.2;

% IMU dead-reckoning tuning.
cfg.maxImuDtSec = 0.1;
cfg.accelDeadband = 0.08;
cfg.gyroDeadband = 0.03;
cfg.stationaryHoldSec = 0.25;
cfg.accelLpfTauSec = 0.35;
cfg.biasCalibSec = 2.0;
cfg.zeroVelocityOnLowAccel = true;
cfg.planarMotionOnly = true;
cfg.removeGravity = true;
cfg.gWorld = [0, 0, 9.81];
cfg.useYawOnlyAccelTransform = true;

% Simple fusion: IMU prediction + TF correction.
cfg.enableSimpleFusion = true;
cfg.fusionPosTauSec = 1.2;
cfg.fusionYawTauSec = 0.8;
cfg.fusionVelFromPosErrGain = 0.15;

% LiDAR SLAM tuning.
cfg.mapResolution = 20;          % cells per meter
cfg.maxLidarRange = 3.5;         % LDS-02 ~8m, change to 3.5 for LDS-01
cfg.loopClosureThreshold = 210;
cfg.loopClosureSearchRadius = 6;
cfg.maxNumScans = 3500;

%% ROS2 setup
setenv('ROS_DOMAIN_ID', cfg.rosDomainId);
node = ros2node('/base_station_l7_lidar_slam');

% Wait for required topics before creating subscribers.
maxWaitSec = cfg.maxWaitSec;
pollInterval = cfg.pollInterval;
requiredTopics = ["/imu", "/tf", "/scan"];
startTime = tic;

disp('Waiting for required ROS2 topics...');
while toc(startTime) < maxWaitSec
    try
        topicOutput = evalc("ros2 topic list");
        topics = strtrim(splitlines(string(topicOutput)));
        topics = topics(topics ~= "");
        if all(ismember(requiredTopics, topics))
            disp(['Topics found after ' num2str(toc(startTime), '%.1f') ' seconds.']);
            break;
        end
    catch
        % Ignore transient discovery errors during startup.
    end
    pause(pollInterval);
end

if toc(startTime) >= maxWaitSec
    error('Timeout waiting for /imu, /tf, /scan topics. Check TurtleBot bringup.');
end

%% Subscribers
imuSub = ros2subscriber(node, '/imu', @imuCallback);
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');
scanSub = ros2subscriber(node, '/scan', @scanCallback, 'Reliability', 'besteffort');
pause(0.5);

%% Wait for first messages
disp('Waiting for first IMU, TF, and Scan messages...');
firstMsgStart = tic;
while isempty(imuMsg) || isempty(tfMsg) || isempty(scanMsg)
    if toc(firstMsgStart) > cfg.firstMsgTimeoutSec
        error('Timeout waiting for first /imu, /tf, /scan messages.');
    end
    pause(0.01);
end

%% Exercise 1 state: dead reckoning from IMU
pWorld = [0, 0, 0];
vWorld = [0, 0, 0];
yawImuInit = [];
yawImuRel = 0;
imuTimePrev = NaN;

biasWorld = [0, 0, 0];
biasAccum = [0, 0, 0];
biasCount = 0;
biasReady = false;
biasStartImuSec = NaN;
aWorldFilt = [0, 0, 0];
stationaryAccumSec = 0;

tfInitPose = [];
tfPoseRel = [nan, nan, nan];

%% Exercise 2 state: LiDAR SLAM
slamObj = lidarSLAM(cfg.mapResolution, cfg.maxLidarRange, cfg.maxNumScans);
slamObj.LoopClosureThreshold = cfg.loopClosureThreshold;
slamObj.LoopClosureSearchRadius = cfg.loopClosureSearchRadius;

lastScanStamp = '';
prevScanTfPose = [];
prevScanDrPose = [];

%% Logging for comparison
logT = [];
logTf = [];
logDr = [];
logSlam = [];
logScanAccepted = [];

%% Live trajectory figure
figTraj = figure('Name', 'L7 Dead Reckoning + LiDAR SLAM Trajectories', 'NumberTitle', 'off');
axTraj = gca;
hold(axTraj, 'on');
grid(axTraj, 'on');
axis(axTraj, 'equal');
xlabel(axTraj, 'x [m]');
ylabel(axTraj, 'y [m]');
title(axTraj, 'Trajectory Comparison');

hDr = plot(axTraj, NaN, NaN, 'b-', 'LineWidth', 1.2);
hTf = plot(axTraj, NaN, NaN, 'r--', 'LineWidth', 1.2);
hSlam = plot(axTraj, NaN, NaN, 'k-', 'LineWidth', 1.4);
legend(axTraj, {'Dead reckoning/fused (IMU)', 'Odometry from /tf', 'LiDAR SLAM pose graph'}, 'Location', 'best');

disp('Running Exercise 1 + 2. Move TurtleBot to build map...');
loopStart = tic;
lastPlotUpdate = -inf;

try
    while true
        tNow = toc(loopStart);

        %% 1) Update dead reckoning from each new IMU sample
        imuTimeNow = readImuTimeSec(imuMsg);
        if isfinite(imuTimeNow)
            if ~isnan(imuTimePrev)
                dtImu = imuTimeNow - imuTimePrev;
            else
                dtImu = NaN;
            end

            if isfinite(dtImu) && dtImu > 0
                dtImu = min(dtImu, cfg.maxImuDtSec);

                qRos = [imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w];
                qMatlab = [qRos(4), qRos(1), qRos(2), qRos(3)]; % [w x y z]

                yawImu = quatRosToYaw(qRos(1), qRos(2), qRos(3), qRos(4));
                if isempty(yawImuInit)
                    yawImuInit = yawImu;
                end
                yawImuRel = wrapToPiLocal(yawImu - yawImuInit);

                aBody = [imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z];
                wBody = [imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z];

                aBodyLin = aBody;
                if cfg.removeGravity
                    gBody = quatrotate(quatconj(qMatlab), cfg.gWorld);
                    aBodyLin = aBody - gBody;
                end

                if cfg.useYawOnlyAccelTransform
                    c = cos(yawImuRel);
                    s = sin(yawImuRel);
                    aXYWorld = [c, -s; s, c] * aBodyLin(1:2)';
                    aWorldRaw = [aXYWorld(1), aXYWorld(2), 0];
                else
                    aWorldRaw = quatrotate(qMatlab, aBodyLin);
                end

                if ~biasReady
                    if isnan(biasStartImuSec)
                        biasStartImuSec = imuTimeNow;
                        disp('Calibrating IMU bias. Keep robot still...');
                    end

                    biasAccum = biasAccum + aWorldRaw;
                    biasCount = biasCount + 1;
                    if (imuTimeNow - biasStartImuSec) >= cfg.biasCalibSec
                        biasWorld = biasAccum / max(biasCount, 1);
                        biasReady = true;
                        disp(['IMU bias estimate [m/s^2]: [' num2str(biasWorld(1), '%.4f') ', ' num2str(biasWorld(2), '%.4f') ', ' num2str(biasWorld(3), '%.4f') ']']);
                    end
                else
                    aWorld = aWorldRaw - biasWorld;

                    alpha = dtImu / (cfg.accelLpfTauSec + dtImu);
                    aWorldFilt = aWorldFilt + alpha * (aWorld - aWorldFilt);
                    aWorldUse = aWorldFilt;

                    isLowAccel = norm(aWorldUse) < cfg.accelDeadband;
                    isLowGyro = norm(wBody) < cfg.gyroDeadband;
                    if isLowAccel && isLowGyro
                        stationaryAccumSec = stationaryAccumSec + dtImu;
                    else
                        stationaryAccumSec = 0;
                    end

                    if stationaryAccumSec >= cfg.stationaryHoldSec
                        aWorldUse = [0, 0, 0];
                    end

                    if norm(aWorldUse) < cfg.accelDeadband
                        aWorldUse = [0, 0, 0];
                        if cfg.zeroVelocityOnLowAccel
                            vWorld = [0, 0, 0];
                        end
                    end

                    if cfg.planarMotionOnly
                        aWorldUse(3) = 0;
                        vWorld(3) = 0;
                        pWorld(3) = 0;
                    end

                    pWorld = pWorld + vWorld * dtImu + 0.5 * aWorldUse * dtImu * dtImu;
                    vWorld = vWorld + aWorldUse * dtImu;
                end
            end

            imuTimePrev = imuTimeNow;
        end

        %% 2) Read current TF odometry pose
        tfPose = extractOdomPoseFromTf(tfMsg);
        if ~isempty(tfPose)
            if isempty(tfInitPose)
                tfInitPose = tfPose;
            end
            tfYawRel = wrapToPiLocal(tfPose(3) - tfInitPose(3));
            tfPoseRel = [tfPose(1) - tfInitPose(1), tfPose(2) - tfInitPose(2), tfYawRel];
        else
            tfPoseRel = [nan, nan, nan];
        end

        %% 3) Simple fusion correction toward TF (optional)
        if cfg.enableSimpleFusion && all(isfinite(tfPoseRel)) && isfinite(imuTimeNow) && ~isnan(imuTimePrev)
            dtFuse = min(max(dtImu, 1e-3), cfg.maxImuDtSec);
            kPos = dtFuse / (cfg.fusionPosTauSec + dtFuse);
            kYaw = dtFuse / (cfg.fusionYawTauSec + dtFuse);

            posErr = tfPoseRel(1:2) - pWorld(1:2);
            pWorld(1:2) = pWorld(1:2) + kPos * posErr;
            yawErr = wrapToPiLocal(tfPoseRel(3) - yawImuRel);
            yawImuRel = wrapToPiLocal(yawImuRel + kYaw * yawErr);
            pWorld(3) = yawImuRel;

            vWorld(1:2) = vWorld(1:2) + cfg.fusionVelFromPosErrGain * (kPos / max(dtFuse, 1e-3)) * posErr;
        end

        %% 4) Process each new /scan and add to lidarSLAM
        scanStamp = readScanStamp(scanMsg);
        if ~strcmp(scanStamp, lastScanStamp)
            lastScanStamp = scanStamp;

            scan = lidarScanFromRos(scanMsg, cfg.maxLidarRange);

            tfValid = all(isfinite(tfPoseRel));
            drPoseNow = [pWorld(1), pWorld(2), pWorld(3)];

            if isempty(prevScanTfPose) && tfValid
                relPose = [0, 0, 0];
            elseif tfValid && ~isempty(prevScanTfPose)
                relPose = poseDiff2D(prevScanTfPose, tfPoseRel);
            elseif ~isempty(prevScanDrPose)
                relPose = poseDiff2D(prevScanDrPose, drPoseNow);
            else
                relPose = [0, 0, 0];
            end

            [isAccepted, ~, ~] = addScan(slamObj, scan, relPose);
            logScanAccepted(end+1, 1) = double(isAccepted);

            if isAccepted
                if tfValid
                    prevScanTfPose = tfPoseRel;
                end
                prevScanDrPose = drPoseNow;

                [~, slamPoses] = scansAndPoses(slamObj);
                slamPoseNow = slamPoses(end, :);
            else
                if isempty(logSlam)
                    slamPoseNow = [nan, nan, nan];
                else
                    slamPoseNow = logSlam(end, :);
                end
            end

            logT(end+1, 1) = tNow;
            logTf(end+1, :) = tfPoseRel;
            logDr(end+1, :) = drPoseNow;
            logSlam(end+1, :) = slamPoseNow;
        end

        %% 5) Live plot updates
        if (tNow - lastPlotUpdate) >= cfg.plotUpdateInterval && isgraphics(axTraj)
            lastPlotUpdate = tNow;

            if ~isempty(logDr)
                set(hDr, 'XData', logDr(:,1), 'YData', logDr(:,2));
            end

            validTf = ~isempty(logTf) && any(all(isfinite(logTf), 2));
            if validTf
                idxTf = all(isfinite(logTf), 2);
                set(hTf, 'XData', logTf(idxTf,1), 'YData', logTf(idxTf,2));
            end

            validSlam = ~isempty(logSlam) && any(all(isfinite(logSlam), 2));
            if validSlam
                idxSlam = all(isfinite(logSlam), 2);
                set(hSlam, 'XData', logSlam(idxSlam,1), 'YData', logSlam(idxSlam,2));
            end

            drawnow limitrate;
        end

        if tNow >= cfg.runDurationSec
            break;
        end

        pause(0.001);

        if ~isgraphics(figTraj)
            error('L7:UserClosedFigure', 'Trajectory figure was closed by user.');
        end
    end

catch ME
    clear imuSub tfSub scanSub
    if ~strcmp(ME.identifier, 'L7:UserClosedFigure')
        rethrow(ME)
    end
end

clear imuSub tfSub scanSub

%% Final outputs and comparison
acceptedScans = sum(logScanAccepted > 0);
disp(['Accepted LiDAR scans: ' num2str(acceptedScans) ' / ' num2str(numel(logScanAccepted))]);

figure('Name', 'LiDAR SLAM Occupancy Map', 'NumberTitle', 'off');
show(slamObj);
title('LiDAR SLAM Result');

if ~isempty(logT)
    figure('Name', 'Pose Comparison', 'NumberTitle', 'off');
    tiledlayout(2,1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    hold on; grid on;
    plot(logT, logDr(:,1), 'b-', 'LineWidth', 1.2);
    if any(all(isfinite(logTf),2)), plot(logT, logTf(:,1), 'r--', 'LineWidth', 1.2); end
    if any(all(isfinite(logSlam),2)), plot(logT, logSlam(:,1), 'k-', 'LineWidth', 1.2); end
    ylabel('x [m]');
    legend({'DR/Fused', 'TF Odom', 'SLAM'}, 'Location', 'best');
    title('X position over time');

    nexttile;
    hold on; grid on;
    plot(logT, logDr(:,2), 'b-', 'LineWidth', 1.2);
    if any(all(isfinite(logTf),2)), plot(logT, logTf(:,2), 'r--', 'LineWidth', 1.2); end
    if any(all(isfinite(logSlam),2)), plot(logT, logSlam(:,2), 'k-', 'LineWidth', 1.2); end
    xlabel('time [s]');
    ylabel('y [m]');
    title('Y position over time');
end

disp('Exercise 1 and Exercise 2 completed.');

%% Callbacks and helpers
function imuCallback(message)
    global imuMsg
    imuMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end

function scanCallback(message)
    global scanMsg
    scanMsg = message;
end

function tSec = readImuTimeSec(imuMessage)
    sec = NaN;
    nanosec = NaN;

    try
        sec = double(imuMessage.header.stamp.sec);
        nanosec = double(imuMessage.header.stamp.nanosec);
    catch
        try
            sec = double(imuMessage.Header.Stamp.Sec);
            nanosec = double(imuMessage.Header.Stamp.Nanosec);
        catch
            tSec = NaN;
            return;
        end
    end

    tSec = sec + 1e-9 * nanosec;
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

function scan = lidarScanFromRos(scanMessage, maxRange)
    % Parse ROS LaserScan fields in lower/upper-case variants.
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

    invalid = ~isfinite(ranges) | ranges <= 0;
    ranges(invalid) = maxRange;
    ranges = min(ranges, maxRange);

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
    a = mod(a + pi, 2 * pi) - pi;
end
