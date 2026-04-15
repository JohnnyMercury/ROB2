%% L7 Exercise 1: Dead Reckoning from IMU (ROS2)
% Estimates robot displacement using inertial dead reckoning and compares
% it against odometry extracted from /tf.

clear all
clc
close all

global imuMsg tfMsg
imuMsg = [];
tfMsg = [];

%% User configuration
cfg.rosDomainId = '30';
cfg.maxWaitSec = 120;
cfg.pollInterval = 0.5;
cfg.runDurationSec = 120;
cfg.plotUpdateInterval = 0.2;    % seconds, ~5 Hz GUI refresh
cfg.maxHistoryLength = 500;      % keep bounded history for fast plotting
cfg.accelDeadband = 0.08;        % m/s^2 noise floor for zero-motion rejection
cfg.visualizeRobotFromTF = true; % true: robot icon follows TF, marker follows DR

% Gravity compensation: true subtracts [0 0 9.81] after rotating IMU accel
% to world frame. Set false if your IMU already outputs gravity-free accel.
cfg.removeGravity = true;
cfg.gWorld = [0, 0, 9.81];

% Visualization toggles.
cfg.showRobotVisualizer = true;
cfg.showHeadingPlot = true;
cfg.showDistancePlot = true;

%% ROS2 setup
setenv('ROS_DOMAIN_ID', cfg.rosDomainId);
node = ros2node('/base_station_l7_e1_dead_reckoning');

% Wait for required topics before creating subscribers.
maxWaitSec = cfg.maxWaitSec;
pollInterval = cfg.pollInterval;
requiredTopics = ["/imu", "/tf"];
startTime = tic;

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
        % Ignore transient discovery errors during startup.
    end
    pause(pollInterval);
end

if toc(startTime) >= maxWaitSec
    error('Timeout waiting for /imu and /tf topics. Check TurtleBot bringup.');
end

%% Subscribers
imuSub = ros2subscriber(node, '/imu', @imuCallback);
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');
pause(0.5);

%% Wait for first IMU and TF samples
disp('Waiting for first IMU and TF messages...');
while isempty(imuMsg) || isempty(tfMsg)
    pause(0.01);
end

%% Dead-reckoning state
pWorld = [0, 0, 0];
vWorld = [0, 0, 0];
yawImuInit = [];

% Keep initial TF position as reference origin.
tfInitPose = [];

removeGravity = cfg.removeGravity;
gWorld = cfg.gWorld;

%% Logging buffers (preallocated for performance)
N = cfg.maxHistoryLength;
pDrHist = NaN(N, 3);
pTfHist = NaN(N, 3);
tHist = NaN(N, 1);
distanceErrHist = NaN(N, 1);
headingImuHist = NaN(N, 1);
headingTfHist = NaN(N, 1);
histIdx = 0;  % running sample count

showRobotVisualizer = cfg.showRobotVisualizer;
showHeadingPlot = cfg.showHeadingPlot;
showDistancePlot = cfg.showDistancePlot;

if showRobotVisualizer
    visualise = TurtleBotVisualise();
else
    visualise = [];
end

if showHeadingPlot
    hHead = plotHeadings();
else
    hHead = [];
end

if showDistancePlot
    hDist = plotDistance();
else
    hDist = [];
end

loopStart = tic;
tPrev = 0;
runDurationSec = cfg.runDurationSec;
lastImuStamp = '';

figure('Name', 'L7-E1 Dead Reckoning vs TF Odometry', 'NumberTitle', 'off');
ax = gca;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
title(ax, '2D Trajectory Comparison');
hLineDr = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5);
hLineTf = plot(ax, NaN, NaN, 'r--', 'LineWidth', 1.5);
legend(ax, {'Dead reckoning (IMU)', 'Odometry from /tf'}, 'Location', 'best');

disp('Running dead reckoning. Move TurtleBot to evaluate localization drift...');
lastPlotUpdate = -inf;

try
    while true
        % Integrate only once per new IMU sample to avoid over-integrating
        % the same message and creating false motion/drift.
        imuStamp = readImuStamp(imuMsg);
        if strcmp(imuStamp, lastImuStamp)
            pause(0.001);
            continue;
        end
        lastImuStamp = imuStamp;

        tNow = toc(loopStart);
        dt = max(tNow - tPrev, 1e-3);
        tPrev = tNow;

        % Read latest IMU orientation and acceleration (body frame).
        qRos = [imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w];
        qMatlab = [qRos(4), qRos(1), qRos(2), qRos(3)]; % [w x y z]
        yawImu = quatRosToYaw(qRos(1), qRos(2), qRos(3), qRos(4));
        if isempty(yawImuInit)
            yawImuInit = yawImu;
        end
        yawImuRel = wrapToPiLocal(yawImu - yawImuInit);

        aBody = [imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z];

        % Transform acceleration to world frame using quaternion rotation.
        aWorld = quatrotate(qMatlab, aBody);

        if removeGravity
            aWorld = aWorld - gWorld;
        end

        if norm(aWorld) < cfg.accelDeadband
            aWorld = [0, 0, 0];
        end

        % Dead reckoning integration:
        % p[k+1] = p[k] + v[k]*dt + 1/2*a[k]*dt^2
        % v[k+1] = v[k] + a[k]*dt
        pWorld = pWorld + vWorld * dt + 0.5 * aWorld * dt * dt;
        vWorld = vWorld + aWorld * dt;

        % Extract odometry pose from /tf.
        tfPose = extractOdomPoseFromTf(tfMsg);
        if ~isempty(tfPose)
            if isempty(tfInitPose)
                tfInitPose = tfPose;
            end
            tfPoseRel = tfPose - tfInitPose;
        else
            tfPoseRel = [nan, nan, nan];
        end

        % Log for plotting and post-analysis (circular buffer).
        histIdx = histIdx + 1;
        ci = mod(histIdx - 1, N) + 1;  % circular index 1..N

        pDrHist(ci, :) = pWorld;
        pTfHist(ci, :) = tfPoseRel;
        tHist(ci, 1) = tNow;

        if all(isfinite(tfPoseRel))
            headingTf = tfPoseRel(3);
            distanceErr = norm(pWorld(1:2) - tfPoseRel(1:2));
        else
            headingTf = nan;
            distanceErr = nan;
        end

        headingImuHist(ci, 1) = yawImuRel;
        headingTfHist(ci, 1) = headingTf;
        distanceErrHist(ci, 1) = distanceErr;

        % Throttle all visual updates to prevent UI backlog and multi-second lag.
        if (tNow - lastPlotUpdate) >= cfg.plotUpdateInterval
            lastPlotUpdate = tNow;

            % Build ordered view from circular buffer.
            if histIdx <= N
                oi = 1:histIdx;  % buffer not yet full
            else
                tail = mod(histIdx - 1, N) + 1;  % most recent write
                oi = [tail+1:N, 1:tail];          % oldest → newest
            end

            set(hLineDr, 'XData', pDrHist(oi,1), 'YData', pDrHist(oi,2));
            validTf = all(isfinite(pTfHist(oi,:)), 2);
            if any(validTf)
                set(hLineTf, 'XData', pTfHist(oi(validTf),1), 'YData', pTfHist(oi(validTf),2));
            end

            if showRobotVisualizer
                if cfg.visualizeRobotFromTF && all(isfinite(tfPoseRel))
                    visualise = updatePositionDesired(visualise, pWorld(1:2));
                    visualise = updatePose(visualise, tfPoseRel(1:2), tfPoseRel(3));
                else
                    visualise = updatePositionDesired(visualise, tfPoseRel(1:2));
                    visualise = updatePose(visualise, pWorld(1:2), yawImuRel);
                end
            end

            if showHeadingPlot
                hHead = plotHeadings(hHead, tHist(oi), headingTfHist(oi), headingImuHist(oi));
            end

            if showDistancePlot
                validErr = isfinite(distanceErrHist(oi));
                hDist = plotDistance(hDist, tHist(oi(validErr)), distanceErrHist(oi(validErr)));
            end

            drawnow limitrate;
        end

        if tNow >= runDurationSec
            break;
        end

        pause(0.001);

        if ~isgraphics(ax)
            error('L7E1:UserClosedFigure', 'Figure was closed by user.');
        end
    end

catch ME
    clear imuSub tfSub
    if ~strcmp(ME.identifier, 'L7E1:UserClosedFigure')
        rethrow(ME)
    end
end

% Ensure subscriptions are released after normal completion.
clear imuSub tfSub

%% Summary plot with position error over time
% Build final ordered view from circular buffer.
if histIdx <= N
    oi = 1:histIdx;
else
    tail = mod(histIdx - 1, N) + 1;
    oi = [tail+1:N, 1:tail];
end

validTf = all(isfinite(pTfHist(oi,:)), 2);
if any(validTf)
    dr2 = pDrHist(oi(validTf), 1:2);
    tf2 = pTfHist(oi(validTf), 1:2);
    err = vecnorm(dr2 - tf2, 2, 2);

    figure('Name', 'L7-E1 Position Error', 'NumberTitle', 'off');
    plot(tHist(oi(validTf)), err, 'k-', 'LineWidth', 1.5);
    grid on
    xlabel('time [s]');
    ylabel('||p_{DR} - p_{TF}|| [m]');
    title('Dead reckoning position error relative to /tf odometry');
end

disp('Exercise 1 completed.');

%% Callbacks and helpers
function imuCallback(message)
    global imuMsg
    imuMsg = message;
end

function tfCallback(message)
    global tfMsg
    tfMsg = message;
end

function pose = extractOdomPoseFromTf(tfMessage)
% Returns [x y yaw] from transform odom -> base_* if available.
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

function a = wrapToPiLocal(a)
    while a > pi
        a = a - 2 * pi;
    end
    while a < -pi
        a = a + 2 * pi;
    end
end

function stampStr = readImuStamp(imuMessage)
% Creates a comparable string timestamp from IMU header.
    sec = 0;
    nanosec = 0;

    try
        sec = double(imuMessage.header.stamp.sec);
        nanosec = double(imuMessage.header.stamp.nanosec);
    catch
        try
            sec = double(imuMessage.Header.Stamp.Sec);
            nanosec = double(imuMessage.Header.Stamp.Nanosec);
        catch
            stampStr = sprintf('fallback_%f', now);
            return;
        end
    end

    stampStr = sprintf('%d_%d', sec, nanosec);
end
