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
cfg.maxImuDtSec = 0.1;           % clamp dt for robustness to delayed samples
cfg.firstMsgTimeoutSec = 20;     % timeout waiting for first IMU/TF messages
cfg.biasCalibSec = 2.0;          % initial stationary window for accel bias estimation
cfg.zeroVelocityOnLowAccel = true;
cfg.accelLpfTauSec = 0.35;       % low-pass time constant for acceleration smoothing
cfg.gyroDeadband = 0.03;         % rad/s threshold for stationary detection
cfg.stationaryHoldSec = 0.25;    % require low accel/gyro for this long before zeroing
cfg.planarMotionOnly = true;     % TurtleBot operates in 2D; reject vertical integration
cfg.useYawOnlyAccelTransform = true; % avoids roll/pitch coupling from IMU tilt
cfg.enableSimpleFusion = true;   % fuse IMU integration with /tf odometry
cfg.fusionPosTauSec = 1.2;       % smaller => stronger position correction to /tf
cfg.fusionYawTauSec = 0.8;       % smaller => stronger heading correction to /tf
cfg.fusionVelFromPosErrGain = 0.15; % velocity correction from position innovation
cfg.showTfRobotInVisualizer = false; % true draws second robot icon from /tf
cfg.showTfTrajectoryInFigure = false; % true draws /tf trajectory in 2D comparison figure

% Gravity compensation: true subtracts [0 0 9.81] after rotating IMU accel
% to world frame. Set false if your IMU already outputs gravity-free accel.
cfg.removeGravity = true;
cfg.gWorld = [0, 0, 9.81];

% Visualization toggles.
cfg.showRobotVisualizer = true;

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
    error('Timeout waiting for /imu and /tf topics. Check TurtleBot bringup.');
end

%% Subscribers
imuSub = ros2subscriber(node, '/imu', @imuCallback);
tfSub = ros2subscriber(node, '/tf', @tfCallback, 'Reliability', 'besteffort');
pause(0.5);

%% Wait for first IMU and TF samples
disp('Waiting for first IMU and TF messages...');
firstMsgStart = tic;
while isempty(imuMsg) || isempty(tfMsg)
    if toc(firstMsgStart) > cfg.firstMsgTimeoutSec
        error('Timeout waiting for first /imu and /tf messages.');
    end
    pause(0.01);
end

%% Dead-reckoning state
pWorld = [0, 0, 0];
vWorld = [0, 0, 0];
yawImuInit = [];
imuTimePrev = NaN;

% Bias estimate in world frame (computed during initial stationary period).
biasWorld = [0, 0, 0];
biasAccum = [0, 0, 0];
biasCount = 0;
biasReady = false;
biasStartImuSec = NaN;
aWorldFilt = [0, 0, 0];
stationaryAccumSec = 0;

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

if showRobotVisualizer
    visualise = TurtleBotVisualise();
    if cfg.showTfRobotInVisualizer
        title(visualise.h_ax, 'Black: Fused Estimate (IMU + /tf), Blue: /tf Odometry');
    else
        title(visualise.h_ax, 'Black: Fused Estimate (IMU + /tf)');
    end
else
    visualise = [];
end

loopStart = tic;
runDurationSec = cfg.runDurationSec;

figure('Name', 'L7-E1 Dead Reckoning vs TF Odometry', 'NumberTitle', 'off');
ax = gca;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
title(ax, '2D Trajectory Comparison');
hLineDr = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5);
if cfg.showTfTrajectoryInFigure
    hLineTf = plot(ax, NaN, NaN, 'r--', 'LineWidth', 1.5);
    legend(ax, {'Fused estimate (IMU + /tf)', 'Odometry from /tf'}, 'Location', 'best');
else
    hLineTf = [];
    legend(ax, {'Fused estimate (IMU + /tf)'}, 'Location', 'best');
end

disp('Running dead reckoning. Move TurtleBot to evaluate localization drift...');
lastPlotUpdate = -inf;

try
    while true
        % Integrate once per new IMU timestamp.
        imuTimeNow = readImuTimeSec(imuMsg);
        if ~isfinite(imuTimeNow)
            pause(0.001);
            continue;
        end

        if ~isnan(imuTimePrev)
            dt = imuTimeNow - imuTimePrev;
        else
            dt = NaN;
        end
        imuTimePrev = imuTimeNow;

        if ~isfinite(dt) || dt <= 0
            pause(0.001);
            continue;
        end

        dt = min(dt, cfg.maxImuDtSec);
        tNow = toc(loopStart);

        % Read latest IMU orientation and acceleration (body frame).
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
        if removeGravity
            % Estimate gravity in IMU/body frame from full orientation and
            % remove it before any planar projection.
            gBody = quatrotate(quatconj(qMatlab), gWorld);
            aBodyLin = aBody - gBody;
        end

        % Transform acceleration to world frame.
        % For planar TurtleBot motion, use yaw-only mapping to avoid
        % roll/pitch tilt injecting false x/y acceleration.
        if cfg.useYawOnlyAccelTransform
            c = cos(yawImuRel);
            s = sin(yawImuRel);
            aXYWorld = [c, -s; s, c] * aBodyLin(1:2)';
            aWorldRaw = [aXYWorld(1), aXYWorld(2), 0];
        else
            aWorldRaw = quatrotate(qMatlab, aBodyLin);
        end

        % Estimate acceleration bias while robot is initially stationary.
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

            pause(0.001);
            continue;
        end

        aWorld = aWorldRaw - biasWorld;

        % First-order LPF on acceleration to suppress high-frequency IMU noise.
        alpha = dt / (cfg.accelLpfTauSec + dt);
        aWorldFilt = aWorldFilt + alpha * (aWorld - aWorldFilt);

        aWorldUse = aWorldFilt;
        isLowAccel = norm(aWorldUse) < cfg.accelDeadband;
        isLowGyro = norm(wBody) < cfg.gyroDeadband;
        if isLowAccel && isLowGyro
            stationaryAccumSec = stationaryAccumSec + dt;
        else
            stationaryAccumSec = 0;
        end

        isStationary = stationaryAccumSec >= cfg.stationaryHoldSec;
        if isStationary
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

        % Dead reckoning integration:
        % p[k+1] = p[k] + v[k]*dt + 1/2*a[k]*dt^2
        % v[k+1] = v[k] + a[k]*dt
        pWorld = pWorld + vWorld * dt + 0.5 * aWorldUse * dt * dt;
        vWorld = vWorld + aWorldUse * dt;

        % Extract odometry pose from /tf.
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

        % Simple complementary fusion:
        % IMU provides prediction, /tf provides low-frequency correction.
        if cfg.enableSimpleFusion && all(isfinite(tfPoseRel))
            kPos = dt / (cfg.fusionPosTauSec + dt);
            kYaw = dt / (cfg.fusionYawTauSec + dt);

            posErr = tfPoseRel(1:2) - pWorld(1:2);
            pWorld(1:2) = pWorld(1:2) + kPos * posErr;
            yawErr = wrapToPiLocal(tfPoseRel(3) - yawImuRel);
            yawImuRel = wrapToPiLocal(yawImuRel + kYaw * yawErr);

            vWorld(1:2) = vWorld(1:2) + cfg.fusionVelFromPosErrGain * (kPos / max(dt, 1e-3)) * posErr;
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
            if cfg.showTfTrajectoryInFigure
                validTf = all(isfinite(pTfHist(oi,:)), 2);
                if any(validTf)
                    set(hLineTf, 'XData', pTfHist(oi(validTf),1), 'YData', pTfHist(oi(validTf),2));
                end
            end

            if showRobotVisualizer
                if cfg.showTfRobotInVisualizer && all(isfinite(tfPoseRel))
                    visualise = updateComparison(visualise, pWorld(1:2), yawImuRel, tfPoseRel(1:2), tfPoseRel(3));
                else
                    visualise = updatePose(visualise, pWorld(1:2), yawImuRel);
                end
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

if cfg.enableSimpleFusion
    disp('Exercise 1 completed with simple IMU + /tf fusion.');
else
    disp('Exercise 1 completed.');
end

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
    a = mod(a + pi, 2 * pi) - pi;
end

function tSec = readImuTimeSec(imuMessage)
% Returns IMU ROS timestamp in seconds.
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
