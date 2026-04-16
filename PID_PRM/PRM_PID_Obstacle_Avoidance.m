%% PRM + PID + Obstacle Avoidance waypoint follower for TurtleBot (ROS2)
% Flow:
% 1) Build occupancy map and generate waypoint path with PRM.
% 2) Follow each waypoint with PID-style control.
% 3) If LiDAR detects a close obstacle, temporarily override commands
%    with a local avoidance behavior, then return to waypoint tracking.

clear all
clc
close all

global pose scan
pose = [];
scan = [];

%% ROS2 setup
setenv('ROS_DOMAIN_ID', '30');
controlNode = ros2node('/base_station_prm_pid_obstacle_avoidance');

% Discovery guard: do not create control behavior before topics exist.
maxWaitSec = 120;
pollInterval = 0.5;
requiredTopics = ["/odom", "/scan"];
startTime = tic;

disp('Waiting for required ROS2 topics...');
while toc(startTime) < maxWaitSec
	try
		% Capture ROS2 CLI output so we can parse available topic names.
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
	error('Timeout waiting for topics. Check TurtleBot bringup.');
end

%% ROS2 subscribers and publisher
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback);
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort');
pause(0.5);

cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
cmdMsg = ros2message('geometry_msgs/Twist');

%% Wait for first odometry sample
disp('Waiting for first odometry sample...');
while isempty(pose) || ~isstruct(pose)
	pause(0.01);
end

%% Build planning map (same scale as PRM example)
mapWidth = 4;
mapHeight = 4;
resolution = 50;
robotInflationRadius = 0.07;

simplegrid = binaryOccupancyMap(mapWidth, mapHeight, resolution); % Map base

% Obstacles are inserted in world coordinates [x y] with occupancy blocks. Ones are the size (starts from bottom right corner)
setOccupancy(simplegrid, [-0.30 0.3], ...
    ones(round(0.60*resolution), round(0.60*resolution)), 'world');
setOccupancy(simplegrid, [0.62 -0.4], ...
    ones(round(0.60*resolution), round(0.60*resolution)), 'world');

setOccupancy(simplegrid, [0.62 0.98], ...
    ones(round(0.60*resolution), round(0.60*resolution)), 'world');
setOccupancy(simplegrid, [1.2 0.0], ...
    ones(round(1.60*resolution), round(0.60*resolution)), 'world');

inflate(simplegrid, robotInflationRadius); % Inflate objects 

%% Define start and goal in world coordinates (meters)
startPoint = [0.0 0.0];
goalPoint = [1.5 3.0];

%% PRM planning
rng(1);
prm = mobileRobotPRM(simplegrid);
% Graph density controls: more nodes/connections improve path finding, but cost more time.
prm.NumNodes = 300;
prm.ConnectionDistance = 0.35;

% PRM output is an N-by-2 list of [x y] waypoints.
path = findpath(prm, startPoint, goalPoint);

% Retry with denser graph if first solve fails.
if isempty(path)
	prm.NumNodes = 250;
	prm.ConnectionDistance = 0.45;
	path = findpath(prm, startPoint, goalPoint);
end

if isempty(path)
	error('PRM could not find a path from start to goal.');
end

%% Show planned path
figure('Name', 'PRM Plan', 'NumberTitle', 'off');
show(prm);
hold on;
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPoint(1), goalPoint(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Roadmap', 'Path', 'Start', 'Goal', 'Location', 'best');
title('PRM path used for waypoint tracking');

%% Live robot visualization
plotScan = true;
visualise = TurtleBotVisualise();

%% PID parameters (waypoint tracking)
% Heading loop (PD): rotate toward active waypoint.
Kp_h = 4.0;
Kd_h = 1.5;

% Distance loop (PI): drive forward toward active waypoint.
Kp_d = 1.2;
Ki_d = 0.05;

% Command saturations.
maxLinear = 0.12;
maxAngular = 1.2;

% Waypoint acceptance radii.
waypointTolerance = 0.08;
finalTolerance = 0.06;

%% Local obstacle avoidance parameters (LiDAR)
% Hysteresis thresholds prevent rapid switching between modes.
avoidStartDistance = 0.30;   % Enter avoidance when obstacle is closer than this
avoidClearDistance = 0.40;   % Return to PID when front is clear beyond this

% Sector definitions around robot heading (radians).
frontHalfAngle = deg2rad(25);
sideInnerAngle = deg2rad(25);
sideOuterAngle = deg2rad(100);

% Avoidance motion commands.
avoidLinear = 0.03;
avoidTurnGain = 1.1;

%% Controller memory/state
headingErrorPrev = 0;
distanceIntegral = 0;
loopStart = tic;
tPrev = 0;

% State flag: true means reactive avoidance currently owns control output.
avoidMode = false;

disp(['Path has ' num2str(size(path,1)) ' waypoints. Starting tracking with obstacle avoidance...']);

try
	%% Main execution: iterate through PRM waypoints
	for i = 1:size(path,1)
		targetWaypoint = path(i, :);

		if i == size(path,1)
			activeTolerance = finalTolerance;
		else
			activeTolerance = waypointTolerance;
		end

		%% Inner control loop for current waypoint
		while true
			if isempty(pose) || ~isstruct(pose)
				pause(0.001);
				continue;
			end

			t = toc(loopStart);
			dt = max(t - tPrev, 1e-3);
			tPrev = t;

			% Current robot state from latest odometry.
			position = [pose.position.x, pose.position.y];
			heading = quatRosToYaw(pose.orientation);

			% Always visualize current waypoint and robot state.
			visualise = updatePositionDesired(visualise, targetWaypoint);
			visualise = updatePose(visualise, position, heading);

			if plotScan && ~isempty(scan)
				cart = rosReadCartesian(scan);
				cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position;
				visualise = updateScan(visualise, cart);
			end

			% Waypoint error in Cartesian coordinates.
			dx = targetWaypoint(1) - position(1);
			dy = targetWaypoint(2) - position(2);
			distanceError = hypot(dx, dy);

			% If waypoint reached, command stop and move to next waypoint.
			if distanceError < activeTolerance
				cmdMsg.linear.x = 0.0;
				cmdMsg.angular.z = 0.0;
				send(cmdPub, cmdMsg);

				% Reset controller memory at each waypoint transition.
				distanceIntegral = 0;
				headingErrorPrev = 0;
				break;
			end

			%% LiDAR sector analysis for obstacle avoidance decision
			[frontMin, leftMin, rightMin] = getScanSectorMinimums(scan, frontHalfAngle, sideInnerAngle, sideOuterAngle);

			% Enter avoidance when obstacle is too close in front.
			if frontMin < avoidStartDistance
				avoidMode = true;
			end

			% Exit avoidance only when front clears with margin (hysteresis).
			if avoidMode && frontMin > avoidClearDistance
				avoidMode = false;
			end

			if avoidMode
				%% Reactive obstacle avoidance command
				% Turn away from the closer side. If left is more blocked,
				% turn right (negative omega), and vice versa.
				sideBias = rightMin - leftMin;
				angularVelocity = avoidTurnGain * sign(sideBias);
				if angularVelocity == 0
					angularVelocity = avoidTurnGain; % default turn direction
				end

				% Reduce forward speed when object is very close.
				if frontMin < 0.20
					linearVelocity = 0.0;
				else
					linearVelocity = avoidLinear;
				end
			else
				%% PID waypoint tracking command
				headingDesired = atan2(dy, dx);
				headingError = wrapToPiLocal(headingDesired - heading);
				headingErrorDot = (headingError - headingErrorPrev) / dt;
				headingErrorPrev = headingError;

				% PI integral with anti-windup clamping.
				distanceIntegral = distanceIntegral + distanceError * dt;
				distanceIntegral = min(max(distanceIntegral, -1.5), 1.5);

				angularVelocity = Kp_h * headingError + Kd_h * headingErrorDot;
				linearVelocity = Kp_d * distanceError + Ki_d * distanceIntegral;

				% Turn-first behavior when heading error is large.
				if abs(headingError) > 0.6
					linearVelocity = 0.03;
				end
			end

			%% Saturate and publish velocity command
			cmdMsg.linear.x = min(max(linearVelocity, -maxLinear), maxLinear);
			cmdMsg.angular.z = min(max(angularVelocity, -maxAngular), maxAngular);
			send(cmdPub, cmdMsg);

			drawnow limitrate;

			if ~isgraphics(visualise.fig)
				error('PRMPIDOA:UserClosedFigure', 'The visualisation window was closed.');
			end
		end
	end

	% Goal reached: ensure robot is fully stopped.
	cmdMsg.linear.x = 0.0;
	cmdMsg.angular.z = 0.0;
	send(cmdPub, cmdMsg);
	disp('Goal reached through PRM waypoints (with obstacle avoidance active).');

catch ME
	% Safety stop on any exit path.
	cmdMsg.linear.x = 0.0;
	cmdMsg.angular.z = 0.0;
	send(cmdPub, cmdMsg);

	clear odomSub scanSub

	if ~strcmp(ME.identifier, 'PRMPIDOA:UserClosedFigure')
		rethrow(ME)
	end
end

%% Callbacks and helper functions
function odomCallback(message)
	% Store latest odometry pose for control loop access.
	global pose
	pose = message.pose.pose;
end

function scanCallback(message)
	% Store latest LiDAR scan for obstacle checks and plotting.
	global scan
	scan = message;
end

function yaw = quatRosToYaw(q)
	% ROS quaternion is (x,y,z,w); convert to yaw angle.
	x = q.x;
	y = q.y;
	z = q.z;
	w = q.w;

	siny_cosp = 2 * (w * z + x * y);
	cosy_cosp = 1 - 2 * (y * y + z * z);
	yaw = atan2(siny_cosp, cosy_cosp);
end

function a = wrapToPiLocal(a)
	% Normalize angle into [-pi, pi].
	while a > pi
		a = a - 2 * pi;
	end
	while a < -pi
		a = a + 2 * pi;
	end
end

function [frontMin, leftMin, rightMin] = getScanSectorMinimums(scanMsg, frontHalfAngle, sideInnerAngle, sideOuterAngle)
	% Convert ROS scan message to lidarScan object.
	lidarObj = rosReadLidarScan(scanMsg);
	ranges = lidarObj.Ranges;
	angles = lidarObj.Angles;

	% Keep only finite positive returns.
	valid = isfinite(ranges) & ranges > 0;
	ranges = ranges(valid);
	angles = angles(valid);

	if isempty(ranges)
		% No valid returns -> treat as clear space.
		frontMin = inf;
		leftMin = inf;
		rightMin = inf;
		return;
	end

	% Sector masks in robot frame (0 rad is forward).
	frontMask = abs(angles) <= frontHalfAngle;
	leftMask = angles >= sideInnerAngle & angles <= sideOuterAngle;
	rightMask = angles <= -sideInnerAngle & angles >= -sideOuterAngle;

	frontMin = maskedMin(ranges, frontMask);
	leftMin = maskedMin(ranges, leftMask);
	rightMin = maskedMin(ranges, rightMask);
end

function m = maskedMin(values, mask)
	if any(mask)
		m = min(values(mask));
	else
		m = inf;
	end
end
