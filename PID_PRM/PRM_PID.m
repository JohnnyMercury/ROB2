%% PRM + PID waypoint follower for TurtleBot (ROS2)
% Plans a path from start to goal on a binary occupancy map with PRM,
% then tracks each waypoint in sequence using a PID-style controller.

clear all
clc
close all

global pose scan
pose = [];
scan = [];

%% ROS2 setup
setenv('ROS_DOMAIN_ID', '30');
controlNode = ros2node('/base_station_prm_pid');

% Discovery guard: avoid subscribing before topics exist.
maxWaitSec = 120; % How long to wait for ROS2 topics (seconds)
pollInterval = 0.5; % how long to pause before checking again (seconds)
requiredTopics = ["/odom", "/scan"];
startTime = tic;

disp('Waiting for required ROS2 topics...');
while toc(startTime) < maxWaitSec
	try
		% evalc captures command-window output so we can parse topic names.
		topicOutput = evalc("ros2 topic list");
		% split(...) tokenizes whitespace/newlines into a simple string list.
		topics = string(split(string(topicOutput)));
		if all(ismember(requiredTopics, topics))
			disp(['Topics found after ' num2str(toc(startTime), '%.1f') ' seconds.']);
			break;
		end
	catch
		% Ignore transient discovery errors
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

% path is an N-by-2 list of waypoints [x y].
path = findpath(prm, startPoint, goalPoint);

% Retry with denser graph if first path attempt fails.
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
% Overlay selected route and endpoints on top of sampled roadmap.
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPoint(1), goalPoint(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Roadmap', 'Path', 'Start', 'Goal', 'Location', 'best');
title('PRM path used for waypoint tracking');

%% Live robot visualisation
plotScan = true;
visualise = TurtleBotVisualise();

%% PID parameters
% Heading loop (PD): points robot toward current waypoint.
Kp_h = 4.0;
Kd_h = 1.5;

% Distance loop (PI): pushes robot forward toward waypoint.
Kp_d = 1.2;
Ki_d = 0.05;

% Output saturations for safety and smoother behavior.
maxLinear = 0.12;
maxAngular = 1.2;

% Intermediate waypoints can be looser than final goal tolerance.
waypointTolerance = 0.08;
finalTolerance = 0.06;

% Controller memory terms.
headingErrorPrev = 0;
distanceIntegral = 0;
loopStart = tic;
tPrev = 0;

disp(['Path has ' num2str(size(path,1)) ' waypoints. Starting PID waypoint tracking...']);

try
	%% Main execution: iterate through PRM waypoints
	for i = 1:size(path,1)
		targetWaypoint = path(i, :);

		% Final waypoint uses stricter tolerance to finish near the goal.
		if i == size(path,1)
			activeTolerance = finalTolerance;
		else
			activeTolerance = waypointTolerance;
		end

		%% Inner control loop: run until current waypoint is reached
		while true
			if isempty(pose) || ~isstruct(pose)
				pause(0.001);
				continue;
			end

			t = toc(loopStart);
			% Protect derivative term against division-by-zero at startup.
			dt = max(t - tPrev, 1e-3);
			tPrev = t;

			% Current robot state from odometry callback.
			position = [pose.position.x, pose.position.y];
			heading = quatRosToYaw(pose.orientation);

			% Update visualization: robot pose + active target waypoint.
			visualise = updatePositionDesired(visualise, targetWaypoint);
			visualise = updatePose(visualise, position, heading);

			if plotScan && ~isempty(scan)
				% Convert scan from robot frame to world frame for plotting.
				cart = rosReadCartesian(scan);
				cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position;
				visualise = updateScan(visualise, cart);
			end

			% Position error in Cartesian coordinates.
			dx = targetWaypoint(1) - position(1);
			dy = targetWaypoint(2) - position(2);
			distanceError = hypot(dx, dy);

			% Waypoint reached -> stop briefly and move to next waypoint.
			if distanceError < activeTolerance
				cmdMsg.linear.x = 0.0;
				cmdMsg.angular.z = 0.0;
				send(cmdPub, cmdMsg);
				break;
			end

			% Heading target points directly to the active waypoint.
			headingDesired = atan2(dy, dx);
			% Wrap angle error to [-pi, pi] for shortest-turn behavior.
			headingError = wrapToPiLocal(headingDesired - heading);
			headingErrorDot = (headingError - headingErrorPrev) / dt;
			headingErrorPrev = headingError;

			% Integrate distance error with anti-windup clamping.
			distanceIntegral = distanceIntegral + distanceError * dt;
			distanceIntegral = min(max(distanceIntegral, -1.5), 1.5);

			% Control laws: PD for turning, PI for forward speed.
			angularVelocity = Kp_h * headingError + Kd_h * headingErrorDot;
			linearVelocity = Kp_d * distanceError + Ki_d * distanceIntegral;

			% Slow down linear speed when heading error is large.
			if abs(headingError) > 0.6
				linearVelocity = 0.03;
			end

			% Saturate outputs and publish Twist command.
			cmdMsg.linear.x = min(max(linearVelocity, -maxLinear), maxLinear);
			cmdMsg.angular.z = min(max(angularVelocity, -maxAngular), maxAngular);
			send(cmdPub, cmdMsg);

			drawnow limitrate;

			% Exit cleanly if user closes visualizer window.
			if ~isgraphics(visualise.fig)
				error('PRMPID:UserClosedFigure', 'The visualisation window was closed.');
			end
		end
	end

	% Final stop once all waypoints are reached.
	cmdMsg.linear.x = 0.0;
	cmdMsg.angular.z = 0.0;
	send(cmdPub, cmdMsg);
	disp('Goal reached through PRM waypoints.');

catch ME
	cmdMsg.linear.x = 0.0;
	cmdMsg.angular.z = 0.0;
	send(cmdPub, cmdMsg);

	% Release subscribers after stop.
	clear odomSub scanSub

	if ~strcmp(ME.identifier, 'PRMPID:UserClosedFigure')
		rethrow(ME)
	end
end

%% Callbacks and helper functions
function odomCallback(message)
	% Store latest odometry pose globally for control loop access.
	global pose
	pose = message.pose.pose;
end

function scanCallback(message)
	% Store latest laser scan globally for visualization.
	global scan
	scan = message;
end

function yaw = quatRosToYaw(q)
	% ROS geometry_msgs/Quaternion order is (x,y,z,w).
	x = q.x;
	y = q.y;
	z = q.z;
	w = q.w;

	siny_cosp = 2 * (w * z + x * y);
	cosy_cosp = 1 - 2 * (y * y + z * z);
	yaw = atan2(siny_cosp, cosy_cosp);
end

function a = wrapToPiLocal(a)
	% Normalize angle to [-pi, pi] using repeated wrap.
	while a > pi
		a = a - 2 * pi;
	end
	while a < -pi
		a = a + 2 * pi;
	end
end
