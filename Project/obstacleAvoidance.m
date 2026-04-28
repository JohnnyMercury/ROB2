function [v_cmd, w_cmd, avoid_state, debug] = obstacleAvoidance(v_in, w_in, scan_msg, avoid_state, heading_only_mode)
% obstacleAvoidance Reactive APF (repulsive-force) safety layer.
% Blends nominal PID commands with LiDAR-derived repulsive forces.
%
% INPUTS:
%   v_in, w_in  : nominal commands from goal-tracking controller
%   scan_msg    : ROS2 LaserScan message
%   avoid_state : persistent struct (pass [] on first call)
%
% OUTPUTS:
%   v_cmd, w_cmd: command after APF blending and safety clipping
%   avoid_state : updated persistent state
%   debug       : diagnostic values (mins + force components)

if nargin < 5 || isempty(heading_only_mode)
    heading_only_mode = false;
end

if nargin < 4 || isempty(avoid_state)
    % APF parameters (mirrors original PID-Obstacle_Avoidance approach).
    avoid_state.beta = 0.3;                 % Repulsive force strength
    avoid_state.d0 = 0.20;                  % APF activation distance, should not be lower than avoid_state.stop_distance (m)
    avoid_state.front_back_angle = deg2rad(40);

    % Lightweight angular smoothing for jitter reduction.
    avoid_state.w_filter_alpha = 0.25;
    avoid_state.w_prev = 0.0;

    % Valid scan limits.
    avoid_state.min_valid_range = 0.05;
    avoid_state.max_valid_range = 6.0;
end

v_cmd = v_in;
w_cmd = w_in;

debug.front_min = inf;
debug.left_min = inf;
debug.right_min = inf;
debug.repulsive_x = 0.0;
debug.repulsive_y = 0.0;
debug.avoid_mode = false;

if isempty(scan_msg)
    return;
end

[front_min, left_min, right_min] = getScanSectorMinimums(scan_msg, ...
    deg2rad(35), deg2rad(25), deg2rad(100), ...
    avoid_state.min_valid_range, avoid_state.max_valid_range);

[f_rep_x, f_rep_y] = computeRepulsiveForces(scan_msg, ...
    avoid_state.beta, avoid_state.d0, avoid_state.front_back_angle, ...
    avoid_state.min_valid_range, avoid_state.max_valid_range);

% Break APF symmetry: dead-center obstacles can cancel lateral force.
if front_min < avoid_state.d0 && abs(f_rep_y) < 0.01
    if left_min > right_min
        f_rep_y = 0.25;
    else
        f_rep_y = -0.25;
    end
end

% Apply the original APF-style blend directly to the PID commands.
% Keep the linear repulsive term as braking only, so it cannot force the
% robot to keep moving when the goal controller has already commanded v = 0.
front_brake = max(0.0, -f_rep_x);

if heading_only_mode
    % Final heading alignment near goal: rotate in place only.
    v_cmd = 0.0;
else
    v_cmd = v_in - front_brake;
end
if heading_only_mode
    % Ignore APF angular bias in final heading phase.
    w_cmd = w_in;
else
    w_cmd = w_in + f_rep_y;
end

% Avoid reversing and keep the command bounded.
v_cmd = max(0.0, v_cmd);
w_cmd = max(min(w_cmd, 1.0), -1.0);

alpha = avoid_state.w_filter_alpha;
w_cmd = (1 - alpha) * avoid_state.w_prev + alpha * w_cmd;
avoid_state.w_prev = w_cmd;

% Apply the identical clipping used manually in the original script.
v_cmd = min(v_cmd, 0.1);

debug.avoid_mode = front_min < avoid_state.d0;
debug.repulsive_x = f_rep_x;
debug.repulsive_y = f_rep_y;

debug.front_min = front_min;
debug.left_min = left_min;
debug.right_min = right_min;

end

function [front_min, left_min, right_min] = getScanSectorMinimums(scan_msg, front_half_angle, side_inner_angle, side_outer_angle, min_valid_range, max_valid_range)
scan_obj = rosReadLidarScan(scan_msg);
ranges = double(scan_obj.Ranges(:));
angles = double(scan_obj.Angles(:));

valid = isfinite(ranges) & (ranges > min_valid_range) & (ranges < max_valid_range);
ranges = ranges(valid);
angles = angles(valid);

if isempty(ranges)
    front_min = inf;
    left_min = inf;
    right_min = inf;
    return;
end

front_mask = abs(angles) <= front_half_angle;
left_mask = angles >= side_inner_angle & angles <= side_outer_angle;
right_mask = angles <= -side_inner_angle & angles >= -side_outer_angle;

front_min = maskedMin(ranges, front_mask);
left_min = maskedMin(ranges, left_mask);
right_min = maskedMin(ranges, right_mask);
end

function m = maskedMin(values, mask)
if any(mask)
    m = min(values(mask));
else
    m = inf;
end
end

function [f_rep_x, f_rep_y] = computeRepulsiveForces(scan_msg, beta, d0, front_back_angle, min_valid_range, max_valid_range)
scan_obj = rosReadLidarScan(scan_msg);
ranges = double(scan_obj.Ranges(:));
angles = double(scan_obj.Angles(:));

valid = isfinite(ranges) & (ranges > min_valid_range) & (ranges < max_valid_range);
ranges = ranges(valid);
angles = angles(valid);

if isempty(ranges)
    f_rep_x = 0.0;
    f_rep_y = 0.0;
    return;
end

% Match original APF behavior: use front and back rays only.
front_back_mask = (angles >= -front_back_angle & angles <= front_back_angle) | ...
                  (angles >= pi - front_back_angle | angles <= -pi + front_back_angle);

ranges = ranges(front_back_mask);
angles = angles(front_back_mask);

if isempty(ranges)
    f_rep_x = 0.0;
    f_rep_y = 0.0;
    return;
end

f_rep_x = 0.0;
f_rep_y = 0.0;

for i = 1:numel(ranges)
    di = ranges(i);
    if di > d0
        continue;
    end

    % APF magnitude used in original script.
    f_mag = 2 * beta / (di^2) * (1 / di - 1 / d0)^2;
    a = angles(i);

    % Repulsive force points away from obstacle ray.
    f_rep_x = f_rep_x - f_mag * cos(a);
    f_rep_y = f_rep_y - f_mag * sin(a);
end
end
