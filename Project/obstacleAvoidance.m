function [v_cmd, w_cmd, avoid_state, debug] = obstacleAvoidance(v_in, w_in, scan_msg, avoid_state)
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

if nargin < 4 || isempty(avoid_state)
    % APF parameters (mirrors original PID-Obstacle_Avoidance approach).
    avoid_state.beta = 0.5;                 % Repulsive force strength
    avoid_state.d0 = 0.50;                  % Activation distance (m)
    avoid_state.front_back_angle = deg2rad(10);

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

% Break APF symmetry: if obstacle is perfectly in front, forces cancel out
% and it just rocks back and forth. A tiny sideways nudge breaks the trap.
if f_rep_x < -0.1 && abs(f_rep_y) < 0.01
    f_rep_y = 0.3; % Arbitrary turn to escape dead-center minimum
end

% Match original directly: simply add forces to PID
v_cmd = v_in + f_rep_x;
w_cmd = w_in + f_rep_y;

% Apply the identical clipping used manually in the original script
v_cmd = max(min(v_cmd, 0.1), -0.1);
w_cmd = max(min(w_cmd, 1.0), -1.0);

debug.avoid_mode = (abs(f_rep_x) > 1e-3) || (abs(f_rep_y) > 1e-3);
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
