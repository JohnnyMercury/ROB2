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
    % APF parameters tuned to match TurtleBot3 burger (radius ~0.14 m).
    avoid_state.beta = 0.3;                 % Repulsive force strength
    avoid_state.d0 = 0.25;                  % APF activation distance (0.35 was too aggressive in corridors)
    avoid_state.front_back_angle = deg2rad(40);

    % Angular smoothing. 0.25 was too heavy (slow reaction => overshoot).
    % Higher alpha = more responsive to new commands.
    avoid_state.w_filter_alpha = 0.5;
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

% Read lidar once and reuse (was being read twice => potentially stale).
try
    scan_obj = rosReadLidarScan(scan_msg);
    ranges_full = double(scan_obj.Ranges(:));
    angles_full = double(scan_obj.Angles(:));
catch
    return;  % bad scan, keep nominal commands
end

valid_full = isfinite(ranges_full) & ...
    (ranges_full > avoid_state.min_valid_range) & ...
    (ranges_full < avoid_state.max_valid_range);
ranges_v = ranges_full(valid_full);
angles_v = angles_full(valid_full);

[front_min, left_min, right_min] = getScanSectorMinimumsCached(ranges_v, angles_v, ...
    deg2rad(35), deg2rad(25), deg2rad(100));

[f_rep_x, f_rep_y] = computeRepulsiveForcesCached(ranges_v, angles_v, ...
    avoid_state.beta, avoid_state.d0, avoid_state.front_back_angle);

% Break APF symmetry: dead-center obstacles can cancel lateral force.
% Threshold 0.01 was too tight (almost any tiny imbalance skipped the fix).
if front_min < avoid_state.d0 && abs(f_rep_y) < 0.05
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
% Reduced max angular velocity from 1.0 to 0.6 rad/s for smoother turns.
v_cmd = max(0.0, v_cmd);
w_cmd = max(min(w_cmd, 0.6), -0.6);

alpha = avoid_state.w_filter_alpha;
w_cmd = (1 - alpha) * avoid_state.w_prev + alpha * w_cmd;
avoid_state.w_prev = w_cmd;

% Final velocity cap. Raised from 0.1 to 0.15 m/s for less sluggish motion.
v_cmd = min(v_cmd, 0.15);

debug.avoid_mode = front_min < avoid_state.d0;
debug.repulsive_x = f_rep_x;
debug.repulsive_y = f_rep_y;

debug.front_min = front_min;
debug.left_min = left_min;
debug.right_min = right_min;

end

function [front_min, left_min, right_min] = getScanSectorMinimumsCached(ranges, angles, front_half_angle, side_inner_angle, side_outer_angle)
% Operates on already-filtered ranges/angles. Avoids re-reading scan.
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

function [f_rep_x, f_rep_y] = computeRepulsiveForcesCached(ranges, angles, beta, d0, front_back_angle)
% Operates on already-filtered ranges/angles. Avoids re-reading scan.
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
