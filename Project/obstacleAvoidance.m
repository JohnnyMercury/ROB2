function [v_cmd, w_cmd, avoid_state, debug] = obstacleAvoidance(v_in, w_in, scan_msg, avoid_state)
% obstacleAvoidance Reactive LiDAR safety layer for velocity commands.
% Applies front-stop and steer-away behavior with hysteresis.
%
% INPUTS:
%   v_in, w_in  : nominal commands from goal-tracking controller
%   scan_msg    : ROS2 LaserScan message
%   avoid_state : persistent struct (pass [] on first call)
%
% OUTPUTS:
%   v_cmd, w_cmd: safe commands after obstacle arbitration
%   avoid_state : updated persistent state
%   debug       : struct with front/left/right minimum ranges and mode flag

if nargin < 4 || isempty(avoid_state)
    avoid_state.avoid_mode = false;
    avoid_state.mode_enter_time = -inf;
    avoid_state.turn_dir = 1;

    % Distances in meters.
    avoid_state.avoid_start_distance = 0.40;   % Enter avoidance at this distance
    avoid_state.avoid_clear_distance = 0.80;   % Require this much clearance to exit (wider hysteresis)
    avoid_state.stop_distance = 0.32;
    avoid_state.forward_resume_distance = 0.75;

    % Commitment time to avoid ping-pong with global goal controller.
    avoid_state.min_avoid_hold_sec = 1.2;

    % LiDAR sector limits around forward axis.
    avoid_state.front_half_angle = deg2rad(35);
    avoid_state.side_inner_angle = deg2rad(25);
    avoid_state.side_outer_angle = deg2rad(100);

    % Override commands while avoiding.
    avoid_state.avoid_linear = 0.05;
    avoid_state.avoid_turn_gain = 1.2;
    avoid_state.side_bias_deadband = 0.05;

    % Valid scan limits.
    avoid_state.min_valid_range = 0.05;
    avoid_state.max_valid_range = 6.0;
end

v_cmd = v_in;
w_cmd = w_in;

debug.front_min = inf;
debug.left_min = inf;
debug.right_min = inf;
debug.avoid_mode = avoid_state.avoid_mode;

if isempty(scan_msg)
    return;
end

[front_min, left_min, right_min] = getScanSectorMinimums(scan_msg, ...
    avoid_state.front_half_angle, avoid_state.side_inner_angle, avoid_state.side_outer_angle, ...
    avoid_state.min_valid_range, avoid_state.max_valid_range);

% Enter avoidance when obstacle is close ahead.
if front_min < avoid_state.avoid_start_distance
    if ~avoid_state.avoid_mode
        side_bias = right_min - left_min;
        if abs(side_bias) < avoid_state.side_bias_deadband
            avoid_state.turn_dir = 1;
        else
            avoid_state.turn_dir = sign(side_bias);
        end
        avoid_state.mode_enter_time = now * 86400;
    end
    avoid_state.avoid_mode = true;
end

if avoid_state.avoid_mode
    hold_elapsed = (now * 86400) - avoid_state.mode_enter_time;

    % Exit only when both conditions are met:
    % 1) minimum hold time elapsed, and
    % 2) front is clearly open.
    if hold_elapsed >= avoid_state.min_avoid_hold_sec && front_min > avoid_state.avoid_clear_distance
        avoid_state.avoid_mode = false;
    end
end

if avoid_state.avoid_mode
    w_cmd = avoid_state.avoid_turn_gain * avoid_state.turn_dir;

    if front_min < avoid_state.stop_distance
        v_cmd = 0.0;
    elseif front_min < avoid_state.forward_resume_distance
        v_cmd = 0.0;
    else
        v_cmd = min(v_in, avoid_state.avoid_linear);
    end
end

debug.front_min = front_min;
debug.left_min = left_min;
debug.right_min = right_min;
debug.avoid_mode = avoid_state.avoid_mode;

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
