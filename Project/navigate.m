function [v_cmd, w_cmd, controller_state] = navigate(pose, goal, controller_state, dt)
% [v_cmd, w_cmd, controller_state] = navigate(pose, goal, controller_state, dt)
%
% PID-based navigation controller for TurtleBot unicycle model.
% Drives robot from current pose to goal position (2D: x, y), and optionally
% aligns to goal heading when goal has a third element.
%
% INPUT:
%   pose: [x; y; theta] current robot pose in world frame (m, m, rad)
%   goal: [x_goal; y_goal] or [x_goal; y_goal; theta_goal]
%   controller_state: struct with PID parameters and error history
%   dt: timestep (s)
%
% OUTPUT:
%   v_cmd: linear velocity command (m/s), saturated
%   w_cmd: angular velocity command (rad/s), saturated
%   controller_state: updated struct with integrated errors
%
% CONTROL LAWS:
%   Heading PD: w = Kp_h * err_heading + Kd_h * d(err_heading)/dt
%   Position PI: v = Kp_p * err_distance + Ki_p * integral(err_distance)

% Initialize controller state on first call
if nargin < 3 || isempty(controller_state)
    controller_state.Kp_h = 4.0;              % Heading proportional gain
    controller_state.Kd_h = 2.0;              % Heading derivative gain
    controller_state.Kp_p = 8.0;              % Position proportional gain
    controller_state.Ki_p = 0.02;             % Position integral gain
    
    controller_state.heading_error_prev = 0;  % Previous heading error (for derivative)
    controller_state.integral_distance = 0;   % Accumulated distance error
    controller_state.distance_tol = 0.05;     % Distance threshold (m)
    controller_state.heading_tol = 0.05;      % Heading threshold (rad)
    controller_state.final_heading_tol = 0.08; % Goal heading threshold (rad)
    
    % Saturation limits
    controller_state.v_max = 0.25;             % Max linear velocity (m/s)
    controller_state.w_max = 1.5;              % Max angular velocity (rad/s)
    controller_state.integral_max = 1.0;       % Anti-windup threshold
    
    % Speed reduction when aligned poorly
    controller_state.large_angle_thresh = pi / 3;  % 60 degrees
    controller_state.slow_speed_factor = 0.1;
end

if numel(goal) ~= 2 && numel(goal) ~= 3
    error('Goal must be [x; y] or [x; y; theta].');
end
goal = goal(:);

% Compute distance error
dx = goal(1) - pose(1);
dy = goal(2) - pose(2);
distance_error = hypot(dx, dy);

% Compute desired heading (point toward goal)
desired_heading = atan2(dy, dx);

% If close to goal and goal heading is provided, switch to heading alignment mode.
if numel(goal) == 3 && distance_error < controller_state.distance_tol
    desired_heading = wrapToPi(goal(3));
end

% Wrap heading error to [-pi, pi]
heading_error = wrapToPi(desired_heading - pose(3));

% Compute heading error derivative
d_heading_error = (heading_error - controller_state.heading_error_prev) / dt;
controller_state.heading_error_prev = heading_error;

% Accumulate distance error for integral term
controller_state.integral_distance = controller_state.integral_distance + distance_error * dt;

% Anti-windup: reset integral if too large
if controller_state.integral_distance > controller_state.integral_max
    controller_state.integral_distance = controller_state.integral_max;
elseif controller_state.integral_distance < -controller_state.integral_max
    controller_state.integral_distance = -controller_state.integral_max;
end

% Zero error if within tolerance (dead band)
if distance_error < controller_state.distance_tol
    distance_error = 0;
    controller_state.integral_distance = 0;
end

if abs(heading_error) < controller_state.heading_tol
    heading_error = 0;
    d_heading_error = 0;
end

% Compute PID commands
v_cmd = controller_state.Kp_p * distance_error + controller_state.Ki_p * controller_state.integral_distance;
w_cmd = controller_state.Kp_h * heading_error + controller_state.Kd_h * d_heading_error;

% In heading-alignment mode, hold position and rotate in place.
if numel(goal) == 3 && distance_error < controller_state.distance_tol
    v_cmd = 0;
    if abs(heading_error) < controller_state.final_heading_tol
        w_cmd = 0;
    end
end

% Saturate commands
v_cmd = max(min(v_cmd, controller_state.v_max), -controller_state.v_max);
w_cmd = max(min(w_cmd, controller_state.w_max), -controller_state.w_max);

% Reduce forward speed if heading error is large (avoid sharp turns)
if abs(heading_error) > controller_state.large_angle_thresh
    v_cmd = controller_state.slow_speed_factor * v_cmd;
end

end

function a = wrapToPi(a)
    % Wrap angle to [-pi, pi]
    a = mod(a + pi, 2 * pi) - pi;
end
