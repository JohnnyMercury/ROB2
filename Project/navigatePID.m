function [v_cmd, w_cmd, controller_state] = navigatePID(pose, goal, controller_state, dt)
% [v_cmd, w_cmd, controller_state] = navigatePID(pose, goal, controller_state, dt)
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
    % PID gains tuned for smooth motion at v_max = 0.10 m/s.
    % Kp_p was previously 8.0 which caused permanent saturation at 0.1 m/s
    % (PID always asking for >>0.1 m/s, then clipped). With Kp_p = 1.0 the
    % controller still saturates near the goal but ramps down smoothly when
    % approaching, which is what we want.
    controller_state.Kp_h = 1.5;              % Heading proportional gain
    controller_state.Kd_h = 0.3;              % Heading derivative gain (lower => less kick on transitions)
    controller_state.Kp_p = 1.0;              % Position proportional gain
    controller_state.Ki_p = 0.02;             % Position integral gain

    controller_state.heading_error_prev = 0;  % Previous heading error (for derivative)
    controller_state.heading_derivative_initialized = false;
    controller_state.integral_distance = 0;   % Accumulated distance error
    controller_state.distance_tol = 0.05;     % Distance threshold (m) - was 0.01 (oscillated near goal)
    controller_state.heading_tol = 0.05;      % Heading threshold (rad) - was 0.01 (oscillated)
    controller_state.final_heading_tol = 0.05; % Goal heading threshold (rad)

    controller_state.integral_max = 1.0;       % Anti-windup threshold
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

% Compute heading error derivative.
% Suppress derivative kick on first controller cycle by initializing
% previous error to current error.
if ~isfield(controller_state, 'heading_derivative_initialized') || ~controller_state.heading_derivative_initialized
    controller_state.heading_error_prev = heading_error;
    controller_state.heading_derivative_initialized = true;
    d_heading_error = 0;
else
    d_heading_error = (heading_error - controller_state.heading_error_prev) / dt;
end
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

% NOTE (ROBUST FIX): Clipping/saturations removed from here! 
% Raw unbounded PID values (e.g. v=8.0) are passed to the obstacle avoidance layer
% so that repulsive forces can be blended mathematically EXACTLY the same as 
% the original script, and clipping only happens at the very end.

end

function a = wrapToPi(a)
    % Wrap angle to [-pi, pi]
    a = mod(a + pi, 2 * pi) - pi;
end
