classdef plotTurtlebot
    properties
        fig
        h_ax
        figName = 'TurtleBot Visualize'
        xLim = [-2 2]
        yLim = [-2 2]

        robotBody
        arrow
        scale = 2

        old_position = [0 0]
        autoExpandMargin = 0.4
        currentXLim = [-2 2]
        currentYLim = [-2 2]
        h_traj

        h_robotBody
        h_axisX
        h_axisY
        h_axisZ
        h_cart
        h_posidion_desired
        h_particles % Handle for AMCL particles
    end

    methods
        function obj = plotTurtlebot(fig_name, x_lim, y_lim)
            if nargin >= 1 && ~isempty(fig_name)
                obj.figName = fig_name;
            end
            if nargin >= 2 && ~isempty(x_lim)
                obj.xLim = x_lim;
            end
            if nargin >= 3 && ~isempty(y_lim)
                obj.yLim = y_lim;
            end

            obj = setupFigure(obj);

            TB_width = 0.14;
            wheel_width = 0.02;
            wheel_radius = 0.03;

            obj.robotBody = obj.scale * ...
                [TB_width/2,               TB_width/2,                TB_width/2 - 2*wheel_radius, TB_width/2 - 2*wheel_radius, -TB_width/2, -TB_width/2, TB_width/2 - 2*wheel_radius, TB_width/2 - 2*wheel_radius;
                TB_width/2 + wheel_width, -TB_width/2 - wheel_width, -TB_width/2 - wheel_width,   -TB_width/2,                 -TB_width/2, TB_width/2,  TB_width/2,                  TB_width/2 + wheel_width];

            obj.arrow = obj.scale * ...
                [0,    0.08, 0.08, 0.1, 0.08,  0.08,  0;
                0.01, 0.01, 0.02, 0,   -0.02, -0.01, -0.01];
        end

        function obj = updatePose(obj, position, heading)
            obj = ensureAxes(obj);

            if nargin == 2
                pose = position;
                if isrow(pose)
                    pose = pose';
                end
                position = pose(1:2)';
                heading = pose(3);
            else
                if iscolumn(position)
                    position = position';
                end
            end

            R = [cos(heading), -sin(heading); sin(heading), cos(heading)];
            robotBodyTransformed = R * obj.robotBody + position';
            axisX = R * obj.arrow + position';
            axisY = [cos(heading + pi/2), -sin(heading + pi/2); sin(heading + pi/2), cos(heading + pi/2)] * obj.arrow + position';

            if isempty(obj.h_traj) || ~isvalid(obj.h_traj)
                obj.h_traj = plot(obj.h_ax, [obj.old_position(1) position(1)], [obj.old_position(2) position(2)], 'k-', 'LineWidth', 0.5);
            else
                set(obj.h_traj, 'XData', [obj.h_traj.XData position(1)], 'YData', [obj.h_traj.YData position(2)]);
            end
            obj.old_position = position;

            if isempty(obj.h_robotBody) || ~isvalid(obj.h_robotBody)
                obj.h_robotBody = patch(obj.h_ax, robotBodyTransformed(1, :), robotBodyTransformed(2, :), 'black', 'EdgeColor', 'none');
            else
                set(obj.h_robotBody, 'XData', robotBodyTransformed(1, :), 'YData', robotBodyTransformed(2, :));
            end

            if isempty(obj.h_axisX) || ~isvalid(obj.h_axisX)
                obj.h_axisX = patch(obj.h_ax, axisX(1, :), axisX(2, :), 'red', 'EdgeColor', 'none');
            else
                set(obj.h_axisX, 'XData', axisX(1, :), 'YData', axisX(2, :));
            end

            if isempty(obj.h_axisY) || ~isvalid(obj.h_axisY)
                obj.h_axisY = patch(obj.h_ax, axisY(1, :), axisY(2, :), 'green', 'EdgeColor', 'none');
            else
                set(obj.h_axisY, 'XData', axisY(1, :), 'YData', axisY(2, :));
            end

            if isempty(obj.h_axisZ) || ~isvalid(obj.h_axisZ)
                obj.h_axisZ = rectangle(obj.h_ax, 'Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02], 'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'none');
            else
                set(obj.h_axisZ, 'Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02]);
            end

            obj = maybeExpandAxes(obj, position, []);
        end

        function obj = updatePositionDesired(obj, position_desired)
            obj = ensureAxes(obj);
            if iscolumn(position_desired)
                position_desired = position_desired';
            end

            if isempty(obj.h_posidion_desired) || ~isvalid(obj.h_posidion_desired)
                obj.h_posidion_desired = scatter(obj.h_ax, position_desired(1), position_desired(2), 50, 'red', 'filled');
            else
                set(obj.h_posidion_desired, 'XData', position_desired(1), 'YData', position_desired(2));
            end

            obj = maybeExpandAxes(obj, position_desired, []);
        end

        function obj = updateScan(obj, cart)
            obj = ensureAxes(obj);

            if isempty(cart)
                return;
            end

            finiteMask = isfinite(cart(:,1)) & isfinite(cart(:,2));
            cart = cart(finiteMask, :);
            if isempty(cart)
                return;
            end

            if size(cart, 2) > 2
                cart = cart(:, 1:2);
            end

            if isempty(obj.h_cart) || ~isvalid(obj.h_cart)
                obj.h_cart = scatter(obj.h_ax, cart(:,1), cart(:,2), 12, [0 0.45 0.95], 'filled', 'MarkerFaceAlpha', 0.8, 'MarkerEdgeAlpha', 0.1);
            else
                set(obj.h_cart, 'XData', cart(:,1), 'YData', cart(:,2));
            end

            obj = maybeExpandAxes(obj, [], cart);
        end
        
        function obj = updateParticles(obj, particles)
            % Visualizes the AMCL probability distribution cloud
            obj = ensureAxes(obj);
            if isempty(particles)
                return;
            end

            if isempty(obj.h_particles) || ~isvalid(obj.h_particles)
                obj.h_particles = scatter(obj.h_ax, particles(:,1), particles(:,2), 6, [0.4660 0.6740 0.1880], 'filled', 'MarkerFaceAlpha', 0.5);
                % Keep particles below the robot graphics
                uistack(obj.h_particles, 'bottom'); 
            else
                set(obj.h_particles, 'XData', particles(:,1), 'YData', particles(:,2));
            end
        end

        function obj = setupFigure(obj)
            obj.fig = figure('Name', obj.figName, 'NumberTitle', 'off', 'units', 'normalized');
            obj.h_ax = gca;
            hold(obj.h_ax, 'on');
            grid(obj.h_ax, 'on');
            axis(obj.h_ax, 'equal');
            xlim(obj.h_ax, obj.xLim);
            ylim(obj.h_ax, obj.yLim);
            obj.currentXLim = obj.xLim;
            obj.currentYLim = obj.yLim;
            set(obj.h_ax, 'fontsize', 10);
            xlabel(obj.h_ax, '$x$ [m]', 'interpreter', 'latex', 'fontsize', 10);
            ylabel(obj.h_ax, '$y$ [m]', 'interpreter', 'latex', 'fontsize', 10);
            set(obj.h_ax, 'TickLabelInterpreter', 'latex');
        end

        function obj = ensureAxes(obj)
            if isempty(obj.fig) || ~isvalid(obj.fig) || isempty(obj.h_ax) || ~isvalid(obj.h_ax)
                error('plotTurtlebot:FigureClosed', 'Figure closed by user.');
            end
        end

        function obj = maybeExpandAxes(obj, pose_xy, scan_xy)
            x_min = obj.currentXLim(1);
            x_max = obj.currentXLim(2);
            y_min = obj.currentYLim(1);
            y_max = obj.currentYLim(2);

            if ~isempty(pose_xy)
                if iscolumn(pose_xy)
                    pose_xy = pose_xy';
                end
                x_min = min(x_min, pose_xy(1) - obj.autoExpandMargin);
                x_max = max(x_max, pose_xy(1) + obj.autoExpandMargin);
                y_min = min(y_min, pose_xy(2) - obj.autoExpandMargin);
                y_max = max(y_max, pose_xy(2) + obj.autoExpandMargin);
            end

            if ~isempty(scan_xy)
                x_min = min(x_min, min(scan_xy(:,1)) - 0.2);
                x_max = max(x_max, max(scan_xy(:,1)) + 0.2);
                y_min = min(y_min, min(scan_xy(:,2)) - 0.2);
                y_max = max(y_max, max(scan_xy(:,2)) + 0.2);
            end

            if x_min < obj.currentXLim(1) || x_max > obj.currentXLim(2) || ...
               y_min < obj.currentYLim(1) || y_max > obj.currentYLim(2)
                xlim(obj.h_ax, [x_min x_max]);
                ylim(obj.h_ax, [y_min y_max]);
                obj.currentXLim = [x_min x_max];
                obj.currentYLim = [y_min y_max];
            end
        end
    end
end