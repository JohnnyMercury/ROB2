classdef TurtleBotVisualise
    properties
        fig
        h_ax  % Store axes handle to ensure plots go to correct figure

        robotBody
        arrow
        scale = 2 % define scaling factor for visualization

        old_position = [0 0];

        h_robotBody
        h_axisX
        h_axisY
        h_axisZ
        h_cart
        h_posidion_desired
    end
    methods
        function obj = TurtleBotVisualise(~)
            %% Set up a figure for visualization
            obj.fig = figure('units', 'normalized');
            obj.h_ax = gca;  % Store axes handle for explicit plotting
            hold on
            grid on
            axis equal
            xlim([-2 2]);  % Define x-axis limits
            ylim([-2 2]);  % Define y-axis limits

            % Configure plot appearance
            set(obj.h_ax, 'fontsize', 10);
            xlabel(obj.h_ax, '$x$ [m]', 'interpreter', 'latex', 'fontsize', 10);
            ylabel(obj.h_ax, '$y$ [m]', 'interpreter', 'latex', 'fontsize', 10);
            set(obj.h_ax, 'TickLabelInterpreter', 'latex')

            %% Define robot dimensions for visualization
            TB_width = 0.14;        % Robot body width
            wheel_width = 0.02;     % Wheel width
            wheel_radius = 0.03;    % Wheel radius

            % Define the robot's body shape and orientation arrow
            obj.robotBody = obj.scale * ...
                [TB_width/2,               TB_width/2,                TB_width/2 - 2*wheel_radius, TB_width/2 - 2*wheel_radius, -TB_width/2, -TB_width/2, TB_width/2 - 2*wheel_radius, TB_width/2 - 2*wheel_radius;
                TB_width/2 + wheel_width, -TB_width/2 - wheel_width, -TB_width/2 - wheel_width,   -TB_width/2,                 -TB_width/2, TB_width/2,  TB_width/2,                  TB_width/2 + wheel_width];
            obj.arrow = obj.scale * ...
                [0,    0.08, 0.08, 0.1, 0.08,  0.08,  0;
                0.01, 0.01, 0.02, 0,   -0.02, -0.01, -0.01];
        end
        function obj = updatePose(obj, position, heading)
            %% Transform the robot's body and axes
            robotBodyTransformed = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.robotBody + position';
            axisX = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.arrow + position';
            axisY = [cos(heading + pi/2), -sin(heading + pi/2); sin(heading + pi/2), cos(heading + pi/2)] * obj.arrow + position';

            % Add trajectory line to the correct axes (explicitly specify axes handle)
            plot(obj.h_ax, [obj.old_position(1) position(1)], [obj.old_position(2) position(2)], 'k-', 'LineWidth', 0.5);
            obj.old_position = position;

            %% Update or create robot body and axes using set() for existing handles
            if isempty(obj.h_robotBody)
                % First run: create patches on the correct axes
                obj.h_robotBody = patch(obj.h_ax, robotBodyTransformed(1, :), robotBodyTransformed(2, :), 'black', 'EdgeColor', 'none');
            else
                % Update existing patch data (much faster than delete/recreate)
                set(obj.h_robotBody, 'XData', robotBodyTransformed(1, :), 'YData', robotBodyTransformed(2, :));
            end

            if isempty(obj.h_axisX)
                obj.h_axisX = patch(obj.h_ax, axisX(1, :), axisX(2, :), 'red', 'EdgeColor', 'none');
            else
                set(obj.h_axisX, 'XData', axisX(1, :), 'YData', axisX(2, :));
            end

            if isempty(obj.h_axisY)
                obj.h_axisY = patch(obj.h_ax, axisY(1, :), axisY(2, :), 'green', 'EdgeColor', 'none');
            else
                set(obj.h_axisY, 'XData', axisY(1, :), 'YData', axisY(2, :));
            end

            if isempty(obj.h_axisZ)
                obj.h_axisZ = rectangle(obj.h_ax, 'Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02], 'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'none');
            else
                set(obj.h_axisZ, 'Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02]);
            end
        end
        function obj = updatePositionDesired(obj, position_desired)
            %% Update or create desired position marker on the correct axes
            if isempty(obj.h_posidion_desired)
                % First run: create scatter plot on the correct axes
                obj.h_posidion_desired = scatter(obj.h_ax, position_desired(1), position_desired(2), 50, 'red', 'filled');
            else
                % Update existing scatter data (much faster than delete/recreate)
                set(obj.h_posidion_desired, 'XData', position_desired(1), 'YData', position_desired(2));
            end
        end
        function obj = updateScan(obj, cart)
            %% Update or create LiDAR scatter plot on the correct axes
            if isempty(obj.h_cart)
                % First run: create scatter plot on the correct axes
                obj.h_cart = scatter(obj.h_ax, cart(:,1), cart(:,2), 10, 'blue', 'filled');
            else
                % Update existing scatter data (much faster than delete/recreate)
                set(obj.h_cart, 'XData', cart(:,1), 'YData', cart(:,2));
            end
        end
    end
end