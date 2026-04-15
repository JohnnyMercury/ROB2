classdef TurtleBotVisualise
    properties
        fig
        h_ax

        robotBody
        arrow
        scale = 2

        old_position = [0 0];
        trajX = [];
        trajY = [];
        maxTrajPoints = 1500;

        h_robotBody
        h_axisX
        h_axisY
        h_axisZ

        h_robotBody2
        h_axisX2
        h_axisY2
        h_axisZ2

        h_cart
        h_posidion_desired
        h_traj
        h_traj2

        trajX2 = [];
        trajY2 = [];
    end
    methods
        function obj = TurtleBotVisualise(~)
            obj.fig = figure('units', 'normalized');
            obj.h_ax = gca;
            hold on
            grid on
            axis equal
            xlim([-3 3]);
            ylim([-3 3]);

            set(obj.h_ax, 'fontsize', 10);
            xlabel(obj.h_ax, '$x$ [m]', 'interpreter', 'latex', 'fontsize', 10);
            ylabel(obj.h_ax, '$y$ [m]', 'interpreter', 'latex', 'fontsize', 10);
            set(obj.h_ax, 'TickLabelInterpreter', 'latex')

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
            robotBodyTransformed = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.robotBody + position';
            axisX = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.arrow + position';
            axisY = [cos(heading + pi/2), -sin(heading + pi/2); sin(heading + pi/2), cos(heading + pi/2)] * obj.arrow + position';

            obj.trajX(end+1) = position(1);
            obj.trajY(end+1) = position(2);
            if numel(obj.trajX) > obj.maxTrajPoints
                obj.trajX = obj.trajX(end-obj.maxTrajPoints+1:end);
                obj.trajY = obj.trajY(end-obj.maxTrajPoints+1:end);
            end

            if isempty(obj.h_traj)
                obj.h_traj = plot(obj.h_ax, obj.trajX, obj.trajY, 'k-', 'LineWidth', 0.5);
            else
                set(obj.h_traj, 'XData', obj.trajX, 'YData', obj.trajY);
            end

            obj.old_position = position;

            if isempty(obj.h_robotBody)
                obj.h_robotBody = patch(obj.h_ax, robotBodyTransformed(1, :), robotBodyTransformed(2, :), 'black', 'EdgeColor', 'none');
            else
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

        function obj = updatePoseSecondary(obj, position, heading)
            robotBodyTransformed = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.robotBody + position';
            axisX = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.arrow + position';
            axisY = [cos(heading + pi/2), -sin(heading + pi/2); sin(heading + pi/2), cos(heading + pi/2)] * obj.arrow + position';

            obj.trajX2(end+1) = position(1);
            obj.trajY2(end+1) = position(2);
            if numel(obj.trajX2) > obj.maxTrajPoints
                obj.trajX2 = obj.trajX2(end-obj.maxTrajPoints+1:end);
                obj.trajY2 = obj.trajY2(end-obj.maxTrajPoints+1:end);
            end

            if isempty(obj.h_traj2)
                obj.h_traj2 = plot(obj.h_ax, obj.trajX2, obj.trajY2, '-', 'LineWidth', 0.5, 'Color', [0.1 0.4 0.9]);
            else
                set(obj.h_traj2, 'XData', obj.trajX2, 'YData', obj.trajY2);
            end

            if isempty(obj.h_robotBody2)
                obj.h_robotBody2 = patch(obj.h_ax, robotBodyTransformed(1, :), robotBodyTransformed(2, :), [0.1 0.4 0.9], 'EdgeColor', 'none');
            else
                set(obj.h_robotBody2, 'XData', robotBodyTransformed(1, :), 'YData', robotBodyTransformed(2, :));
            end

            if isempty(obj.h_axisX2)
                obj.h_axisX2 = patch(obj.h_ax, axisX(1, :), axisX(2, :), [0.85 0.2 0.2], 'EdgeColor', 'none');
            else
                set(obj.h_axisX2, 'XData', axisX(1, :), 'YData', axisX(2, :));
            end

            if isempty(obj.h_axisY2)
                obj.h_axisY2 = patch(obj.h_ax, axisY(1, :), axisY(2, :), [0.2 0.7 0.2], 'EdgeColor', 'none');
            else
                set(obj.h_axisY2, 'XData', axisY(1, :), 'YData', axisY(2, :));
            end

            if isempty(obj.h_axisZ2)
                obj.h_axisZ2 = rectangle(obj.h_ax, 'Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02], 'Curvature', [1 1], 'FaceColor', [0.1 0.4 0.9], 'EdgeColor', 'none');
            else
                set(obj.h_axisZ2, 'Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02]);
            end
        end

        function obj = updateComparison(obj, imuPosition, imuHeading, sensorPosition, sensorHeading)
            obj = updatePose(obj, imuPosition, imuHeading);
            obj = updatePoseSecondary(obj, sensorPosition, sensorHeading);
        end

        function obj = updatePositionDesired(obj, position_desired)
            if isempty(obj.h_posidion_desired)
                obj.h_posidion_desired = scatter(obj.h_ax, position_desired(1), position_desired(2), 50, 'red', 'filled');
            else
                set(obj.h_posidion_desired, 'XData', position_desired(1), 'YData', position_desired(2));
            end
        end

        function obj = updateScan(obj, cart, position, heading)
            if nargin < 4
                heading = 0;
            end

            if nargin < 3 || isempty(position)
                position = [0, 0];
            end

            rotation = [cos(heading), -sin(heading); sin(heading), cos(heading)];
            cartWorld = (rotation * cart')' + position;

            if isempty(obj.h_cart)
                obj.h_cart = scatter(obj.h_ax, cartWorld(:,1), cartWorld(:,2), 10, 'blue', 'filled');
            else
                set(obj.h_cart, 'XData', cartWorld(:,1), 'YData', cartWorld(:,2));
            end
        end
    end
end
