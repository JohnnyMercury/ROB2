classdef TurtleBotVisualise
    properties
        figAvatar
        figImage

        robotBody
        arrow
        scale = 2 % define scaling factor for visualization

        positions = zeros(0,2);

        h_robotBody
        h_axisX
        h_axisY
        h_axisZ
        h_cart
        h_position_desired
        h_trajectory

        cleanup
    end

    methods
        function obj = TurtleBotVisualise(~)
            %% Set up a figure for visualization
            obj.figAvatar = figure('units', 'normalized');
            hold on
            grid on
            axis equal
            xlim([-obj.scale obj.scale]);  % Define x-axis limits
            ylim([-obj.scale obj.scale]);  % Define y-axis limits

            % Configure plot appearance
            set(gca, 'fontsize', 20);
            xlabel('$x$ [m]', 'interpreter', 'latex', 'fontsize', 20);
            ylabel('$y$ [m]', 'interpreter', 'latex', 'fontsize', 20);
            set(gca, 'TickLabelInterpreter', 'latex')

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

            %% Set up a figure for image
            obj.figImage = figure;
        end

        function obj = updatePose(obj, position, heading)
            %% Transform the robot's body and axes
            robotBodyTransformed = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.robotBody + position;
            axisX = [cos(heading), -sin(heading); sin(heading), cos(heading)] * obj.arrow + position;
            axisY = [cos(heading + pi/2), -sin(heading + pi/2); sin(heading + pi/2), cos(heading + pi/2)] * obj.arrow + position;

            %% Delete old plots
            delete(obj.h_robotBody);
            delete(obj.h_axisX);
            delete(obj.h_axisY);
            delete(obj.h_axisZ);
            delete(obj.h_position_desired);
            delete(obj.h_trajectory);

            %% Plot the robot's body and axes
            figure(obj.figAvatar);
            obj.h_robotBody = patch(robotBodyTransformed(1, :), robotBodyTransformed(2, :), 'black', 'EdgeColor', 'none');
            obj.h_axisX = patch(axisX(1, :), axisX(2, :), 'red', 'EdgeColor', 'none');
            obj.h_axisY = patch(axisY(1, :), axisY(2, :), 'green', 'EdgeColor', 'none');
            obj.h_axisZ = rectangle('Position', [position(1), position(2), 0, 0] + obj.scale * [-0.01, -0.01, 0.02, 0.02], 'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'none');

            %% Plot past trajectory
            obj.h_trajectory = plot(obj.positions(:,1), obj.positions(:,2), '--k');
            obj.positions(end + 1,:) = position;
        end

        function obj = updatePositionDesired(obj, position_desired)
            %% Delete old plots
            delete(obj.h_position_desired);

            %% Plot desired position
            figure(obj.figAvatar);
            obj.h_position_desired = scatter(position_desired(1), position_desired(2), 50, 'red', 'filled');  % plot desired position
        end

        function obj = updateScan(obj, cart)
            %% Delete old plots
            delete(obj.h_cart);

            %% Plot laser scan
            figure(obj.figAvatar);
            obj.h_cart = scatter(cart(:,1), cart(:,2), 10, 'blue', 'filled');  % plot laser points
        end

        function obj = updateImage(obj, image)
            figure(obj.figImage);
            warning('off', 'all');
            imshow(image);
            warning('on', 'all');
        end        
    end
end