%% Robot Vision: Object Detection & Distance Estimation (ROS2)
% Exercise 1: Detects orange and blue circles using HSV segmentation,
%             morphological operations, and circular Hough transform.
% Exercise 2: Estimates distance to detected circles using calibrated
%             focal length (f = 1226.5 px) and known circle size (H = 10 cm).

%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose poseOffset scan image

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic
imageSub = ros2subscriber(controlNode, '/camera/image_raw/compressed', @imageCallback); % image topic

% Pause to allow ROS subscriptions to initialize
pause(0.5);
    
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
    
    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();
    
    %% Initialize array for desired positions
    positionDesired = [1; 1];

    %% Setup for Task 3: Positioning and picture
    targetDistance = 1.0; % 1 meter from the circle
    pictureTaken = false; 
    Kp_dist = 0.5;        % P-gain for linear velocity
    Kp_angle = 0.002;     % P-gain for angle velocity

    %% Calculate offset
    % Store the initial pose as a reference so all future positions
    % are relative to where the robot started (origin = [0,0]).
    quatOffset = [poseOffset.orientation.x poseOffset.orientation.y poseOffset.orientation.z poseOffset.orientation.w];
    orientationOffset = quat2eul(quatOffset);  % Convert offset quaternion to Euler angles
    headingOffset = orientationOffset(3); % Extract offset heading (yaw)

    %% Calculate transformations for offset
    % Build rotation (R) and translation (t) matrices to transform
    % world-frame odometry into a local frame starting at [0,0,0].
    positionOffset = [poseOffset.position.x; poseOffset.position.y];
    R_W2R = [cos(-headingOffset), -sin(-headingOffset); sin(-headingOffset), cos(-headingOffset)];
    t_R2V = -R_W2R * positionOffset;
    R_R2V = [cos(headingOffset), -sin(headingOffset); sin(headingOffset), cos(headingOffset)]';
    
    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Visialise desired position
        visualise = updatePositionDesired(visualise, positionDesired);

        %% Get the robot's current position and heading
        position = [pose.position.x; pose.position.y];
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading = orientation(3); % Extract heading (yaw)

        %% Apply offset
        position = R_R2V * position + t_R2V;
        heading = heading - headingOffset; % Offset heading

        %% Visualise the robot
        visualise = updatePose(visualise, position, heading);
    
        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position'; % Transform based on robot position and heading
        visualise = updateScan(visualise, cart);

        %% Visualise image
        imageRGB = flip(image, 1); % Flip image vertically (upside down fix)
        imageGray = rgb2gray(imageRGB);
        centersO = []; radiiO = [];
        centersB = []; radiiB = [];
        
        %% Exercise 1: Object Detection
        % --- A. PREPROCESSING ---
            % stretchlim finds the intensity range covering 98% of pixels,
            % then imadjust stretches it to [0,1] - improves contrast
            % under variable lighting conditions.
            lowHigh = stretchlim(imageRGB, [0.01 0.99]);
            enhancedImage = imadjust(imageRGB, lowHigh, []);
            
            % Gaussian filter (sigma=1) smooths out camera sensor noise
            % to prevent false edges during circle detection.
            smoothImage = imgaussfilt(enhancedImage, 1);
            
            % Convert RGB to HSV because HSV separates color (Hue) from
            % brightness (Value), making color-based segmentation more
            % robust to lighting changes than RGB thresholding.
            hsvImage = rgb2hsv(smoothImage);
            
            % --- B. THRESHOLDING (Color segmentation) ---
            % Saturation > 0.50 filters out gray/white pixels.
            % Value > 0.20 filters out very dark pixels.
            satMin = 0.50; valMin = 0.20;
            
            % --- ORANGE CIRCLE MASK ---
            BW_orange = (hsvImage(:,:,1) >= 0.05 & hsvImage(:,:,1) <= 0.15) & ...
                     (hsvImage(:,:,2) >= satMin) & ...
                     (hsvImage(:,:,3) >= valMin);
                     
            % --- BLUE CIRCLE MASK ---
            BW_blue = (hsvImage(:,:,1) >= 0.55 & hsvImage(:,:,1) <= 0.65) & ...
                      (hsvImage(:,:,2) >= satMin) & ...
                      (hsvImage(:,:,3) >= valMin);
                      
            % Morphology: imopen removes small noise blobs (smaller than
            % the disk radius), imclose fills small holes inside the circle.
            % Using a disk structuring element because we are looking for
            % circular shapes.
            se = strel('disk', 5);
            BW_orange_clean = imclose(imopen(BW_orange, se), se);
            BW_blue_clean = imclose(imopen(BW_blue, se), se);
            
            % imfindcircles uses Circular Hough Transform (CHT) to detect
            % circles on the binary mask. Radius range [20 200] px covers
            % expected circle sizes at different distances.
            % 'ObjectPolarity','bright' = white circles on black background.
            
            % Find orange cirkler i stedet for røde
            [centersO, radiiO] = imfindcircles(BW_orange_clean, [20 200], 'ObjectPolarity', 'bright');
            if ~isempty(centersO)
                diameterO = 2 * radiiO(1);
                disp(['Orange circle detected! Diameter: ', num2str(diameterO), ' px']);
            end
            
            [centersB, radiiB] = imfindcircles(BW_blue_clean, [20 200], 'ObjectPolarity', 'bright');
            if ~isempty(centersB)
                diameterB = 2 * radiiB(1);
                disp(['Blue circle detected! Diameter: ', num2str(diameterB), ' px']);
            end
        
        visualise = updateImage(visualise, imageGray);
        figure(visualise.figImage);
        hold on;
            
            % Draw orange circles
            if ~isempty(centersO)
                % Draw circle outline in orange (RGB: [1 0.5 0])
                viscircles(centersO(1,:), radiiO(1), 'EdgeColor', [1 0.5 0], 'LineWidth', 2);
                % Draw orange cross at center
                plot(centersO(1,1), centersO(1,2), '+', 'Color', [1 0.5 0], 'MarkerSize', 10, 'LineWidth', 2);
                % Write diameter on image
                text(centersO(1,1)+10, centersO(1,2), ...
                     sprintf('Orange (D=%.0f px)', 2*radiiO(1)), ...
                     'Color', [1 0.5 0], 'FontSize', 12, 'FontWeight', 'bold');
            end
            
            % Draw blue circles
            if ~isempty(centersB)
                % Draw circle outline in blue
                viscircles(centersB(1,:), radiiB(1), 'EdgeColor', 'b', 'LineWidth', 2);
                % Draw blue cross at center
                plot(centersB(1,1), centersB(1,2), 'b+', 'MarkerSize', 10, 'LineWidth', 2);
                % Write diameter on image
                text(centersB(1,1)+10, centersB(1,2), ...
                     sprintf('Blue (D=%.0f px)', 2*radiiB(1)), ...
                     'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
            end
            
            hold off; % End overlay drawing
            
        %% Exercise 2: Distance Estimation
        % Using the pinhole camera model: Dp = f * H / D
        %   Dp = circle diameter in pixels (from Exercise 1)
        %   H  = real-world circle diameter (measured: 10 cm)
        %   D  = distance from camera to circle (what we estimate)
        %   f  = focal length in pixels (calibrated from multiple measurements)
        % Rearranged: D = f * H / Dp
        H = 0.10; % Real circle size in meters (10 cm)
        f = 1226.5;  % Camera focal length in pixels (calibrated from 6 measurements)
        
        if ~isempty(centersO)
            distanceO = f * H / (2 * radiiO(1)); % 2*radius = diameter
            fprintf('Orange circle distance: %.2f m\n', distanceO);
        end
        if ~isempty(centersB)
            distanceB = f * H / (2 * radiiB(1));
            fprintf('Blue circle distance: %.2f m\n', distanceB);
        end
        
       %% Task 3 Logic: Positioning and Picture Taking
        % Default hastigheder
        linearVelocity = 0.0;
        angularVelocity = 0.0;
        
        imgWidth = size(imageRGB, 2); 
        
        if ~pictureTaken
            activeDistance = [];
            activeCenterX = [];
            
            % Tjek om vi ser en cirkel (prioriter orange, derefter blå)
            if ~isempty(centersO)
                activeDistance = distanceO;
                activeCenterX = centersO(1, 1);
            elseif ~isempty(centersB)
                activeDistance = distanceB;
                activeCenterX = centersB(1, 1);
            end
            
            % Hvis en cirkel er i syne, start positionering
            if ~isempty(activeDistance)
                % Beregn fejl (hvor langt er vi fra målet?)
                errorDist = activeDistance - targetDistance; % Afstand til 1 meter markeringen
                errorAngle = (imgWidth / 2) - activeCenterX; % Afstand fra cirklens centrum til billedets centrum
                
                % P-controllere sætter hastigheden baseret på fejlen
                linearVelocity = Kp_dist * errorDist;
                angularVelocity = Kp_angle * errorAngle;
                
                % Tjek om vi er "tæt nok" på målet (tolerance: 5 cm og 20 pixels)
                if abs(errorDist) < 0.05 && abs(errorAngle) < 20
                    % Vi er i position! Stop robotten.
                    linearVelocity = 0.0;
                    angularVelocity = 0.0;
                    
                    % Send stop-besked med det samme
                    cmdMsg = ros2message('geometry_msgs/Twist');
                    cmdMsg.linear.x = 0;
                    cmdMsg.angular.z = 0;
                    send(cmdPub, cmdMsg);
                    
                    % Tag billedet og gem det i din arbejdsmappe
                    disp('Target reached! Taking picture...');
                    imwrite(imageRGB, 'TB_Circle_Picture.png');
                    pictureTaken = true; % Lås så vi ikke tager flere billeder
                    
                    % Hold en lille pause for at sikre billedet bliver gemt
                    pause(1); 
                end
            end
        else
            % Hvis billedet allerede er taget, hold robotten stille (Klar til punkt 4)
            linearVelocity = 0.0;
            angularVelocity = 0.0;
            % disp('Task 3 complete. Picture saved as TB_Circle_Picture.png');
        end
        
        %% Publish velocity commands
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = max(min(linearVelocity, 0.15), -0.15); % Begræns hastighed til max 0.15 m/s
        cmdMsg.angular.z = max(min(angularVelocity, 0.5), -0.5); % Begræns drejehastighed
        send(cmdPub, cmdMsg);
        
        %% Pause to visualize and delete old plots
        pause(0.1)

        %% Exit the loop if the figure is closed
        if size(findobj(visualise.figAvatar)) == 0 | size(findobj(visualise.figImage)) == 0
            ME = MException('NonExeption:EndProgram', 'The program was closed.');
            throw(ME)
        end
        
end
        catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.x = 0;
    cmdMsg.Angular.z = 0;
    try send(cmdPub, cmdMsg); catch; end
    % Close all figures
    close all
    
    % Clean up ROS subscriptions
    clear odomSub scanSub imageSub
    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end 

% %% Callback functions
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose poseOffset
    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;
    if isempty(poseOffset)
        poseOffset = message.pose.pose;
    end
end

function scanCallback(message)
    % Use global variable to store laser scan data
    global scan
    % Save the laser scan message
    scan = message;
end

function imageCallback(message)
    % Use global variable to store laser scan data
    global image
    % Save the laser scan message
    image = rosReadImage(message);
end