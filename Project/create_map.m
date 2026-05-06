%% manual_map_creator.m
% Creates a blank map and allows the user to manually draw or erase walls.
% Displays the current mouse coordinates while editing so you know exactly
% where you are placing objects.
clear;
clc;
close all;

%% 1. Configuration: Set up your blank canvas
xMin = -5;          % Minimum X coordinate
xMax = 30;          % Maximum X coordinate
yMin = -5;          % Minimum Y coordinate
yMax = 30;          % Maximum Y coordinate

mapWidth = xMax - xMin;      % Width of the map in meters (35m)
mapHeight = yMax - yMin;     % Height of the map in meters (35m)
mapResolution = 20;          % Grid cells per meter (20 = 5cm accuracy)

% Create the empty binary matrix and map object
occMatrix = false(mapHeight * mapResolution, mapWidth * mapResolution);
mapObj = binaryOccupancyMap(occMatrix, mapResolution);

% Shift the origin so the bottom-left corner starts at [xMin, yMin]
try
    mapObj.GridLocationInWorld = [xMin, yMin];
catch
    mapObj.LocalOriginInWorld = [xMin, yMin];
end

%% 2. Interactive Manual Editing
fprintf('[PROCESS] Starting manual map creator...\n');
editFig = figure('Name', 'Manual Map Creator', 'NumberTitle', 'off', 'Position', [200, 200, 800, 800]);

% Display the initial empty map
ax = axes('Parent', editFig);
show(mapObj, 'Parent', ax);
title(ax, 'Interactive Editor: Waiting for action...');

% Setup a timer to constantly update the mouse coordinates in the title.
% We use a timer instead of WindowButtonMotionFcn so that it continues to
% update even while the drawpolygon tool is actively blocking the main thread.
coordTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, ...
    'TimerFcn', @(~,~) updateCoords(editFig, ax));
start(coordTimer);

keepEditing = true;
while keepEditing
    % Safety check in case the user closed the window manually
    if ~isvalid(ax)
        break;
    end

    % Menu for the user
    choice = questdlg('Select an action to modify your map:', 'Manual Map Creator', ...
        'Add Wall (Fill)', 'Erase Area (Clear)', 'Save & Exit', 'Add Wall (Fill)');
    
    if isempty(choice)
        choice = 'Save & Exit'; % Handle if the user closes the dialog with 'X'
    end
    
    switch choice
        case 'Add Wall (Fill)'
            title(ax, 'Draw a polygon to ADD a wall. Double-click inside to finish.');
            disp('Draw a polygon to add a wall. Double-click inside to apply.');
            roi = drawpolygon(ax);
            if isvalid(roi)
                % Convert spatial [X,Y] polygon vertices to grid matrix indices
                gridPos = world2grid(mapObj, roi.Position);
                % Create a mask of the polygon shape
                mask = poly2mask(gridPos(:,2), gridPos(:,1), mapObj.GridSize(1), mapObj.GridSize(2));
                occMatrix(mask) = true; % Set pixels inside polygon to occupied
                delete(roi);
            end
            
        case 'Erase Area (Clear)'
            title(ax, 'Draw a polygon to ERASE an area. Double-click inside to finish.');
            disp('Draw a polygon to erase a wall. Double-click inside to apply.');
            roi = drawpolygon(ax);
            if isvalid(roi)
                % Convert spatial [X,Y] polygon vertices to grid matrix indices
                gridPos = world2grid(mapObj, roi.Position);
                % Create a mask of the polygon shape
                mask = poly2mask(gridPos(:,2), gridPos(:,1), mapObj.GridSize(1), mapObj.GridSize(2));
                occMatrix(mask) = false; % Set pixels inside polygon to free space
                delete(roi);
            end
            
        case 'Save & Exit'
            keepEditing = false;
    end
    
    % Refresh the visual map display with the new changes
    if keepEditing && isvalid(ax)
        mapObj = binaryOccupancyMap(occMatrix, mapResolution);
        try
            mapObj.GridLocationInWorld = [xMin, yMin];
        catch
            mapObj.LocalOriginInWorld = [xMin, yMin];
        end
        show(mapObj, 'Parent', ax);
        title(ax, 'Interactive Editor: Waiting for action...');
    end
end

% Clean up the timer so it doesn't run forever in the background
if exist('coordTimer', 'var') && isvalid(coordTimer)
    stop(coordTimer);
    delete(coordTimer);
end

if ishandle(editFig)
    close(editFig);
end

%% 3. Save the Custom Map
outputFile = 'manual_custom_map.mat';

% Package the output similar to previous scripts so it works with your other code
output = struct();
finalMapObj = binaryOccupancyMap(occMatrix, mapResolution);
try
    finalMapObj.GridLocationInWorld = [xMin, yMin];
catch
    finalMapObj.LocalOriginInWorld = [xMin, yMin];
end

output.mapCleanBinary = finalMapObj;
output.rawOccMatrix = occMatrix;
output.createdAt = datestr(now, 30);

save(outputFile, 'output', '-v7.3');
fprintf('[SAVE] Custom map saved to: %s\n', outputFile);
disp('You can now load this file using your path planning or goal selector scripts!');

%% Helper Function: Update Live Coordinates
function updateCoords(fig, ax)
    % Checks the mouse position and updates the Figure Name with the coordinates
    if isvalid(fig) && isvalid(ax)
        cp = ax.CurrentPoint;
        x = cp(1,1);
        y = cp(1,2);
        
        % Get map limits to only show coordinates when hovering over the map
        xlims = ax.XLim;
        ylims = ax.YLim;
        
        if x >= xlims(1) && x <= xlims(2) && y >= ylims(1) && y <= ylims(2)
            fig.Name = sprintf('Manual Map Creator | Current Mouse Coords: [X: %.2f,  Y: %.2f]', x, y);
        else
            fig.Name = 'Manual Map Creator';
        end
    end
end