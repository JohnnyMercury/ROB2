%% edit_custom_map.m
% Loads an existing saved map (like manual_custom_map.mat) and allows 
% the user to manually draw or erase walls.
% Displays the current mouse coordinates while editing.
clear;
clc;
close all;

%% 1. Select and Load the Map
disp('[PROCESS] Opening file browser to select a map...');
[inFile, inPath] = uigetfile('edited_custom_map.mat', 'Select the map file to edit (e.g., manual_custom_map.mat)');
if isequal(inFile, 0)
    disp('User canceled file selection. Exiting.');
    return;
end

inputPath = fullfile(inPath, inFile);
fprintf('[LOAD] Loading %s...\n', inputPath);

load(inputPath, 'output');
if ~isfield(output, 'rawOccMatrix') || ~isfield(output, 'mapCleanBinary')
    error('The selected file does not contain the required map data structs (rawOccMatrix and mapCleanBinary).');
end

% Extract the existing data
occMatrix = output.rawOccMatrix;
mapObj = output.mapCleanBinary;
mapResolution = mapObj.Resolution;

% Get the origin limits for coordinate tracking
try
    origin = mapObj.GridLocationInWorld;
catch
    origin = mapObj.LocalOriginInWorld;
end
xMin = origin(1);
yMin = origin(2);

%% 2. Interactive Manual Editing
fprintf('[PROCESS] Starting interactive editor...\n');
editFig = figure('Name', 'Custom Map Editor', 'NumberTitle', 'off', 'Position', [200, 200, 800, 800]);

% Display the pre-loaded map
ax = axes('Parent', editFig);
show(mapObj, 'Parent', ax);
title(ax, 'Interactive Editor: Waiting for action...');

% Setup a timer to constantly update the mouse coordinates in the title
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
    choice = questdlg('Select an action to modify your map:', 'Custom Map Editor', ...
        'Add Wall (Fill)', 'Erase Area (Clear)', 'Save & Exit', 'Add Wall (Fill)');
    
    if isempty(choice)
        choice = 'Save & Exit'; 
    end
    
    switch choice
        case 'Add Wall (Fill)'
            title(ax, 'Draw a polygon to ADD a wall. Double-click inside to finish.');
            disp('Draw a polygon to add a wall. Double-click inside to apply.');
            roi = drawpolygon(ax);
            if isvalid(roi)
                gridPos = world2grid(mapObj, roi.Position);
                mask = poly2mask(gridPos(:,2), gridPos(:,1), mapObj.GridSize(1), mapObj.GridSize(2));
                occMatrix(mask) = true; 
                delete(roi);
            end
            
        case 'Erase Area (Clear)'
            title(ax, 'Draw a polygon to ERASE an area. Double-click inside to finish.');
            disp('Draw a polygon to erase a wall. Double-click inside to apply.');
            roi = drawpolygon(ax);
            if isvalid(roi)
                gridPos = world2grid(mapObj, roi.Position);
                mask = poly2mask(gridPos(:,2), gridPos(:,1), mapObj.GridSize(1), mapObj.GridSize(2));
                occMatrix(mask) = false; 
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

% Clean up the timer 
if exist('coordTimer', 'var') && isvalid(coordTimer)
    stop(coordTimer);
    delete(coordTimer);
end

if ishandle(editFig)
    close(editFig);
end

%% 3. Save the Edited Map
disp('[PROCESS] Opening file browser to save the new map...');
[outFile, outPath] = uiputfile('*.mat', 'Save Edited Map As', 'edited_custom_map.mat');

if isequal(outFile, 0)
    disp('User canceled save. Changes were NOT saved.');
else
    outputPath = fullfile(outPath, outFile);
    
    % Package the output 
    finalMapObj = binaryOccupancyMap(occMatrix, mapResolution);
    try
        finalMapObj.GridLocationInWorld = [xMin, yMin];
    catch
        finalMapObj.LocalOriginInWorld = [xMin, yMin];
    end

    output.mapCleanBinary = finalMapObj;
    output.rawOccMatrix = occMatrix;
    output.lastEditedAt = datestr(now, 30); % Add a timestamp for the edit

    save(outputPath, 'output', '-v7.3');
    fprintf('[SAVE] Edited map successfully saved to: %s\n', outputPath);
end

%% Helper Function: Update Live Coordinates
function updateCoords(fig, ax)
    if isvalid(fig) && isvalid(ax)
        cp = ax.CurrentPoint;
        x = cp(1,1);
        y = cp(1,2);
        
        xlims = ax.XLim;
        ylims = ax.YLim;
        
        if x >= xlims(1) && x <= xlims(2) && y >= ylims(1) && y <= ylims(2)
            fig.Name = sprintf('Custom Map Editor | Current Mouse Coords: [X: %.2f,  Y: %.2f]', x, y);
        else
            fig.Name = 'Custom Map Editor';
        end
    end
end