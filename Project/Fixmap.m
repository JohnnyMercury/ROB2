%% edit_saved_map.m
% Loads a previously saved SLAM map and allows for manual, interactive
% editing (drawing walls or erasing noise) using polygons.
clear;
clc;
close all;

%% 1. Configuration & Parameters
inputFile = 'slam_map_test_best_sofar.mat';
outputFile = 'slam_map_fixed.mat';

% Base thresholds (applied before you manually edit)
newOccupancyThreshold = 0.50;  
newMinNeighbors = 1;           

%% 2. Load the Map Data
fprintf('[LOAD] Loading %s...\n', inputFile);
if ~isfile(inputFile)
    error('File %s not found in current directory.', inputFile);
end

load(inputFile, 'output');
if ~isfield(output, 'rawOccMatrix')
    error('The loaded file does not contain rawOccMatrix.');
end

rawMatrix = output.rawOccMatrix;
res = output.rawOccMap.Resolution;

%% 3. Apply Base Filtering
fprintf('[PROCESS] Applying base thresholds...\n');
occ = rawMatrix >= newOccupancyThreshold;
neighborCount = conv2(double(occ), ones(3), 'same');
occFiltered = occ & (neighborCount >= newMinNeighbors);

%% 4. Interactive Manual Editing
fprintf('[PROCESS] Starting interactive editor...\n');
editFig = figure('Name', 'Interactive Map Editor', 'NumberTitle', 'off', 'Position', [200, 200, 800, 800]);
imshow(occFiltered, 'InitialMagnification', 'fit');
title('Interactive Editor: Waiting for action...');

keepEditing = true;
while keepEditing
    % Ask the user what they want to do
    choice = questdlg('Select an action to modify the map:', 'Manual Map Edit', ...
        'Add Wall (Fill)', 'Erase Area (Clear)', 'Done Editing', 'Done Editing');
    
    switch choice
        case 'Add Wall (Fill)'
            title('Draw a polygon to ADD a wall. Double-click inside to finish.');
            disp('Draw a polygon. Click to add points, double-click to finish.');
            roi = drawpolygon();
            if isvalid(roi)
                mask = createMask(roi);
                occFiltered(mask) = true; % Set pixels inside polygon to occupied
                delete(roi);
            end
            imshow(occFiltered, 'InitialMagnification', 'fit');
            title('Interactive Editor: Waiting for action...');
            
        case 'Erase Area (Clear)'
            title('Draw a polygon to ERASE an area. Double-click inside to finish.');
            disp('Draw a polygon. Click to add points, double-click to finish.');
            roi = drawpolygon();
            if isvalid(roi)
                mask = createMask(roi);
                occFiltered(mask) = false; % Set pixels inside polygon to free space
                delete(roi);
            end
            imshow(occFiltered, 'InitialMagnification', 'fit');
            title('Interactive Editor: Waiting for action...');
            
        case 'Done Editing'
            keepEditing = false;
            
        otherwise
            % If the user closes the dialog box with the 'X'
            keepEditing = false;
    end
end
if ishandle(editFig)
    close(editFig);
end

% Create probabilistic overlay based on your manual edits
cleanProb = zeros(size(rawMatrix));
cleanProb(occFiltered) = 0.95;

%% 5. Rebuild Map Objects
fprintf('[PROCESS] Rebuilding map objects...\n');
newMapBinary = binaryOccupancyMap(occFiltered, res);
newMapProb = occupancyMap(cleanProb, res);

% Align origins with the raw map
if isprop(newMapBinary, 'GridLocationInWorld')
    try
        newMapBinary.GridLocationInWorld = output.rawOccMap.GridLocationInWorld;
        newMapProb.GridLocationInWorld = output.rawOccMap.GridLocationInWorld;
    catch
    end
end

%% 6. Save the Edited Map
output.mapCleanBinary = newMapBinary;
output.cleanOccProb = cleanProb;

save(outputFile, 'output', '-v7.3');
fprintf('[SAVE] Fixed map saved to: %s\n', outputFile);

%% 7. Visual Comparison
fig = figure('Name', 'Final Comparison', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 450]);
tiledlayout(1, 3);

nexttile;
show(output.rawOccMap);
title('1. Raw SLAM Map');

nexttile;
show(binaryOccupancyMap(output.rawOccMatrix >= 0.62, res));
title('2. Original Cleaned Map');

nexttile;
show(newMapBinary);
title('3. Manually Edited Map');