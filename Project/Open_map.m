%% get_map_coordinates.m
% Loads the fixed SLAM map and lets the user click to get [X, Y] goal coordinates.
clear;
clc;
close all;

%% 1. Configuration
inputFile = 'slam_map_fixed.mat';

%% 2. Load the Map Data
fprintf('[LOAD] Loading %s...\n', inputFile);
if ~isfile(inputFile)
    error('File %s not found in current directory. Please run the editor script first.', inputFile);
end

load(inputFile, 'output');
if ~isfield(output, 'mapCleanBinary')
    error('The loaded file does not contain mapCleanBinary.');
end

% We'll use the binary map for clear visualization
map = output.mapCleanBinary;

%% 3. Display Map and Gather Coordinates
fig = figure('Name', 'Goal Coordinate Selector', 'NumberTitle', 'off', 'Position', [200, 200, 800, 600]);
show(map);
title('Click to record goals. Press ENTER when done.');

disp('--- Goal Coordinate Selector ---');
disp('Click on the map to record goal coordinates.');
disp('Press ENTER inside the figure window to finish and print the list.');

% ginput allows user to click points on the current axes
[x, y] = ginput();

%% 4. Print Selected Goals
if isempty(x)
    disp('No coordinates selected.');
else
    disp(' ');
    disp('--- Selected Goal Coordinates [X, Y] ---');
    % Print in a format easy to copy-paste into navigation code
    fprintf('goals = [\n');
    for i = 1:length(x)
        fprintf('    %.3f, %.3f;\n', x(i), y(i));
    end
    fprintf('];\n');
    disp('----------------------------------------');
    
    % Plot the selected points and paths on the map so you can verify them
    hold on;
    plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Draw points
    plot(x, y, 'r--', 'LineWidth', 1.5);              % Draw connecting lines
    hold off;
    title('Selected Goal Path');
end