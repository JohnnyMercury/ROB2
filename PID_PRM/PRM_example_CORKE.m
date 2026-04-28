% Examples of PRM methods
clear all
clc
close all

%% Initialization - create map, set start and goal points
% 1.1m x 1.2m
% cellsize = 10cm x 10cm
% robot size -> 15cm x 15cm (xy base, only approx)


% Determine map scale
map_width = 1.5; % in meters
map_height = 1.5; % in meters
robot_size_side = 0.05; % in meters Hvor tæt path kommer på obstacle
simplegrid = binaryOccupancyMap(map_width, map_height, res=20);
resolution = 20;


setOccupancy(simplegrid, [0.7 0.0], ...
    ones(round(0.5*resolution), round(0.05*resolution)), 'world');
setOccupancy(simplegrid, [0.7 1.0], ...
    ones(round(0.5*resolution), round(0.05*resolution)), 'world');

% % Create occupancy grid and add obstacles

% i = [0.6];
% j = [0.6];
% simplegrid.setOccupancy([i j],ones(0.3,0.3),'world');

% Inflate obstacles
simplegrid.inflate(robot_size_side)
%% Set start and goal locations
start = [0.5 0.5];
goal = [1.1 1.1];

%% Probabilistic Roadmap method
figure
prm = mobileRobotPRM(simplegrid); % create prm planner
prm.NumNodes = 25;
path = prm.findpath(start, goal); % query planner for path
prm.show() % show path
hold on, plot(start(1), start(2), 'r*', 'MarkerSize', 20), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro', 'MarkerSize', 20), text(goal(1), goal(2), 'GOAL')
