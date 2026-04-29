 %% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Create map
pos_bot_left_of_box = [20 40]; % 20 down, 40 left (starts at 0.5, 0.5 though.
size_of_box = true(2,4); % a martix of ones, 
simplegrid = binaryOccupancyMap(50,50);
%simplegrid.setOccupancy([35 30],true(2,3),"grid");
%simplegrid.setOccupancy([5 4],true(1,2),"grid");
%setOccupancy(simplegrid,[25 40],true(1,2),"grid");
%setOccupancy(simplegrid, [1 2], true(1,1),"grid");

%% Create planner
dx = DistanceTransformPlanner(simplegrid)
 
%% Create plan for reaching goal
dx.plan([2 2])
 
%% Euclidean or Manhattan 
dx = DistanceTransformPlanner(simplegrid,metric="manhattan");
%dx = DStarPlanner(simplegrid,metric="manhattan");

%% Plan for goal
 dx.plan([2 2]);
% 
% %% Get path from starting point
 pathPoints = dx.query([35 23])

%% Plot path
figure
dx.plot(pathPoints)

%% Robot scale
% assumption robot occupies one cell. Some scale relating 1 cell to
% real-world sizefigure
% figure
%  dx = DistanceTransformPlanner(simplegrid, inflate=0);
%  dx.plan([2 2]);
%  pathPoints = dx.query([35 23]);
%  dx.plot(pathPoints)
