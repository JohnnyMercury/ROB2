function map = createMap(cellsize)
%Create a map based on the cellsize

% Determine map scale
map_width = ...; % in meters
map_height = ...; % in meters
robot_size_side = 0.07; % in meters

% Create occupancy grid and add obstacles
simplegrid = binaryOccupancyMap(map_width, map_height, 'grid', res=X); % Set cell resolution X
i = [k]; % Set grid x index obstacle start
j = [l]; % Set grid y index obstacle start
simplegrid.setOccupancy([i j],ones(N,M),'grid'); % Set N & M to set size of obstacle area.

% Inflate obstacles
simplegrid.inflate(robot_size_side)

% Return generated map
map = simplegrid;
end