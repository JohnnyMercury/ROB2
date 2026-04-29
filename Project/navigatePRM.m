function result = navigatePRM(mapInputFile, startPoint, goalPoint, cfg)
% navigatePRM
% Build a PRM roadmap from a saved SLAM map file and return a waypoint path.
%
% This function only plans (no velocity control). Main can then track the
% returned waypoints with navigatePID.
%
% INPUTS
%   mapInputFile : map file name or absolute path. If empty, latest
%                  slam_map_*.mat in Project/Maps is used.
%   startPoint   : [x y] in map frame. If empty, uses [0 0].
%   goalPoint    : [x y] in map frame (required).
%   cfg          : optional struct for PRM settings.
%
% OUTPUT
%   result struct fields:
%     .mapFilePath
%     .planningMap
%     .prm
%     .path
%     .startPoint
%     .goalPoint
%     .figureHandle
%
% EXAMPLE
%   goal = [1.8, 0.4];
%   out = navigatePRM('', [], goal);
%   waypoints = out.path;

if nargin < 4 || isempty(cfg)
    cfg = struct();
end
if nargin < 3
    error('navigatePRM requires at least goalPoint.');
end
if nargin < 2
    startPoint = [];
end
if nargin < 1
    mapInputFile = '';
end

cfg = applyDefaults(cfg);

if isempty(goalPoint) || numel(goalPoint) ~= 2
    error('goalPoint must be [x y].');
end
goalPoint = double(goalPoint(:)');

if isempty(startPoint)
    startPoint = [0, 0];
end
if numel(startPoint) ~= 2
    error('startPoint must be [x y].');
end
startPoint = double(startPoint(:)');

mapsFolder = fullfile(fileparts(mfilename('fullpath')), 'Maps');
mapFilePath = resolveMapFile(mapInputFile, mapsFolder, cfg.mapPrefix);

fprintf('[PRM] Loading map file: %s\n', mapFilePath);
S = load(mapFilePath);
planningMap = extractPlanningMap(S, cfg);

% Validate start/goal are in free space and within map limits.
[startPoint, goalPoint] = sanitizeStartGoal(planningMap, startPoint, goalPoint);

% Create PRM roadmap and find a path.
rng(cfg.randomSeed);
prm = mobileRobotPRM(planningMap);
prm.NumNodes = cfg.numNodes;
prm.ConnectionDistance = cfg.connectionDistance;
path = findpath(prm, startPoint, goalPoint);

% Retry with denser graph if needed.
if isempty(path)
    prm.NumNodes = cfg.retryNumNodes;
    prm.ConnectionDistance = cfg.retryConnectionDistance;
    path = findpath(prm, startPoint, goalPoint);
end

if isempty(path)
    error('PRM could not find a path from start to goal on this map.');
end

% Optional simplification to reduce waypoint count.
if cfg.simplifyPath
    path = reducePathCollinear(path, cfg.simplifyEps);
end

% Visualize plan.
fig = figure('Name', 'Project PRM Plan', 'NumberTitle', 'off');
show(prm);
hold on;
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
plot(startPoint(1), startPoint(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPoint(1), goalPoint(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Roadmap', 'Path', 'Start', 'Goal', 'Location', 'best');
title('PRM path from SLAM map');

result = struct();
result.mapFilePath = mapFilePath;
result.planningMap = planningMap;
result.prm = prm;
result.path = path;
result.startPoint = startPoint;
result.goalPoint = goalPoint;
result.figureHandle = fig;

fprintf('[PRM] Path found with %d waypoints.\n', size(path, 1));
end

function cfg = applyDefaults(cfg)
def.mapPrefix = 'slam_map_';
def.randomSeed = 1;
def.numNodes = 350;
def.connectionDistance = 0.35;
def.retryNumNodes = 700;
def.retryConnectionDistance = 0.5;
def.inflateRadius = 0.18;  % was 0.08, too small for TurtleBot3 burger (~14 cm radius). 0.18 m gives a safety margin.
def.occupancyThreshold = 0.62;
def.simplifyPath = true;
def.simplifyEps = 0.015;

f = fieldnames(def);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}) || isempty(cfg.(f{i}))
        cfg.(f{i}) = def.(f{i});
    end
end
end

function mapFilePath = resolveMapFile(mapInputFile, mapsFolder, mapPrefix)
if ~exist(mapsFolder, 'dir')
    error('Maps folder not found: %s', mapsFolder);
end

if isempty(mapInputFile)
    files = dir(fullfile(mapsFolder, [mapPrefix '*.mat']));
    if isempty(files)
        error('No %s*.mat found in %s', mapPrefix, mapsFolder);
    end
    [~, idx] = max([files.datenum]);
    mapFilePath = fullfile(files(idx).folder, files(idx).name);
    return;
end

if isfile(mapInputFile)
    mapFilePath = mapInputFile;
    return;
end

candidate = fullfile(mapsFolder, mapInputFile);
if isfile(candidate)
    mapFilePath = candidate;
    return;
end

error('Map file not found: %s', mapInputFile);
end

function planningMap = extractPlanningMap(S, cfg)
% Prefer cleaned binary map from mapBuildSLAM output.
if isfield(S, 'output')
    out = S.output;

    if isfield(out, 'mapCleanBinary') && ~isempty(out.mapCleanBinary)
        planningMap = out.mapCleanBinary;
        if isfield(out, 'rawOccMap') && ~isempty(out.rawOccMap)
            planningMap = tryReanchorMap(planningMap, out.rawOccMap);
        end
    elseif isfield(out, 'mapCleanProb') && ~isempty(out.mapCleanProb)
        occ = occupancyMatrix(out.mapCleanProb);
        planningMap = binaryOccupancyMap(occ >= cfg.occupancyThreshold, out.mapCleanProb.Resolution);
        if isfield(out, 'rawOccMap') && ~isempty(out.rawOccMap)
            planningMap = tryReanchorMap(planningMap, out.rawOccMap);
        end
    elseif isfield(out, 'rawOccMap') && ~isempty(out.rawOccMap)
        occ = occupancyMatrix(out.rawOccMap);
        planningMap = binaryOccupancyMap(occ >= cfg.occupancyThreshold, out.rawOccMap.Resolution);
        planningMap = tryReanchorMap(planningMap, out.rawOccMap);
    else
        error('Output map file does not contain a usable occupancy map field.');
    end
else
    error('Expected mapBuildSLAM output struct ''output'' in map file.');
end

if cfg.inflateRadius > 0
    inflate(planningMap, cfg.inflateRadius);
end
end

function [startPoint, goalPoint] = sanitizeStartGoal(planningMap, startPoint, goalPoint)
mapX = planningMap.XWorldLimits;
mapY = planningMap.YWorldLimits;

% Small tolerance avoids false outside-map errors due to floating-point
% near-zero values like -0.0000.
tol = max(1e-6, 0.5 / planningMap.Resolution);

if ~isInsideWorldTol(mapX, mapY, startPoint, tol)
    startPoint = clampToWorld(mapX, mapY, startPoint, tol);
    fprintf('[PRM] startPoint clamped into map limits: [%.3f %.3f]\n', startPoint(1), startPoint(2));
end
if ~isInsideWorldTol(mapX, mapY, goalPoint, tol)
    goalPoint = clampToWorld(mapX, mapY, goalPoint, tol);
    fprintf('[PRM] goalPoint clamped into map limits: [%.3f %.3f]\n', goalPoint(1), goalPoint(2));
end

% Always clamp to strict bounds to avoid tiny negative floating-point values
% (e.g. -0.0000) failing mobileRobotPRM.validateStartGoal.
startPoint = clampToWorld(mapX, mapY, startPoint, tol);
goalPoint = clampToWorld(mapX, mapY, goalPoint, tol);

if getOccupancy(planningMap, startPoint) > 0.5
    startPoint = nearestFreePoint(planningMap, startPoint);
    fprintf('[PRM] startPoint projected to nearest free cell: [%.3f %.3f]\n', startPoint(1), startPoint(2));
end

if getOccupancy(planningMap, goalPoint) > 0.5
    goalPoint = nearestFreePoint(planningMap, goalPoint);
    fprintf('[PRM] goalPoint projected to nearest free cell: [%.3f %.3f]\n', goalPoint(1), goalPoint(2));
end
end

function tf = isInsideWorld(xl, yl, p)
tf = (p(1) >= xl(1)) && (p(1) <= xl(2)) && (p(2) >= yl(1)) && (p(2) <= yl(2));
end

function tf = isInsideWorldTol(xl, yl, p, tol)
tf = (p(1) >= (xl(1) - tol)) && (p(1) <= (xl(2) + tol)) && ...
     (p(2) >= (yl(1) - tol)) && (p(2) <= (yl(2) + tol));
end

function p2 = clampToWorld(xl, yl, p, tol)
% Clamp to slightly-inside limits to avoid edge sampling artifacts.
xMin = xl(1) + tol;
xMax = xl(2) - tol;
yMin = yl(1) + tol;
yMax = yl(2) - tol;

if xMin > xMax
    xMin = xl(1);
    xMax = xl(2);
end
if yMin > yMax
    yMin = yl(1);
    yMax = yl(2);
end

p2 = [min(max(p(1), xMin), xMax), min(max(p(2), yMin), yMax)];
end

function pFree = nearestFreePoint(map, p)
res = map.Resolution;
searchRadiusM = 0.5;
stepCells = max(1, round(0.05 * res));
maxCells = max(stepCells, round(searchRadiusM * res));

for r = stepCells:stepCells:maxCells
    xs = linspace(p(1) - r / res, p(1) + r / res, 8 + r);
    ys = linspace(p(2) - r / res, p(2) + r / res, 8 + r);

    ring = [xs(:), repmat(ys(1), numel(xs), 1); ...
            xs(:), repmat(ys(end), numel(xs), 1); ...
            repmat(xs(1), numel(ys), 1), ys(:); ...
            repmat(xs(end), numel(ys), 1), ys(:)];

    % Keep points inside map limits.
    xl = map.XWorldLimits;
    yl = map.YWorldLimits;
    inside = ring(:,1) >= xl(1) & ring(:,1) <= xl(2) & ring(:,2) >= yl(1) & ring(:,2) <= yl(2);
    ring = ring(inside, :);
    if isempty(ring)
        continue;
    end

    occ = getOccupancy(map, ring);
    freeIdx = find(occ <= 0.5, 1, 'first');
    if ~isempty(freeIdx)
        pFree = ring(freeIdx, :);
        return;
    end
end

error('Could not find nearby free point from [%.3f %.3f].', p(1), p(2));
end

function pathOut = reducePathCollinear(pathIn, epsVal)
if size(pathIn, 1) <= 2
    pathOut = pathIn;
    return;
end

keep = true(size(pathIn, 1), 1);
for i = 2:size(pathIn, 1)-1
    a = pathIn(i, :) - pathIn(i-1, :);
    b = pathIn(i+1, :) - pathIn(i, :);
    if norm(a) < epsVal || norm(b) < epsVal
        keep(i) = false;
        continue;
    end

    crossVal = abs(a(1)*b(2) - a(2)*b(1));
    if crossVal < epsVal
        keep(i) = false;
    end
end

pathOut = pathIn(keep, :);
end

function mapOut = tryReanchorMap(mapIn, mapRef)
mapOut = mapIn;

if isprop(mapOut, 'GridLocationInWorld') && isprop(mapRef, 'GridLocationInWorld')
    try
        mapOut.GridLocationInWorld = mapRef.GridLocationInWorld;
        return;
    catch
        % Fall through.
    end
end

if isprop(mapOut, 'LocalOriginInWorld') && isprop(mapRef, 'LocalOriginInWorld')
    try
        mapOut.LocalOriginInWorld = mapRef.LocalOriginInWorld;
    catch
        % Keep map as-is for MATLAB releases without writable origin props.
    end
end
end
