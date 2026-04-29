%% mapBuildSLAM
% Stage 2 mapping script.
% Loads a saved SLAM session, applies stronger denoising, rebuilds SLAM,
% and exports cleaned occupancy map artifacts to Project/Maps.

clear;
clc;
close all;

%% User config
cfg.mapsFolder = fullfile(fileparts(mfilename('fullpath')), 'Maps');

% Set input session file manually, or leave empty to auto-pick latest.
cfg.inputSessionFile = 'slam_session_test_20260429_090926.mat';

cfg.outputPrefix = 'slam_map_test_';
cfg.maxLidarRange = 3.5;
cfg.mapResolution = 20;
cfg.maxNumScans = 6000;

cfg.loopClosureThreshold = 210;
cfg.loopClosureSearchRadius = 6;

% Stronger offline denoising settings
cfg.minValidRange = 0.08;
cfg.maxRangeMargin = 0.02;
cfg.baseMedianWindow = 11;
cfg.turnMedianWindow = 17;
cfg.turnStepThresholdRad = 0.08;      % if abs(dtheta) > this, use stronger denoise
cfg.localOutlierWindow = 7;           % odd
cfg.localOutlierThreshold = 0.12;     % meters

% Occupancy cleanup settings
cfg.occupancyThreshold = 0.62;
cfg.minOccupiedNeighbors = 3;         % 3x3 neighborhood vote
cfg.keepLargestComponent = false;

%% Resolve input file
if ~exist(cfg.mapsFolder, 'dir')
    error('Maps folder not found: %s', cfg.mapsFolder);
end

if isempty(cfg.inputSessionFile)
    files = dir(fullfile(cfg.mapsFolder, 'slam_session_*.mat'));
    if isempty(files)
        error('No slam_session_*.mat found in %s', cfg.mapsFolder);
    end
    [~, idx] = max([files.datenum]);
    inputPath = fullfile(files(idx).folder, files(idx).name);
else
    inputPath = cfg.inputSessionFile;
    if ~isfile(inputPath)
        inputPath = fullfile(cfg.mapsFolder, cfg.inputSessionFile);
    end
    if ~isfile(inputPath)
        error('Input session file not found: %s', cfg.inputSessionFile);
    end
end

fprintf('[LOAD] Input session: %s\n', inputPath);
S = load(inputPath);
if ~isfield(S, 'session')
    error('Input MAT does not contain ''session'' struct.');
end
session = S.session;

if ~isfield(session, 'scanArchive') || isempty(session.scanArchive)
    error('Session scanArchive is missing or empty.');
end

%% Rebuild filtered SLAM
slamFiltered = lidarSLAM(cfg.mapResolution, cfg.maxLidarRange, cfg.maxNumScans);
slamFiltered.LoopClosureThreshold = cfg.loopClosureThreshold;
slamFiltered.LoopClosureSearchRadius = cfg.loopClosureSearchRadius;

accepted = 0;
skipped = 0;

for i = 1:numel(session.scanArchive)
    rec = session.scanArchive(i);
    if ~isfield(rec, 'ranges') || ~isfield(rec, 'angles') || ~isfield(rec, 'relPose')
        skipped = skipped + 1;
        continue;
    end

    ranges = double(rec.ranges(:));
    angles = double(rec.angles(:));
    relPose = double(rec.relPose(:)');
    if numel(relPose) ~= 3
        skipped = skipped + 1;
        continue;
    end

    [rangesClean, anglesClean] = denoiseRangesAngles(ranges, angles, relPose(3), cfg);
    if numel(rangesClean) < 10
        skipped = skipped + 1;
        continue;
    end

    scan = lidarScan(rangesClean, anglesClean);
    [isAccepted, ~, ~] = addScan(slamFiltered, scan, relPose);
    if isAccepted
        accepted = accepted + 1;
    else
        skipped = skipped + 1;
    end
end

fprintf('[BUILD] Accepted scans: %d | skipped: %d\n', accepted, skipped);

%% Export occupancy maps
rawOccMap = getOccupancyMapCompat(slamFiltered, cfg.mapResolution, cfg.maxLidarRange);
rawOccMatrix = occupancyMatrix(rawOccMap);

[cleanBinary, cleanOccProb] = cleanOccupancy(rawOccMatrix, cfg);

res = rawOccMap.Resolution;
mapCleanBinary = binaryOccupancyMap(cleanBinary, res);
mapCleanProb = occupancyMap(cleanOccProb, res);

% Preserve world-frame origin so cleaned maps stay aligned with raw SLAM map.
mapCleanBinary = copyMapOrigin(mapCleanBinary, rawOccMap);
mapCleanProb = copyMapOrigin(mapCleanProb, rawOccMap);

%% Save outputs
output = struct();
output.inputSessionFile = inputPath;
output.cfg = cfg;
output.createdAt = datestr(now, 30);
output.acceptedScans = accepted;
output.skippedScans = skipped;
output.slamFiltered = slamFiltered;
output.rawOccMap = rawOccMap;
output.mapCleanBinary = mapCleanBinary;
output.mapCleanProb = mapCleanProb;
output.rawOccMatrix = rawOccMatrix;
output.cleanOccProb = cleanOccProb;

outName = [cfg.outputPrefix datestr(now, 'yyyymmdd_HHMMSS') '.mat'];
outPath = fullfile(cfg.mapsFolder, outName);
save(outPath, 'output', '-v7.3');

fprintf('[SAVE] Output map file: %s\n', outPath);

%% Quick visual comparison
fig = figure('Name', 'mapBuildSLAM Output', 'NumberTitle', 'off');
tiledlayout(fig, 1, 2);

nexttile;
show(rawOccMap);
title('Raw Occupancy (from rebuilt SLAM)');

nexttile;
show(mapCleanProb);
title('Cleaned Occupancy (offline filtered)');

%% Local helpers
function [rangesOut, anglesOut] = denoiseRangesAngles(ranges, angles, dtheta, cfg)
    keep = isfinite(ranges) & ranges > cfg.minValidRange & ranges < (cfg.maxLidarRange - cfg.maxRangeMargin);
    ranges = ranges(keep);
    angles = angles(keep);

    if isempty(ranges)
        rangesOut = [];
        anglesOut = [];
        return;
    end

    if abs(dtheta) > cfg.turnStepThresholdRad
        mw = makeOdd(cfg.turnMedianWindow);
    else
        mw = makeOdd(cfg.baseMedianWindow);
    end

    if numel(ranges) >= mw
        ranges = movmedian(ranges, mw);
    end

    lw = makeOdd(cfg.localOutlierWindow);
    if numel(ranges) >= lw
        localMed = movmedian(ranges, lw);
        inlier = abs(ranges - localMed) <= cfg.localOutlierThreshold;
        ranges = ranges(inlier);
        angles = angles(inlier);
    end

    rangesOut = ranges;
    anglesOut = angles;
end

function w = makeOdd(w)
    w = max(1, round(w));
    if mod(w, 2) == 0
        w = w + 1;
    end
end

function [cleanBinary, cleanProb] = cleanOccupancy(rawOccMatrix, cfg)
    occ = rawOccMatrix >= cfg.occupancyThreshold;

    neighborCount = conv2(double(occ), ones(3), 'same');
    occFiltered = occ & (neighborCount >= cfg.minOccupiedNeighbors);

    if cfg.keepLargestComponent
        occFiltered = keepLargestCC(occFiltered);
    end

    cleanBinary = occFiltered;

    % Keep a probabilistic map: occupied cells high confidence, others low.
    cleanProb = zeros(size(rawOccMatrix));
    cleanProb(occFiltered) = 0.95;
end

function bwOut = keepLargestCC(bw)
    cc = bwconncomp(bw, 8);
    if cc.NumObjects <= 1
        bwOut = bw;
        return;
    end

    sizes = cellfun(@numel, cc.PixelIdxList);
    [~, idx] = max(sizes);
    bwOut = false(size(bw));
    bwOut(cc.PixelIdxList{idx}) = true;
end

function occMap = getOccupancyMapCompat(slamObj, mapResolution, maxLidarRange)
    try
        occMap = getOccupancyGrid(slamObj);
        return;
    catch
        % Fall back for MATLAB versions where getOccupancyGrid(lidarSLAM)
        % is unavailable.
    end

    [scans, poses] = scansAndPoses(slamObj);
    if isempty(scans) || isempty(poses)
        error('Cannot build occupancy map: no scans/poses in SLAM object.');
    end

    occMap = buildMap(scans, poses, mapResolution, maxLidarRange);
end

function mapOut = copyMapOrigin(mapIn, mapRef)
mapOut = mapIn;

if isprop(mapOut, 'GridLocationInWorld') && isprop(mapRef, 'GridLocationInWorld')
    try
        mapOut.GridLocationInWorld = mapRef.GridLocationInWorld;
        return;
    catch
        % Fall through to alternate property names.
    end
end

if isprop(mapOut, 'LocalOriginInWorld') && isprop(mapRef, 'LocalOriginInWorld')
    try
        mapOut.LocalOriginInWorld = mapRef.LocalOriginInWorld;
    catch
        % Keep map as-is if origin property is unavailable in this release.
    end
end
end
