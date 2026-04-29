function handles = plotDistance(varargin)
% PLOTDISTANCE Create or update live distance-to-target plot.
%   H = PLOTDISTANCE() creates a dedicated figure with one axes and one line.
%
%   H = PLOTDISTANCE(H, t, d) updates the existing line where H can be
%   either the handles struct returned by PLOTDISTANCE() or the
%   corresponding figure handle.

% varargin lets this function accept a variable number of inputs.
% Here we support two modes:
%   1) Create mode: plotDistance()
%   2) Update mode: plotDistance(handlesOrFig, t, d)

if nargin == 0 % n arg in -> number of arguments input
	% --- Create mode (first call) ---
	% Create a separate window so this plot does not interfere with other figures.
	hFig = figure( ...
		'Name', 'Distance to Target', ...
		'NumberTitle', 'off', ...
		'Tag', 'DistancePlotFigure', ...
		'HandleVisibility', 'on');

	% Create axes inside the figure and keep future drawings on the same axes.
	hAx = axes('Parent', hFig);
	hold(hAx, 'on');
	% Pre-create one line with NaN data; we will update this line each cycle.
	hLine = plot(hAx, NaN, NaN, '-g', 'LineWidth', 1.5);
	title(hAx, 'Distance to Target vs Time');
	xlabel(hAx, 'Time (s)');
	ylabel(hAx, 'Distance (m)');
	grid(hAx, 'on');

	% Store graphics handles so the caller can pass them back for fast updates.
	handles.Figure = hFig;
	handles.Axes = hAx;
	handles.LineDistance = hLine;

	% Also store handles on the figure, so update mode can accept figure handle too.
	setappdata(hFig, 'DistanceHandles', handles);
	return;
end

if nargin ~= 3
	% Any call that is not create mode must provide exactly 3 inputs.
	error('plotDistance expects either 0 inputs or 3 inputs: (handlesOrFig, t, d).');
end

% --- Update mode (called every loop) ---
handlesOrFig = varargin{1};
t = varargin{2};
d = varargin{3};

% Resolve handle source: either figure handle or handles struct.
if isgraphics(handlesOrFig, 'figure')
	handles = getappdata(handlesOrFig, 'DistanceHandles');
	if isempty(handles)
		error('Figure does not contain DistanceHandles. Call plotDistance first.');
	end
else
	handles = handlesOrFig;
end

% Safety check: ensure stored line still exists.
if ~isgraphics(handles.LineDistance, 'line')
	error('Invalid line handle in handles struct.');
end

% Force row vectors so XData and YData shapes are consistent.
t = t(:).';
d = d(:).';

% Fast live update: change existing line data (do not create a new line).
set(handles.LineDistance, 'XData', t, 'YData', d);

% Keep x-axis matched to time range (only update if range changed significantly).
if ~isempty(t)
	xr = [min(t), max(t)];
	if diff(xr) == 0, xr = xr + [-0.5 0.5]; end
	xlim(handles.Axes, xr);
end

% Keep y-axis non-negative and auto-scaled with a small headroom.
if ~isempty(d)
	ymax = max(d);
	if ymax <= 0
		ylim(handles.Axes, [0 1]);
	else
		ylim(handles.Axes, [0 1.1 * ymax]);
	end
end

% Request screen refresh, but rate-limited for performance (drawnow limitrate is key).
drawnow limitrate;
end
