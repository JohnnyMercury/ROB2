function handles = plotDistance(varargin)
% PLOTDISTANCE Create or update live distance-to-target plot.

if nargin == 0
    hFig = figure( ...
        'Name', 'Distance to Target', ...
        'NumberTitle', 'off', ...
        'Tag', 'DistancePlotFigure', ...
        'HandleVisibility', 'on');

    hAx = axes('Parent', hFig);
    hold(hAx, 'on');
    hLine = plot(hAx, NaN, NaN, '-g', 'LineWidth', 1.5);
    title(hAx, 'Distance to Target vs Time');
    xlabel(hAx, 'Time (s)');
    ylabel(hAx, 'Distance (m)');
    grid(hAx, 'on');

    handles.Figure = hFig;
    handles.Axes = hAx;
    handles.LineDistance = hLine;

    setappdata(hFig, 'DistanceHandles', handles);
    return;
end

if nargin ~= 3
    error('plotDistance expects either 0 inputs or 3 inputs: (handlesOrFig, t, d).');
end

handlesOrFig = varargin{1};
t = varargin{2};
d = varargin{3};

if isgraphics(handlesOrFig, 'figure')
    handles = getappdata(handlesOrFig, 'DistanceHandles');
    if isempty(handles)
        error('Figure does not contain DistanceHandles. Call plotDistance first.');
    end
else
    handles = handlesOrFig;
end

if ~isgraphics(handles.LineDistance, 'line')
    error('Invalid line handle in handles struct.');
end

t = t(:).';
d = d(:).';

set(handles.LineDistance, 'XData', t, 'YData', d);

if ~isempty(t)
    xr = [t(1), t(end)];
    if diff(xr) == 0
        xr = xr + [-0.5 0.5];
    end
    xlim(handles.Axes, xr);
end

if ~isempty(d)
    ymax = max(d);
    if ymax <= 0
        ylim(handles.Axes, [0 1]);
    else
        ylim(handles.Axes, [0 1.1 * ymax]);
    end
end
end
