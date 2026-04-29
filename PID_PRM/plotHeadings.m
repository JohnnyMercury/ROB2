function handles = plotHeadings(varargin)
% PLOTHEADINGS Create or update heading plots.
%   H = PLOTHEADINGS() creates a dedicated figure (not using gcf) with one
%   shared axes containing two lines: desired and actual heading.
%
%   H = PLOTHEADINGS(H, t, hd_des, hd_act) updates the existing lines where
%   H can be either the handles struct returned from PLOTHEADINGS() or the
%   corresponding figure handle.
%
%   This file must be saved as plotHeadings.m

% varargin lets this function accept a variable number of inputs.
% Here we support two modes:
%   1) Create mode: plotHeadings()
%   2) Update mode: plotHeadings(handlesOrFig, t, hd_des, hd_act)

if nargin == 0
    % --- Create mode (first call) ---
    % Create a dedicated figure (not interfering with other visuals)
    hFig = figure( ...
        'Name', 'Heading Plots', ...
        'NumberTitle', 'off', ...
        'Tag', 'HeadingPlotsFigure', ...
        'HandleVisibility', 'on' );

    % Create one shared axes with two lines
    hAx = axes('Parent', hFig);
    hold(hAx, 'on');
    % Pre-create desired and actual lines with NaN values; update every loop.
    hLineDes = plot(hAx, NaN, NaN, '-ob', 'LineWidth', 1.5, 'MarkerSize', 4);
    hLineAct = plot(hAx, NaN, NaN, '--or', 'LineWidth', 1.5, 'MarkerSize', 4);
    title(hAx, 'Desired vs Actual Heading');
    xlabel(hAx, 'Time (s)');
    ylabel(hAx, 'Heading (rad)');
    legend(hAx, {'Desired Heading', 'Actual Heading'}, 'Location', 'best');
    grid(hAx, 'on');

    % Store graphics handles so the caller can pass them back for fast updates.
    handles.Figure      = hFig;
    handles.Axes        = hAx;
    handles.LineDesired = hLineDes;
    handles.LineActual  = hLineAct;

    % Also store handles on the figure, so update mode can accept figure handle too.
    setappdata(hFig, 'HeadingHandles', handles);
    return;
end

if nargin ~= 4
    % Any call that is not create mode must provide exactly 4 inputs.
    error('plotHeadings expects either 0 inputs or 4 inputs: (handlesOrFig, t, hd_des, hd_act).');
end

% --- Update mode (called every loop) ---
handlesOrFig = varargin{1};
t = varargin{2};
hd_des = varargin{3};
hd_act = varargin{4};

% Resolve handle source: either figure handle or handles struct.
if isgraphics(handlesOrFig, 'figure')
    handles = getappdata(handlesOrFig, 'HeadingHandles');
    if isempty(handles)
        error('Figure does not contain HeadingHandles. Call plotHeadings first.');
    end
else
    handles = handlesOrFig;
end

% Safety check: ensure stored line handles still exist.
if ~isgraphics(handles.LineDesired, 'line') || ~isgraphics(handles.LineActual, 'line')
    error('Invalid line handles in handles struct.');
end

% Force row vectors so XData and YData shapes are consistent.
t = t(:).';
hd_des = hd_des(:).';
hd_act = hd_act(:).';

% Optional: unwrap headings to avoid visual jumps at +/- pi.
hd_des = unwrap(hd_des);
hd_act = unwrap(hd_act);

% Fast live update: change existing line data (do not create new lines).
set(handles.LineDesired, 'XData', t, 'YData', hd_des);
set(handles.LineActual, 'XData', t, 'YData', hd_act);

% Keep x-axis matched to time range (only update if range changed significantly).
if ~isempty(t)
    xr = [min(t), max(t)];
    if diff(xr) == 0, xr = xr + [-0.5 0.5]; end
    xlim(handles.Axes, xr);
end

% Auto-scale y-axis to data range (heading in radians: -π to +π)
if ~isempty(hd_des) && ~isempty(hd_act)
    ymax = max([max(hd_des), max(hd_act)]);
    ymin = min([min(hd_des), min(hd_act)]);
    yrange = ymax - ymin;
    if yrange < 0.1  % Prevent axis collapse if data is nearly constant
        yrange = 1.0;
        ymid = (ymax + ymin) / 2;
        ylim(handles.Axes, [ymid - 0.5, ymid + 0.5]);
    else
        % Add 10% headroom above/below data
        ylim(handles.Axes, [ymin - 0.1*yrange, ymax + 0.1*yrange]);
    end
end

% Request screen refresh, but rate-limited for performance (drawnow called in main loop).
% drawnow limitrate;
end