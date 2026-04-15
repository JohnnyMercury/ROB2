function handles = plotHeadings(varargin)
% PLOTHEADINGS Create or update heading plots.

if nargin == 0
    hFig = figure( ...
        'Name', 'Heading Plots', ...
        'NumberTitle', 'off', ...
        'Tag', 'HeadingPlotsFigure', ...
        'HandleVisibility', 'on' );

    hAx = axes('Parent', hFig);
    hold(hAx, 'on');
    hLineDes = plot(hAx, NaN, NaN, '-ob', 'LineWidth', 1.5, 'MarkerSize', 4);
    hLineAct = plot(hAx, NaN, NaN, '--or', 'LineWidth', 1.5, 'MarkerSize', 4);
    title(hAx, 'Desired vs Actual Heading');
    xlabel(hAx, 'Time (s)');
    ylabel(hAx, 'Heading (rad)');
    legend(hAx, {'Desired Heading', 'Actual Heading'}, 'Location', 'best');
    grid(hAx, 'on');
    ylim(hAx, [-pi pi]);

    handles.Figure      = hFig;
    handles.Axes        = hAx;
    handles.LineDesired = hLineDes;
    handles.LineActual  = hLineAct;

    setappdata(hFig, 'HeadingHandles', handles);
    return;
end

if nargin ~= 4
    error('plotHeadings expects either 0 inputs or 4 inputs: (handlesOrFig, t, hd_des, hd_act).');
end

handlesOrFig = varargin{1};
t = varargin{2};
hd_des = varargin{3};
hd_act = varargin{4};

if isgraphics(handlesOrFig, 'figure')
    handles = getappdata(handlesOrFig, 'HeadingHandles');
    if isempty(handles)
        error('Figure does not contain HeadingHandles. Call plotHeadings first.');
    end
else
    handles = handlesOrFig;
end

if ~isgraphics(handles.LineDesired, 'line') || ~isgraphics(handles.LineActual, 'line')
    error('Invalid line handles in handles struct.');
end

t = t(:).';
hd_des = hd_des(:).';
hd_act = hd_act(:).';

set(handles.LineDesired, 'XData', t, 'YData', hd_des);
set(handles.LineActual, 'XData', t, 'YData', hd_act);

if ~isempty(t)
    % Keep x-axis auto-scaling lightweight.
    xr = [t(1), t(end)];
    if diff(xr) == 0
        xr = xr + [-0.5 0.5];
    end
    xlim(handles.Axes, xr);
end

% Keep fixed y-limits for lightweight updates.
end
