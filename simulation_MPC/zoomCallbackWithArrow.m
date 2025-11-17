function zoomCallbackWithArrow(event, trajPlot, Results)
    % Get the x-axis (time) limits for the zoom range
    xLimits = get(event.Axes, 'XLim'); % Retrieve the current x-axis zoom range (start and end times)

    % Define the start and end times of the zoom range
    startTime = xLimits(1); % Set the start time of the zoom range
    endTime = xLimits(2);   % Set the end time of the zoom range

    % Find data points within the zoom range
    timeIndices = find(Results.bike_states.Time >= startTime & Results.bike_states.Time <= endTime); 
    % Identify indices of data points within the start and end time range
    if isempty(timeIndices)
        return; % Exit if no data points are found
    end

    % Hold the plot and remove previous highlights and arrows
    hold(trajPlot, 'on'); % Hold the current plot to add new content
    delete(findobj(trajPlot, 'Tag', 'highlight')); % Remove previous highlighted segments (identified by Tag)
    delete(findall(gcf, 'Tag', 'arrow')); % Remove previous arrow annotations

    % Plot the highlighted red trajectory in the zoom range
    plot(trajPlot, Results.bike_states.Data(timeIndices, 1), ...
        Results.bike_states.Data(timeIndices, 2), ...
        'r', 'LineWidth', 2, 'Tag', 'highlight');
    % Plot the highlighted red trajectory segment within the zoom range, tagged as 'highlight' on trajPlot

    % Calculate the starting point and direction of the arrow
    x1 = Results.bike_states.Data(timeIndices(1), 1); % Get the X coordinate of the segment start
    y1 = Results.bike_states.Data(timeIndices(1), 2); % Get the Y coordinate of the segment start
    x2 = Results.bike_states.Data(timeIndices(2), 1); % Get the X coordinate of the second point of the segment
    y2 = Results.bike_states.Data(timeIndices(2), 2); % Get the Y coordinate of the second point of the segment

    % Arrow length and angle
    arrowLength = 0.1 * sqrt((x2 - x1)^2 + (y2 - y1)^2); % Set the arrow length based on the distance between start and second point
    angle = atan2(y2 - y1, x2 - x1); % Calculate the arrow direction angle (X and Y axis offsets)

    % Get current axis limits and position
    ax = ancestor(trajPlot, 'axes'); % Retrieve the parent axis of trajPlot
    xLimits = xlim(ax); % Get the current x-axis limits for normalization
    yLimits = ylim(ax); % Get the current y-axis limits
    axPosition = get(ax, 'Position'); % Get axis position within the figure window

    % Convert data coordinates to normalized coordinates
    xNorm = @(x) axPosition(1) + (x - xLimits(1)) / (xLimits(2) - xLimits(1)) * axPosition(3);
    % Define a normalization function to convert data X coordinates to figure coordinates
    yNorm = @(y) axPosition(2) + (y - yLimits(1)) / (yLimits(2) - yLimits(1)) * axPosition(4);
    % Define a normalization function to convert data Y coordinates to figure coordinates

    % Define start and end points for the arrow in normalized coordinates
    xStartNorm = xNorm(x1); % Normalized X coordinate of the arrow start point
    yStartNorm = yNorm(y1); % Normalized Y coordinate of the arrow start point
    xEndNorm = xNorm(x1 + arrowLength * cos(angle)); % Normalized X coordinate of the arrow end point
    yEndNorm = yNorm(y1 + arrowLength * sin(angle)); % Normalized Y coordinate of the arrow end point

    % Draw the arrow
    annotation('arrow', [xStartNorm xEndNorm], [yStartNorm yEndNorm], ...
               'Color', 'b', 'LineWidth', 1.5, 'Tag', 'arrow');
    % Add an arrow annotation in blue with specified width, tagged as 'arrow'

    % Release the plot hold
    hold(trajPlot, 'off'); % Release the hold to finalize current plot modifications
end
