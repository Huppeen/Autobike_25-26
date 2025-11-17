function txt = syncSelectedPoint(~, event_obj, Results, trajPlot)
    % Get the coordinates of the selected point in the small plot
    pos = get(event_obj, 'Position');
    selectedTime = pos(1);  % Assume the first value is time
    selectedX = pos(2);     % The second value is the X coordinate

    % Find the index of the closest trajectory point
    timeDistances = abs(Results.bike_states.Time - selectedTime);
    [~, closestIndex] = min(timeDistances);

    % Get the corresponding coordinates in the main trajectory plot
    closestX = Results.bike_states.Data(closestIndex, 1);
    closestY = Results.bike_states.Data(closestIndex, 2);

    % Remove the previous black dot and mark the new point
    delete(findobj(trajPlot, 'Tag', 'selected_point'));
    hold(trajPlot, 'on');
    plot(trajPlot, closestX, closestY, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'selected_point');
    hold(trajPlot, 'off');

    % Return the text for display in Data Tips
    txt = {['Time: ', num2str(selectedTime)], ...
           ['X: ', num2str(closestX)], ...
           ['Y: ', num2str(closestY)]};
end
