function [y, t, marker] = generate_line_with_circles(simTime, dt)
    % generate_line_with_circles: Generate a line with slope 2 and mark integer points
    %
    % Inputs:
    % simTime - Total simulation time (e.g., 10 seconds)
    % dt - Time step (e.g., 0.01 seconds)
    %
    % Outputs:
    % y - The line values (y = 2 * t)
    % t - Time vector
    % marker - A binary vector marking where y is an integer

    % Generate time vector
    t = 0:dt:simTime; % Time steps

    % Generate the line (y = 2 * t)
    y = 2 * t;

    % Mark points where y is an integer
    marker = mod(y, 1) == 0; % 1 where y is an integer, 0 otherwise
end
