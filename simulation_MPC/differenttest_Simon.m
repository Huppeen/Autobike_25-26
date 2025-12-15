function [Xref,Yref,Psiref,t_ref] = differenttest_Simon(testnumber,ref_dis,lL,laps,Vref_test)
switch testnumber 
case '1' 
% Generate a straight-line trajectory with the function (ReferenceGenerator)
[Xref,Yref,Psiref] = ReferenceGenerator('line',ref_dis,lL,laps);
% Time vector calculated based on trajectory length and speed
t_ref = linspace(0, (lL-1)/Vref_test, lL)';
case '2'
%% test 2: Rotate the trajectory of Test 1 counterclockwise by "rotate_angle_traj" degrees; the other parameters keep unchanged
% Generate a straight-line trajectory with the function (ReferenceGenerator)
[Xref,Yref,Psiref] = ReferenceGenerator('line',ref_dis,lL,laps);
% Convert the angle "rotate_angle_traj" from degrees to radians
rotate_angle_traj = 0;
rotate_angle_traj = deg2rad(rotate_angle_traj); 
% Define the 2D rotation matrix
rotate_matrix = [cos(rotate_angle_traj), -sin(rotate_angle_traj); 
sin(rotate_angle_traj), cos(rotate_angle_traj)];
% Combine Xref and Yref into a matrix for transformation
coords = [Xref(:), Yref(:)]'; 
% Apply the rotation matrix
rotated_coords = rotate_matrix * coords;
% Rotated Xref
Xref = rotated_coords(1, :)'; 
% Rotated Yref
Yref = rotated_coords(2, :)'; 
% Rotate Psiref
Psiref = Psiref + rotate_angle_traj;
% Time reference
t_ref = linspace(0, (lL-1)/Vref_test, lL)';
% make it so it completes a full circle and is also compatible with
% making mulitple laps
case '3'
% Parameters
% radius = 10; %usually 10 gonna try 30 or 40 to give it 10 laps without crashing needs 120 straight length for 40 radius
% circle_center = [40, 10]; %usually 40 but needed to be 120
straight_start = [0, 0];
% straight_end = [40, 0]; %usually 40 but updated to 120 
radius = 30;
straight_end = [90,0];
circle_center = [90,30];
ref_dis = 0.1;
% First segment: straight line
x1 = linspace(straight_start(1), straight_end(1));
y1 = zeros(size(x1));
% Circle segment, scaled to laps
points_per_lap = 200; % resolution per lap
total_circle_points = laps * points_per_lap;
theta = linspace(-pi/2, 3*pi/2 + 2*pi*(laps-1), total_circle_points);
x2 = circle_center(1) + radius * cos(theta);
y2 = circle_center(2) + radius * sin(theta);
% Combine segments
x = [x1, x2];
y = [y1, y2];
% Remove duplicates if any
coords = [x', y'];
[unique_coords, idx] = unique(coords, 'rows', 'stable');
x = unique_coords(:, 1);
y = unique_coords(:, 2);
% Recalculate cumulative distance
dx = diff(x);
dy = diff(y);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];
% evenly spaced reference points
num_points = floor(cumulative_distance(end)/ref_dis) + 1;
equal_distances = linspace(0, cumulative_distance(end), num_points);
% Interpolate
Xref = interp1(cumulative_distance, x, equal_distances)';
Yref = interp1(cumulative_distance, y, equal_distances)';
% heading
Psiref = [atan2(Yref(2)-Yref(1), Xref(2)-Xref(1));
atan2(diff(Yref), diff(Xref))];
% time
t_ref = equal_distances' / Vref_test;
case '4'
% Define parameters
N_line = 50; % Number of points for the straight line
N_circle = 280; % Number of points for the circle, 135
scale = 50; % Circle radius
ref_dis = 0.8; % Reference distance
% Straight line part
t_line = (0:(N_line-1))';
Xref_line = t_line * ref_dis;
Yref_line = zeros(N_line, 1);
% Circle part
t_circle = (1:N_circle)' * ref_dis / scale; % Start from the first point to avoid overlap
Xref_circle = scale * sin(t_circle) + Xref_line(end); % Connect circle start to line end
Yref_circle = -scale * cos(t_circle) + scale; % Keep the circle centered within the range
% Combine path
Xref = [Xref_line; Xref_circle];
Yref = [Yref_line; Yref_circle];
% Calculate distances between consecutive points
distances = sqrt(diff(Xref).^2 + diff(Yref).^2);
% Define speed profile parameters
initial_speed = 2; % Initial speed in m/s
final_speed = 4; % Final speed in m/s
hold_time = 10; % Time to hold the final speed in seconds
switch_time = 10; % Time at which speed switches to 4 m/s
% Generate time reference for each point
time_ref = [0; cumsum(distances / initial_speed)];
% Adjust speed profile based on time_ref
speed_profile = initial_speed * ones(size(time_ref));
speed_profile(time_ref >= switch_time & time_ref < switch_time + hold_time) = final_speed;
speed_profile(time_ref >= switch_time + hold_time) = initial_speed;
% Calculate tangent angles between consecutive points
Psiref = [atan2(Yref(2) - Yref(1), Xref(2) - Xref(1)); atan2(Yref(2:end) - Yref(1:end-1), Xref(2:end) - Xref(1:end-1))];
% Recalculate adjusted time reference based on the new speed profile
t_ref = [0; cumsum(distances ./ speed_profile(1:end-1))];
case '5'
% Define parameters
N_line = 50; % Number of points for the straight line
Vref_test = 2; % Initial speed in m/s
% Straight line part
t_ref = (0:N_line-1)' * 0.5; % Generate values like 0, 0.5, 1, 1.5, ..., 24.5
Xref = zeros(N_line, 1);
Yref = zeros(N_line, 1);
% Introduce speed change at specific time
boost_time = 5; % Time to boost speed (in seconds)
boost_duration = 5; % Duration of boost (in seconds)
boost_index = find(t_ref >= boost_time & t_ref < (boost_time + boost_duration));
% Adjust Xref to be integers with non-uniform increments
Vref = Vref_test * ones(N_line-1, 1); % Initial speed array
Vref(boost_index(1:end-1)) = 4; % Adjust speed to 4 m/s during boost
% Compute Xref with position as integers
for i = 2:N_line
dt = t_ref(i) - t_ref(i-1); % Constant time step
Xref(i) = Xref(i-1) + Vref(i-1) * dt; % Varying position increments rounded to integers
end
% Calculate tangent angles between consecutive points
Psiref = [atan2(Yref(2) - Yref(1), Xref(2) - Xref(1)); atan2(Yref(2:end) - Yref(1:end-1), Xref(2:end) - Xref(1:end-1))];
case '6'
% Define parameters
N_line = 50; % Number of points for the straight line
Vref_test = 2; % Initial speed in m/s
% Straight line part
t_line = (0:(N_line-1))';
Xref_line = t_line * ref_dis;
Yref_line = zeros(N_line, 1);
% Combine path (only straight line)
Xref = Xref_line;
Yref = Yref_line;
% Calculate distances between consecutive points
distances = sqrt(diff(Xref).^2 + diff(Yref).^2);
% Introduce speed change at a specific time
boost_time = 5; % Time to boost speed (in seconds)
boost_duration = 10; % Duration of boost (in seconds)
Vref = Vref_test * ones(size(distances)); % Initial speed array
% Adjust speed to 4 m/s for the boost duration
for i = 1:length(t_line)-1
if t_line(i) >= boost_time && t_line(i) < (boost_time + boost_duration)
Vref(i) = 4;
end
end
% Calculate time series to account for speed change
t_ref = [0; cumsum(distances ./ Vref)];
% Calculate tangent angles between consecutive points
Psiref = [atan2(Yref(2) - Yref(1), Xref(2) - Xref(1)); atan2(Yref(2:end) - Yref(1:end-1), Xref(2:end) - Xref(1:end-1))];
case '7'
% Define parameters
N_line = 50; % Number of points
Vref_test = 2; % Initial speed (m/s)
% Uniform Xref and Yref
dx = 1; % Position step (1 meter)
Xref = (0:N_line-1)' * dx; % Uniform Xref
Yref = zeros(N_line, 1); % Yref remains zero for a straight line
% Speed change between Xref = 5 and 10 meters
boost_start_index = find(Xref >= 5 & Xref < 15); % Boost interval
Vref = Vref_test * ones(N_line, 1); % Initial speed 2 m/s
Vref(boost_start_index) = 4; % Set speed to 4 m/s during boost
% Calculate t_ref based on uniform Xref and varying Vref
t_ref = zeros(N_line, 1); % Initialize time array
for i = 2:N_line
dt = dx / Vref(i-1); % Time step based on speed
t_ref(i) = t_ref(i-1) + dt; % Accumulate time
end
% Calculate tangent angles
Psiref = [atan2(Yref(2) - Yref(1), Xref(2) - Xref(1)); atan2(Yref(2:end) - Yref(1:end-1), Xref(2:end) - Xref(1:end-1))];
case '8' 
% Parameter settings
start_point = [0, 0]; % Start point
first_straight_end = [40, 0]; % End point of the first straight segment
circle_center = [40, 10]; % Center of the semicircle
circle_radius = 10; % Radius of the semicircle
final_point = [0, 20]; % End point
speed = Vref_test; % Speed
% First segment: Straight line along the X-axis
x1 = linspace(start_point(1), first_straight_end(1));
y1 = zeros(size(x1));
% Second segment: Semicircular path
theta = linspace(-pi/2, pi/2, 100); % Angle range for the semicircle
x2 = circle_center(1) + circle_radius * cos(theta);
y2 = circle_center(2) + circle_radius * sin(theta);
% Third segment: Straight line back to the endpoint
x3 = linspace(first_straight_end(1), final_point(1));
y3 = linspace(circle_center(2) + circle_radius, final_point(2), 100);
% Combine the trajectory
x = [x1, x2, x3];
y = [y1, y2, y3];
% Remove duplicate points
coords = [x', y'];
[unique_coords, idx] = unique(coords, 'rows', 'stable');
x = unique_coords(:, 1);
y = unique_coords(:, 2);
% Recalculate cumulative distance
dx = diff(x);
dy = diff(y);
segment_lengths = sqrt(dx.^2 + dy.^2); % Length of each segment
cumulative_distance = [0; cumsum(segment_lengths)]; % Cumulative distance
% Ensure no duplicate values in cumulative_distance
[cumulative_distance, unique_idx] = unique(cumulative_distance, 'stable');
x = x(unique_idx);
y = y(unique_idx);
% Total distance
total_distance = cumulative_distance(end);
% % Divide into different segments
% equal_distances = linspace(0, total_distance, 200);
% Determine the number of points with a fixed distance of 2
num_points = floor(total_distance / ref_dis) + 1; % Calculate the number of points
% Generate evenly spaced reference points based on the distance
equal_distances = linspace(0, (num_points - 1) * ref_dis, num_points);
% Interpolate trajectory points
Xref = interp1(cumulative_distance, x, equal_distances);
Yref = interp1(cumulative_distance, y, equal_distances);
Xref = Xref(:);
Yref = Yref(:);
% Calculate time to each waypoint t_ref
t_ref = equal_distances / speed;
% Calculate tangent angles
Psiref = [atan2(Yref(2) - Yref(1), Xref(2) - Xref(1)); atan2(Yref(2:end) - Yref(1:end-1), Xref(2:end) - Xref(1:end-1))];
case '9'
% Parameters
radius = 10;
circle_center = [40, 10]; 
straight_start = [0, 0];
straight_end = [40, 0];
% First segment: straight line
x1 = linspace(straight_start(1), straight_end(1));
y1 = zeros(size(x1));
% Second segment: full circle (for a single lap reference)
% The angle for a full circle is 2*pi. To start at the bottom (-pi/2) and complete one lap,
% the angle range is from -pi/2 to -pi/2 + 2pi = 3pi/2.
theta_single_lap = linspace(-pi/2, 3*pi/2, 200); 
x2_single_lap = circle_center(1) + radius * cos(theta_single_lap);
y2_single_lap = circle_center(2) + radius * sin(theta_single_lap);
% Replicate the circular segment for the desired number of laps
x2_multi_lap = [];
y2_multi_lap = [];
for i = 0:(laps - 1)
% Adjust the angle for each lap by adding multiples of 2*pi
current_theta_start = -pi/2 + i * 2*pi;
current_theta_end = 3*pi/2 + i * 2*pi;
theta_current_lap = linspace(current_theta_start, current_theta_end, 200); % Use same number of points per lap
% Calculate coordinates for the current lap
x_current_lap = circle_center(1) + radius * cos(theta_current_lap);
y_current_lap = circle_center(2) + radius * sin(theta_current_lap);
% Append the current lap's coordinates to the overall vectors
x2_multi_lap = [x2_multi_lap, x_current_lap];
y2_multi_lap = [y2_multi_lap, y_current_lap];
end
% Combine segments: straight line + multiple circle laps
x = [x1, x2_multi_lap];
y = [y1, y2_multi_lap];
% Remove duplicates if any (useful at the junction points between straight line and circle, and between laps)
coords = [x', y'];
[unique_coords, idx] = unique(coords, 'rows', 'stable');
x = unique_coords(:, 1);
y = unique_coords(:, 2);
% Recalculate cumulative distance
dx = diff(x);
dy = diff(y);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];
% Evenly spaced reference points
num_points = floor(cumulative_distance(end) / ref_dis) + 1;
equal_distances = linspace(0, cumulative_distance(end), num_points);
% Interpolate to get reference points on the trajectory
Xref = interp1(cumulative_distance, x, equal_distances)';
Yref = interp1(cumulative_distance, y, equal_distances)';
% Calculate heading
Psiref = [atan2(Yref(2)-Yref(1), Xref(2)-Xref(1));
atan2(diff(Yref), diff(Xref))];
% Calculate time
t_ref = equal_distances' / Vref_test; 
case '10'
% Parameters
radius_circle = 10;
radius_transition = 20;
circle_center = [40, 10];
straight_start = [0, 0];
straight_end = [40, 0];
% First segment: straight line
x1 = linspace(straight_start(1), straight_end(1), 100);
y1 = zeros(size(x1));
% Smooth transition arc: quarter circle from straight_end to circle start
theta_transition = linspace(-pi/2, 0, 50); % from pointing right to pointing up
x2 = straight_end(1) + radius_transition * cos(theta_transition);
y2 = straight_end(2) + radius_transition * sin(theta_transition);
% Actual circle laps starting at the top of the transition arc
theta_circle = linspace(pi, pi + 2*pi*laps, 200*laps);
x3 = circle_center(1) + radius_circle * cos(theta_circle);
y3 = circle_center(2) + radius_circle * sin(theta_circle);
% Now correctly combine
x = [x1, x2, x3];
y = [y1, y2, y3];
% Remove duplicates
coords = [x', y'];
[unique_coords, idx] = unique(coords, 'rows', 'stable');
x = unique_coords(:, 1);
y = unique_coords(:, 2);
% Recalculate cumulative distance
dx = diff(x);
dy = diff(y);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];
% Evenly spaced reference points
num_points = floor(cumulative_distance(end) / ref_dis) + 1;
equal_distances = linspace(0, cumulative_distance(end), num_points);
% Interpolate
Xref = interp1(cumulative_distance, x, equal_distances)';
Yref = interp1(cumulative_distance, y, equal_distances)';
% Heading
Psiref = [atan2(Yref(2)-Yref(1), Xref(2)-Xref(1));
atan2(diff(Yref), diff(Xref))];
% Time
t_ref = equal_distances' / Vref_test;
case '11'
% Parameters
radius = 10;
circle_center = [40, 10]; 
% Generate the circular segment for the desired number of laps
x_circle = [];
y_circle = [];
for i = 0:(laps - 1)
theta_start = -pi/2 + i * 2*pi;
theta_end = 3*pi/2 + i * 2*pi;
theta_current_lap = linspace(theta_start, theta_end, 200); % 200 points per lap
x_current_lap = circle_center(1) + radius * cos(theta_current_lap);
y_current_lap = circle_center(2) + radius * sin(theta_current_lap);
x_circle = [x_circle, x_current_lap];
y_circle = [y_circle, y_current_lap];
end
% Clean up duplicate coordinates at lap transitions if any
coords = [x_circle', y_circle'];
[unique_coords, idx] = unique(coords, 'rows', 'stable');
x = unique_coords(:, 1);
y = unique_coords(:, 2);
% Recalculate cumulative distance
dx = diff(x);
dy = diff(y);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];
% Evenly spaced reference points
num_points = floor(cumulative_distance(end) / ref_dis) + 1;
equal_distances = linspace(0, cumulative_distance(end), num_points);
% Interpolate to get reference points
Xref = interp1(cumulative_distance, x, equal_distances)';
Yref = interp1(cumulative_distance, y, equal_distances)';
% Calculate heading
Psiref = [atan2(Yref(2)-Yref(1), Xref(2)-Xref(1));
atan2(diff(Yref), diff(Xref))];
% Calculate time
t_ref = equal_distances' / Vref_test;
case '12'
L_straight = 10; % meters
R_circle = 10; % meters
L_clothoid = 10; % meters
theta_circle = 4*pi; % 2 laps
ref_dis = 0.1;
% straight segment
x_straight = linspace(0, L_straight, 100);
y_straight = zeros(size(x_straight));
% clothoid entry
s_clothoid = linspace(0, L_clothoid, 100);
curvature_max = 1/R_circle;
curvature_clothoid = curvature_max * (s_clothoid/L_clothoid);
heading_clothoid = cumtrapz(s_clothoid, curvature_clothoid);
x_clothoid = cumtrapz(s_clothoid, cos(heading_clothoid)) + x_straight(end);
y_clothoid = cumtrapz(s_clothoid, sin(heading_clothoid)) + y_straight(end);
% circle path
theta_arc = linspace(heading_clothoid(end), heading_clothoid(end)+theta_circle, 500);
theta_arc = theta_arc(1:end-1); % remove repeated endpoint
xc_center = x_clothoid(end) - R_circle * sin(heading_clothoid(end));
yc_center = y_clothoid(end) + R_circle * cos(heading_clothoid(end));
x_circle = xc_center + R_circle * sin(theta_arc);
y_circle = yc_center - R_circle * cos(theta_arc);
% combine
x_path = [x_straight, x_clothoid, x_circle];
y_path = [y_straight, y_clothoid, y_circle];
% remove duplicate consecutive points
coords = [x_path(:), y_path(:)];
[unique_coords, ia] = unique(coords,'rows','stable');
x_path = unique_coords(:,1);
y_path = unique_coords(:,2);
% resample to uniform distance
dx = diff(x_path);
dy = diff(y_path);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];
% fix for duplicate sample points:
[cumulative_distance, unique_idx] = unique(cumulative_distance, 'stable');
x_path = x_path(unique_idx);
y_path = y_path(unique_idx);
equal_distances = linspace(0, cumulative_distance(end), floor(cumulative_distance(end)/ref_dis)+1);
Xref = interp1(cumulative_distance, x_path, equal_distances, 'linear');
Yref = interp1(cumulative_distance, y_path, equal_distances, 'linear');
% heading
heading_diff = atan2(diff(Yref), diff(Xref));
heading_diff = heading_diff(:);
Psiref = [heading_diff(1); heading_diff];
t_ref = equal_distances' / Vref_test;
case '13' %even smoother version of 12 double the transition length
L_straight = 10; % meters
R_circle = 10; % meters
L_clothoid = 20; % meters (increased for smoother transition)
theta_circle = laps * 2*pi; % how many laps we want...
ref_dis = 0.1;
% straight segment
x_straight = linspace(0, L_straight, 100);
y_straight = zeros(size(x_straight));
% clothoid entry
s_clothoid = linspace(0, L_clothoid, 200); % more points for longer length
curvature_max = 1/R_circle;
curvature_clothoid = curvature_max * (s_clothoid/L_clothoid);
heading_clothoid = cumtrapz(s_clothoid, curvature_clothoid);
x_clothoid = cumtrapz(s_clothoid, cos(heading_clothoid)) + x_straight(end);
y_clothoid = cumtrapz(s_clothoid, sin(heading_clothoid)) + y_straight(end);
% circle path
theta_arc = linspace(heading_clothoid(end), heading_clothoid(end) + theta_circle, 500);
theta_arc = theta_arc(1:end-1); % remove repeated endpoint
xc_center = x_clothoid(end) - R_circle * sin(heading_clothoid(end));
yc_center = y_clothoid(end) + R_circle * cos(heading_clothoid(end));
x_circle = xc_center + R_circle * sin(theta_arc);
y_circle = yc_center - R_circle * cos(theta_arc);
% combine
x_path = [x_straight, x_clothoid, x_circle];
y_path = [y_straight, y_clothoid, y_circle];
% remove duplicate consecutive points
coords = [x_path(:), y_path(:)];
[unique_coords, ia] = unique(coords, 'rows', 'stable');
x_path = unique_coords(:,1);
y_path = unique_coords(:,2);
% resample to uniform distance
dx = diff(x_path);
dy = diff(y_path);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];
% fix for duplicate sample points
[cumulative_distance, unique_idx] = unique(cumulative_distance, 'stable');
x_path = x_path(unique_idx);
y_path = y_path(unique_idx);
equal_distances = linspace(0, cumulative_distance(end), floor(cumulative_distance(end)/ref_dis)+1);
Xref = interp1(cumulative_distance, x_path, equal_distances, 'linear')';
Yref = interp1(cumulative_distance, y_path, equal_distances, 'linear')';
% heading
heading_diff = atan2(diff(Yref), diff(Xref));
heading_diff = heading_diff(:);
Psiref = [heading_diff(1); heading_diff];
% time vector (requires Vref_test to be defined in workspace)
t_ref = equal_distances' / Vref_test;
% checking to see why it's stopping at around 74 seconds
fprintf('[DEBUG] case 12 trajectory duration: %.2f seconds\n', t_ref(end));


case '14' % even smoother version of 12 double the transition length
L_straight = 3;  % meters (was 10)
R_circle   = 3;  % meters (was 10)
L_clothoid = 20; % meters (kept for smooth transition)
theta_circle = laps * 2*pi; % how many laps we want...
ref_dis = 0.1;

% straight segment
x_straight = linspace(0, L_straight, 100);
y_straight = zeros(size(x_straight));

% clothoid entry
s_clothoid = linspace(0, L_clothoid, 200); % more points for longer length
curvature_max = 1/R_circle;
curvature_clothoid = curvature_max * (s_clothoid/L_clothoid);
heading_clothoid = cumtrapz(s_clothoid, curvature_clothoid);
x_clothoid = cumtrapz(s_clothoid, cos(heading_clothoid)) + x_straight(end);
y_clothoid = cumtrapz(s_clothoid, sin(heading_clothoid)) + y_straight(end);

% circle path
theta_arc = linspace(heading_clothoid(end), heading_clothoid(end) + theta_circle, 500);
theta_arc = theta_arc(1:end-1); % remove repeated endpoint
xc_center = x_clothoid(end) - R_circle * sin(heading_clothoid(end));
yc_center = y_clothoid(end) + R_circle * cos(heading_clothoid(end));
x_circle = xc_center + R_circle * sin(theta_arc);
y_circle = yc_center - R_circle * cos(theta_arc);

% combine
x_path = [x_straight, x_clothoid, x_circle];
y_path = [y_straight, y_clothoid, y_circle];

% remove duplicate consecutive points
coords = [x_path(:), y_path(:)];
[unique_coords, ia] = unique(coords, 'rows', 'stable');
x_path = unique_coords(:,1);
y_path = unique_coords(:,2);

% resample to uniform distance
dx = diff(x_path);
dy = diff(y_path);
segment_lengths = sqrt(dx.^2 + dy.^2);
cumulative_distance = [0; cumsum(segment_lengths)];

% fix for duplicate sample points
[cumulative_distance, unique_idx] = unique(cumulative_distance, 'stable');
x_path = x_path(unique_idx);
y_path = y_path(unique_idx);

equal_distances = linspace(0, cumulative_distance(end), floor(cumulative_distance(end)/ref_dis)+1);
Xref = interp1(cumulative_distance, x_path, equal_distances, 'linear')';
Yref = interp1(cumulative_distance, y_path, equal_distances, 'linear')';

% heading
heading_diff = atan2(diff(Yref), diff(Xref));
heading_diff = heading_diff(:);
Psiref = [heading_diff(1); heading_diff];

% time vector (requires Vref_test to be defined in workspace)
t_ref = equal_distances' / Vref_test;

% debug
fprintf('[DEBUG] case 13 trajectory duration: %.2f seconds\n', t_ref(end));


case '15'   % straight 3 m -> immediately enter circle of R=3 m (no clothoid)

L_straight = 3;              % m
R_circle   = 3;              % m
ref_dis    = 0.1;

% === 短过渡段：长度按 R 的倍数调（这里取 1*R ≈ 3 m）===
k_trans    = 1.0;            % 0.5~1.5 均可；越小越短
L_clothoid = k_trans * R_circle;

% 直线
x_straight = linspace(0, L_straight, 100);
y_straight = zeros(size(x_straight));

% clothoid（更短 + 自适应采样点数）
N_clothoid = max(round(L_clothoid/ref_dis*8), 40); % 足够光滑即可
s_clothoid = linspace(0, L_clothoid, N_clothoid);
curvature_max = 1/R_circle;
curvature_clothoid = curvature_max * (s_clothoid/L_clothoid);
heading_clothoid = cumtrapz(s_clothoid, curvature_clothoid);
x_clothoid = cumtrapz(s_clothoid, cos(heading_clothoid)) + x_straight(end);
y_clothoid = cumtrapz(s_clothoid, sin(heading_clothoid)) + y_straight(end);

% 圆
theta_circle = laps * 2*pi;  % 工作区需有 laps
theta_arc = linspace(heading_clothoid(end), heading_clothoid(end)+theta_circle, 500);
theta_arc = theta_arc(1:end-1);
xc_center = x_clothoid(end) - R_circle * sin(heading_clothoid(end));
yc_center = y_clothoid(end) + R_circle * cos(heading_clothoid(end));
x_circle = xc_center + R_circle * sin(theta_arc);
y_circle = yc_center - R_circle * cos(theta_arc);

% 合并
x_path = [x_straight, x_clothoid, x_circle];
y_path = [y_straight, y_clothoid, y_circle];

% 去重 + 按弧长等距重采样
coords = [x_path(:), y_path(:)];
[unique_coords, ~] = unique(coords, 'rows', 'stable');
x_path = unique_coords(:,1); y_path = unique_coords(:,2);
dxy = diff([x_path y_path]); seglen = hypot(dxy(:,1), dxy(:,2));
s = [0; cumsum(seglen)];
[s, idx] = unique(s,'stable'); x_path = x_path(idx); y_path = y_path(idx);
s_eq = linspace(0, s(end), floor(s(end)/ref_dis)+1);
Xref = interp1(s, x_path, s_eq, 'linear')';
Yref = interp1(s, y_path, s_eq, 'linear')';

% 航向
psi = atan2(diff(Yref), diff(Xref)); psi = psi(:);
Psiref = [psi(1); psi];

% 时间（需有 Vref_test）
t_ref = s_eq' / Vref_test;

fprintf('[DEBUG] case 14 trajectory duration: %.2f s (L_clothoid=%.2f m)\n', t_ref(end), L_clothoid);

case '16' 
% Generate a straight-line trajectory with the function (ReferenceGenerator)
[Xref,Yref,Psiref] = ReferenceGenerator('sharp_turn',ref_dis,lL,laps);
% Time vector calculated based on trajectory length and speed
t_ref = linspace(0, (lL-1)/Vref_test, lL)';

case '17' 
% Generate a straight-line trajectory with the function (ReferenceGenerator)
[Xref,Yref,Psiref] = ReferenceGenerator('step',ref_dis,lL,laps);
% Time vector calculated based on trajectory length and speed
t_ref = linspace(0, (lL-1)/Vref_test, lL)';

end
end