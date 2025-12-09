function Animate_MPC(Results)

N = evalin('base', 'mpc_outer_params.N');
vv = evalin('base', 'vv');
MPC_Ts = evalin('base', 'MPC_Ts');

X_est  = Results.X_est.Data;      % Nsteps × 1
Y_est  = Results.Y_est.Data;      % Nsteps × 1
PSI_est = Results.PSI_est.Data;   % Nsteps × 1
MPC_saved_states = Results.MPC_states.Data;

% Extract lateral error states (local y prediction)
e1_states = MPC_saved_states(:,1:2:end-1);   % Nsteps × N

% Local x prediction (forward along vehicle axis)
x_local = vv * (1:N) * MPC_Ts;               % 1 × N
x_local = repmat(x_local, size(e1_states,1), 1);   % Nsteps × N

% Local y prediction
y_local = e1_states - e1_states(:,1);                          % Nsteps × N

Nsteps = size(x_local,1);

% Preallocate global coordinates
x_global = zeros(Nsteps, N);
y_global = zeros(Nsteps, N);

% --- Convert local → global for all steps ---
for k = 1:Nsteps
    psi = PSI_est(k);

    % Rotation matrix:
    % [ cos(psi) -sin(psi);
    %   sin(psi)  cos(psi) ]
    if k == 2036
        1;
    end
    xg = x_local(k,:) * cos(psi) - y_local(k,:) * sin(psi);
    yg = x_local(k,:) * sin(psi) + y_local(k,:) * cos(psi);

    % Translation by the vehicle's global position
    x_global(k,:) = X_est(k) + xg;
    y_global(k,:) = Y_est(k) + yg;
end

% ----------- Create figure ----------------
fig = figure('Name','MPC Global Prediction Viewer');
hold on;
axis equal;
xlabel('X position'); ylabel('Y position');
title('MPC Predicted Global Path');

% Plot predicted horizon
hPlot = plot(x_global(1,:), y_global(1,:), 'LineWidth', 2);

% Add car marker at the first prediction point
hCar = plot(x_global(1,1), y_global(1,1), 'ro', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'r');

xlim([min(x_global(:)-2) max(x_global(:))+2]);
ylim([min(y_global(:)-2) max(y_global(:))+2]);


% Slider
hSlider = uicontrol('Style', 'slider', ...
    'Min', 1, 'Max', Nsteps, 'Value', 1, ...
    'SliderStep', [1/(Nsteps-1), 10/(Nsteps-1)], ...
    'Units', 'normalized', ...
    'Position', [0.2 0.02 0.6 0.05], ...
    'Callback', @updatePlot);



% Text label
hText = uicontrol('Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.82 0.02 0.15 0.05], ...
    'String', 'Step: 1');

% Store variables
data.x_global = x_global;
data.y_global = y_global;
data.hPlot = hPlot;
data.hCar = hCar;
data.hText = hText;


guidata(fig, data);

% Overlay it on the actual track
x_trajectory = evalin('base', 'Xref');
y_trajectory = evalin('base', 'Yref');

plot(x_trajectory, y_trajectory)
plot(X_est, Y_est)

end

% ------------ Callback ------------
function updatePlot(src, ~)
    fig = ancestor(src, 'figure');
    data = guidata(fig);

    k = round(src.Value);

    % Update XY trajectory line
    set(data.hPlot, ...
        'XData', data.x_global(k,:), ...
        'YData', data.y_global(k,:));

    % Update car marker (first point)
    set(data.hCar, ...
        'XData', data.x_global(k,1), ...
        'YData', data.y_global(k,1));

    % Update text
    set(data.hText, 'String', sprintf('Step: %d', k));
end
