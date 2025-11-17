set(0,'defaulttextinterpreter','none');
clear all; 
clear;
close all;
clc;

data_path = 'data_test_corridor.csv';  
% data_test_corridor   data_transformatrix7   data_test_corridor_green   
data_lab = readtable(data_path);  

% data_lab = data_lab(1:100, :);

data_lab.GyroscopeX_rad_s_ = 1 * data_lab.GyroscopeX_rad_s_;
data_lab.SteeringAngleEncoder_rad_ = 1 * data_lab.SteeringAngleEncoder_rad_;

% PURPOSE
%   This script is used to check sensor data.
%   Throughout the test, both the balancing controller and the speed
%   controller remain DISABLED.

% DATA GENERATION
%   1) Bike held upright (stationary).
%   2) Left tilt and return to upright.
%   3) Steer the handlebar left and return to center.
%   4) Push the bike forward to move straight.
%   5) Execute a left circular turn while moving.

% CHECKS:
%   1) Roll rate sensor: sign and value
%   2) Steering angle sensor: sign and value


% sampling time of logging data and frequency
dt = 0.05; fs = 1/dt;
% ------------------------------------------------------------------------------------
% ------------------------Parameters for checking IMU data--------------------
% The parameters are given subjectively.

% --- roll rate sign check ---
roll_rate_a = 0;   % LEFT tilt: mean should be > 0, roll rate
roll_rate_b = 0;   % RIGHT tilt: mean should be < 0, roll rate

% --- roll rate range check (only if polarity is OK)  ---
% The parameters are given subjectively.
mag_min_degps = 1;  % too small if below this
mag_max_degps = 10;   % too large if above this

% Params
th_on  = 1;      % Start threshold: > 1 degree/s
th_off = 1;      % End threshold: < 1 degree/s
N_on   = 10;     % Start requires 10 consecutive samples
N_off  = 10;     % End requires 10 consecutive samples


% ------------------------------------------------------------------------------------
% ------------------------Parameters for checking Steering angle data--------------------
% The parameters are given subjectively.
% Params
th_on_sa  = 10;    % Start threshold: > 10 deg
th_off_sa = 10;    % End threshold:   < 10 deg
N_on_sa   = 10;    % Start requires 10 consecutive samples
N_off_sa  = 10;    % End requires 10 consecutive samples
max_steering_angle  = 60;    % maximum steering angle 


disp('1. The following tests are conducted while bike is stationary:');

% ------------------------------------------------------------------------------------
% ------------------------check measured roll rate signal from IMU--------------------

%% check the sign of roll rate  from measured  from IMU when LEFT tilt

% roll_rate_idx : roll-rate during tilt (deg/s)
roll_rate_idx = abs(rad2deg(data_lab.GyroscopeX_rad_s_)); % convert to deg/s

N = numel(roll_rate_idx);
J = (1:N)';      % Index vector

% ---- Start: first i s.t. roll_rate_idx(i:i+N_on-1) > th_on ----
run_on = movsum(roll_rate_idx > th_on, [0, N_on-1]);   % Look-ahead window count
last_pos_on = max(0, N - N_on + 1);                     % Allow 1:0 empty slice
tmp = [find(run_on(1:last_pos_on) == N_on, 1, 'first') NaN];
idx_on = tmp(1);                                        % NaN if not found

% ---- End: from idx_on+N_on, first j s.t. roll_rate_idx(j:j+N_off-1) < th_off ----
run_off = movsum(roll_rate_idx < th_off, [0, N_off-1]);
off_candidates = (run_off == N_off) & (J >= (idx_on + N_on)) & (J <= (N - N_off + 1));
tmp = [find(off_candidates, 1, 'first') NaN];
idx_off = tmp(1);                                       % NaN if not found

% ---- Segment for mean: [idx_on, idx_off-1]; if no idx_off, use end of series ----
mask_seg = (J >= idx_on) & (isnan(idx_off) | (J < idx_off));
roll_rate_mL = mean(rad2deg(data_lab.GyroscopeX_rad_s_(mask_seg)), 'omitnan');  % mean (deg/s)

% Time (s)
t_on  = (idx_on - 1) * dt;                               % Start time (NaN = not triggered)
last_incl = [find(mask_seg, 1, 'last') N];               % If none, default N (mean unaffected)
last_incl = last_incl(1);
t_off = (last_incl - 1) * dt;                            % End time (if none, end of series)

% --- Polarity check (LEFT tilt) ---

if (roll_rate_mL > roll_rate_a)
    disp('IMU sign is OK during left tilt');
    
    roll_rate_left_mag = median(abs(roll_rate_idx), 'omitnan');

    if roll_rate_left_mag < mag_min_degps
        disp('Roll rate during a left tilt is too small');
    elseif roll_rate_left_mag > mag_max_degps
        disp('Roll rate during a left tilt is too large');
    else
        disp('Roll rate during a left tilt is within a reasonable range');
    end
else
    disp('IMU sign mismatch during left tilt');
    % (skip magnitude check)
end


%% Check IMU polarity during RIGHT tilt (then magnitude only if polarity OK)
% === Second segment (scan after first off-run) ===
s2_start = idx_off + N_off;                                      % search start

% start2: first i >= s2_start with 10-in-a-row > th_on
start_candidates2 = (run_on == N_on) & (J >= s2_start) & (J <= (N - N_on + 1));
tmp = [find(start_candidates2, 1, 'first') NaN];
idx_on2 = tmp(1);                                                % NaN if none

% end2: first j >= idx_on2+N_on with 10-in-a-row < th_off
off_candidates2 = (run_off == N_off) & (J >= (idx_on2 + N_on)) & (J <= (N - N_off + 1));
tmp = [find(off_candidates2, 1, 'first') NaN];
idx_off2 = tmp(1);                                               % NaN if none

% segment2 mean: [idx_on2, idx_off2-1]; if no idx_off2, use end of series
mask_seg2 = (J >= idx_on2) & (isnan(idx_off2) | (J < idx_off2));
roll_rate_mR = mean(rad2deg(data_lab.GyroscopeX_rad_s_(mask_seg2)), 'omitnan');  % mean (deg/s)

% times (s)
t_on2  = (idx_on2 - 1) * dt;                                    
last_incl2 = [find(mask_seg2, 1, 'last') N];                     
last_incl2 = last_incl2(1);
t_off2 = (last_incl2 - 1) * dt;


if roll_rate_mR < roll_rate_b
    disp('IMU sign is OK during right tilt');

    % --- Magnitude check (only if polarity is OK) ---
    roll_rate_right_mag = median(abs(roll_rate_mR), 'omitnan');

    if roll_rate_right_mag < mag_min_degps
        disp('Roll rate during a right tilt is too small');
    elseif roll_rate_right_mag > mag_max_degps
        disp('Roll rate during a right tilt is too large');
    else
        disp('Roll rate during a right tilt is within a reasonable range');
    end
else
    disp('IMU sign mismatch during right tilt');
    % (skip magnitude check)
end



% ------------------------------------------------------------------------------------
% ------------------------check steerng angle signal from sensor----------------------

%% Check steering angle polarity during LEFT turn, then magnitude only if polarity OK

% steering_angle_measured : steering angle (rad)
% steering_angle_ref      : reference steering angle (rad) for LEFT turn (optional)
% steer_idxL              : logical indices for LEFT-turn window
% steer_a                 : polarity threshold (deg), set per your sign convention

% Convert to degrees (skip if already in deg)
steering_angle_deg = rad2deg(data_lab.SteeringAngleEncoder_rad_);

% --- Steering angle segment (simple, same pattern as your roll-rate code) ---

N = numel(steering_angle_deg);
J = (1:N)';        % Index vector

% ---- Start: first i s.t. steering_angle_deg(i:i+N_on_sa-1) > th_on_sa ----
run_on_sa     = movsum(steering_angle_deg > th_on_sa, [0, N_on_sa-1]); % look-ahead count
last_pos_on_sa = max(0, N - N_on_sa + 1);                               % allow 1:0 empty slice
tmp = [find(run_on_sa(1:last_pos_on_sa) == N_on_sa, 1, 'first') NaN];
idx_on_sa = tmp(1);                                                      % NaN if not found

% ---- End: from idx_on_sa+N_on_sa, first j s.t. steering_angle_deg(j:j+N_off_sa-1) < th_off_sa ----
run_off_sa = movsum(steering_angle_deg < th_off_sa, [0, N_off_sa-1]);
off_candidates_sa = (run_off_sa == N_off_sa) & ...
                    (J >= (idx_on_sa + N_on_sa)) & (J <= (N - N_off_sa + 1));
tmp = [find(off_candidates_sa, 1, 'first') NaN];
idx_off_sa = tmp(1);                                                     % NaN if not found

% ---- Segment for stats: [idx_on_sa, idx_off_sa-1]; if no idx_off_sa, use end of series ----
mask_seg_sa = (J >= idx_on_sa) & (isnan(idx_off_sa) | (J < idx_off_sa));
steer_mean_deg = mean(rad2deg(data_lab.SteeringAngleEncoder_rad_(mask_seg_sa)), 'omitnan');  % mean (deg)
steer_max_deg  = max(steering_angle_deg(mask_seg_sa), [], 'omitnan');    % max (deg)

% Times (s) if you have dt defined
t_on_sa  = (idx_on_sa - 1) * dt;                                         % start time
last_incl_sa = [find(mask_seg_sa, 1, 'last') N];
last_incl_sa = last_incl_sa(1);
t_off_sa = (last_incl_sa - 1) * dt;                                      % end time


% --- Checks: sign & max ---
steer_sign_ok = ~isnan(steer_mean_deg) && (steer_mean_deg > th_on_sa);

if steer_sign_ok
    disp('Measured steering angle sign OK');
    if ~isnan(steer_max_deg) && (steer_max_deg > max_steering_angle)
        fprintf('Steering max too large');
    else
        fprintf('Measured maximum steering angle OK: %.2f deg \n', steer_max_deg);
    end
else
    disp('Steering angle polarity mismatch');
end




disp('2. The following tests are conducted while bike is in motion:');
%% ===== NEXT LEFT-tilt roll-rate segment (3rd, forward-only from idx_off2) =====
if isnan(idx_off2)
    warning('No idx_off2 found; cannot continue forward search for the 3rd segment.');
else
    % Start search after the 2nd segment
    s3_start = min(N - N_on + 1, idx_off2 + N_off);

    % Reuse run_on / run_off / J / N from previous calculation
    start_candidates3 = (run_on == N_on) & (J >= s3_start) & (J <= (N - N_on + 1));
    tmp = [find(start_candidates3, 1, 'first') NaN];
    idx_on3 = tmp(1);                                        % NaN if not found

    off_candidates3 = (run_off == N_off) & ...
                      (J >= (idx_on3 + N_on)) & (J <= (N - N_off + 1));
    tmp = [find(off_candidates3, 1, 'first') NaN];
    idx_off3 = tmp(1);                                       % NaN if not found

    % Segment mask (if no idx_off3, extend to end of data)
    mask_seg3 = (J >= idx_on3) & (isnan(idx_off3) | (J < idx_off3));

    % Signed mean for polarity check; median abs for magnitude
    roll_rate_mL3 = mean(rad2deg(data_lab.GyroscopeX_rad_s_(mask_seg3)), 'omitnan');           
    roll_rate_left_mag3 = median(abs(rad2deg(data_lab.GyroscopeX_rad_s_(mask_seg3))), 'omitnan'); 

    % Time stamps
    t_on3  = (idx_on3 - 1) * dt;
    last_incl3 = [find(mask_seg3, 1, 'last') N]; last_incl3 = last_incl3(1);
    t_off3 = (last_incl3 - 1) * dt;

    % Polarity and magnitude checks
    if (roll_rate_mL3 > roll_rate_a)
        disp('IMU sign is OK during taking a circle');
        if roll_rate_left_mag3 < mag_min_degps
            disp('Roll rate is too small');
        elseif roll_rate_left_mag3 > mag_max_degps
            disp('Roll rate is too large');
        else
            disp('Roll rate is within a reasonable range');
        end
    else
        disp('IMU sign mismatch during taking a circle');
    end
end





%% -------- Read columns--------
gx = data_lab.GyroscopeX_rad_s_(:).';   % 1×N
gy = data_lab.GyroscopeY_rad_s_(:).';
gz = data_lab.GyroscopeZ_rad_s_(:).';

ax = data_lab.AccelerometerX_rad_s_2_(:).';  
ay = data_lab.AccelerometerY_rad_s_2_(:).';
az = data_lab.AccelerometerZ_rad_s_2_(:).';

% % Gyroscope (rad/s)
% gx = [0.01, 0.0, 0.003, 0.474, 0.495, 0.4170, 0.4310, 0.5090, 0.4530, 0.4240];
% gy = [-0.011, -0.009, 0.005, -0.03, 0.02, -0.01, 0.04, -0.01, 0.0, 0.02];
% gz = [0.003, 0.002, 0.004, 0.445, 0.495, 0.46, 0.417, -0.481, -0.467, -0.424];
% 
% % Accelerometer (m/s^2)
% ax = [-5.8931, -5.893, -5.893, -5.7431, -5.7931, -5.8431, -5.8931, -5.9431, -5.8131, -5.9131];
% ay = [-2.9465, -2.9465, -2.9465, -2.8965, -2.9665, -2.9165, -2.9565, -2.9065, -2.9465, -2.9265];
% az = [ 7.2681,  7.2681,  7.2681,  7.1681,  7.3281,  7.2681,  7.3381,  7.2181,  7.2881,  7.2981];

W = [gx; gy; gz];             % 3×N gyroscope (sensor frame)
A = [ax; ay; az];             % 3×N accelerometer (sensor frame)
N = size(W,2);

%% -------- 1) Detect static segment (for gravity direction + gyro bias) --------
w_norm   = vecnorm(W);                          
th_quiet = prctile(w_norm, 15);               
quiet_mask = w_norm <= th_quiet;

A_static = A(:, quiet_mask);
W_static = W(:, quiet_mask);

% Gyroscope bias
bias_gyro = mean(W_static, 2);
W_corr    = W - bias_gyro;
% W_corr    = W;

A_grav = A;

A_staticG = A_grav(:, quiet_mask);

% Gravity direction in the sensor frame (= body +Z_b, Down in FRD)
zhat_s = mean(A_staticG, 2);
zhat_s = zhat_s / norm(zhat_s);
zhat_s = zhat_s;
%% -------- 2) Select samples dominated by roll to estimate +X_b direction --------
w_norm_corr = vecnorm(W_corr);
rel_z = abs(sum(W_corr .* zhat_s, 1)) ./ max(w_norm_corr, 1e-6);  
roll_mask = (w_norm_corr > prctile(w_norm_corr, 60)) & (rel_z < 0.3);
W_roll = W_corr(:, roll_mask);

C = (W_roll * W_roll.') / max(1, size(W_roll,2));   
[V,D] = eig(C); [~, idx] = max(diag(D));
x_tilde = V(:, idx);

x_proj = x_tilde - (zhat_s' * x_tilde) * zhat_s;
xhat_s = x_proj / norm(x_proj);

% If needed: flip sign so that left roll < 0
xhat_s = -xhat_s; 

%% -------- 3) Use cross product to get +Y_b, then assemble rotation matrix --------
yhat_s = cross(zhat_s, xhat_s);   
yhat_s = yhat_s / norm(yhat_s);

R0 = [xhat_s, yhat_s, zhat_s].';
det(R0)
[U,~,V] = svd(R0);
R_bs = U * diag([1 1 sign(det(U*V'))]) * V';


% Quality check
orth_err = norm(R_bs*R_bs.' - eye(3), 'fro');
detR     = det(R_bs);
fprintf('R_bs orth_err=%.2e, det=%.4f\n', orth_err, detR);


% R_bs = [-0.666667313652250  -0.016449183837412  -0.745173883908163
%   -0.006858425546318  -0.999578748083139   0.028200857780769
%   -0.745323859075180   0.023911309701239   0.666273663265818];



%% -------- 4) Transform to body frame --------
W_b = R_bs * W_corr;     % gyro in body frame
A_b = R_bs * A_grav;     % accel (gravity-like) in body frame

roll_rate_b  = W_b(1,:).';   % ωx 
pitch_rate_b = W_b(2,:).';   % ωy 
yaw_rate_b   = W_b(3,:).';   % ωz 

X_accelerometer_b = A_b(1,:).';   % ax (forward)
Y_accelerometer_b = A_b(2,:).';   % ay (Left)
Z_accelerometer_b = A_b(3,:).';   % az (Up)

% Sanity check
g_est = mean(A_b(:, quiet_mask), 2);
fprintf('mean A_b on static ≈ [%.3f, %.3f, %.3f] (expect [0,0,+g])\n', ...
        g_est(1), g_est(2), g_est(3));



dt = 0.05;                            
inte_roll = cumtrapz(roll_rate_b) * dt;
inte_pitch = cumtrapz(pitch_rate_b) * dt;
inte_yaw = cumtrapz(yaw_rate_b) * dt;

fig = figure();
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-915936), 180/pi * roll_rate_b, 'r')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value transformed into the X-axis of body frame vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-915936), 180/pi * pitch_rate_b, 'r')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value transformed into the Y-axis of body frame vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-915936), 180/pi * yaw_rate_b, 'r')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value transformed into the Z-axis of body frame vs Time')
grid on


fig = figure();
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-276354), 180/pi * inte_roll, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('Roll angle from integration of measured roll rate vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-276354), 180/pi * inte_pitch, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('Pitch angle from integration of measured pitch rate vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-276354), 180/pi * inte_yaw, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
ylim([-10 300]); 
title('Yaw angle from integration of measured yaw rate vs Time')
grid on


fig = figure();
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-276354), X_accelerometer_b, 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('Measured acceleration value transformed into the X-axis of body frame vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-276354), Y_accelerometer_b, 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('Measured acceleration value transformed into the Y-axis of body frame vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-276354), Z_accelerometer_b, 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('Measured acceleration value transformed into the Z-axis of body frame vs Time')
grid on




%% ===== Params =====
g    = 9.81;              % gravity (m/s^2)
k0   = 1;                 % first index where motion starts (including initial static phase)

%% ===== Trim data from k0 =====
Nall = numel(X_accelerometer_b);
idx  = k0:Nall;
t    = (0:numel(idx)-1)' * dt;    

% Body-frame accelerations (Forward-Left-Up)
ax_b = X_accelerometer_b(idx);
ay_b = Y_accelerometer_b(idx);
az_b = Z_accelerometer_b(idx);

% Euler angles (rad), ZYX convention
roll  = inte_roll(idx);
pitch = inte_pitch(idx);
yaw   = inte_yaw(idx);             % in rad

N = numel(idx);

%% ===== Body -> Earth (ENU), subtract gravity =====
acc_e = zeros(N,3);   % [ax_e, ay_e, az_e]
for k = 1:N
    phi = roll(k); th = pitch(k); psi = yaw(k);

    % Active rotation: v_e = Rz(psi)*Ry(th)*Rx(phi)*v_b
    Rz = [ cos(psi) -sin(psi) 0;
           sin(psi)  cos(psi) 0;
                 0         0  1];
    Ry = [ cos(th)  0  sin(th);
                 0  1       0;
           -sin(th) 0  cos(th)];
    Rx = [ 1    0          0;
           0  cos(phi) -sin(phi);
           0  sin(phi)  cos(phi)];
    Rb2e = Rz * Ry * Rx;

    a_e = Rb2e * [ax_b(k); ay_b(k); az_b(k)];

    acc_e(k,:) = (a_e - [0;0;g]).';  
end

%% ===== Earth -> LOCAL (align initial heading to +x) =====
psi0 = yaw(1);  
R_e2l = [ cos(-psi0) -sin(-psi0) 0;
          sin(-psi0)  cos(-psi0) 0;
                   0           0 1 ];
acc_l = (R_e2l * acc_e.').';    % N×3



M = round(2.0/dt);
bias_xy = mean(acc_l(1:M,1:2), 1);
acc_l(:,1:2) = acc_l(:,1:2) - bias_xy;


%% ===== Integration: acc -> vel =====
vx = cumtrapz(t, acc_l(:,1));
vy = cumtrapz(t, acc_l(:,2));

%% ===== Second integration: vel -> pos =====
x = cumtrapz(t, vx);
y = cumtrapz(t, vy);

%% ===== Plot =====
figure; hold on; grid on; axis equal;
plot(x, y, 'b-', 'LineWidth', 1.6);
plot(x(end), y(end), 'ro', 'MarkerSize', 6, 'LineWidth', 1.2); % final point
xlabel('x_{local} (m)'); ylabel('y_{local} (m)');
legend('trajectory','final point','Location','best');







