set(0,'defaulttextinterpreter','none');
clear all; 
clear;
close all;
clc;

data_path = 'data_calibretion.csv';  
data_lab = readtable(data_path);  


%% -------- Read columns--------
roll_rate_b = data_lab.GyroscopeX_rad_s_(:).';   % 1×N
pitch_rate_b = data_lab.GyroscopeY_rad_s_(:).';
yaw_rate_b = data_lab.GyroscopeZ_rad_s_(:).';

X_accelerometer_b = data_lab.AccelerometerX_rad_s_2_(:).';  
Y_accelerometer_b = data_lab.AccelerometerY_rad_s_2_(:).';
Z_accelerometer_b = data_lab.AccelerometerZ_rad_s_2_(:).';

figure()
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-251525), 180/pi * roll_rate_b, 'r')
hold on
% ylim([-30 10])
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value about the X-axis of Bike frame vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-251525), 180/pi * pitch_rate_b, 'r')
hold on
% ylim([-2 6])
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value about the Y-axis of Bike frame vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-251525), 180/pi * yaw_rate_b, 'r')
hold on
% ylim([-10 25])
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value about the Z-axis of Bike frame vs Time')
grid on

figure()
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-251525), X_accelerometer_b, 'r')
hold on
% ylim([-30 10])
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Acceleration value about the X-axis of Bike body frame vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-251525), Y_accelerometer_b, 'r')
hold on
% ylim([-2 6])
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Acceleration value about the Y-axis of Bike body frame vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-251525), Z_accelerometer_b, 'r')
hold on
% ylim([-10 25])
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Acceleration value about the Z-axis of Bike body frame vs Time')
grid on


%% ===== Params =====
dt = 0.05;  
g  = 9.8173;            % Gothenburg 9.8173
k0 = 1;
idx = k0:numel(X_accelerometer_b);
t   = (0:numel(idx)-1)' * dt;

% body-frame specific force (raw accelerometer)
ax_b = X_accelerometer_b(idx);
ay_b = Y_accelerometer_b(idx);
az_b = Z_accelerometer_b(idx);

ax_b = ax_b(:);
ay_b = ay_b(:);
az_b = az_b(:);

f_b  = [ax_b, ay_b, az_b];           % N×3

% ---------- Static window: first ~2 s ----------
K0 = max(1, min(round(4.0/dt), size(f_b,1)));
fb_mean = mean(f_b(1:K0, :), 1, 'omitnan').';   % body-frame mean specific force
% === A 方案：起始直立静止 ===
phi0   = 0;                        % roll init
theta0 = 0;                        % pitch init
psi0   = 0;                        % yaw 可置 0 或用磁/GNSS

% accel bias from upright prior: fb_mean ≈ [0;0;g] + b_a
ba_hat = fb_mean - [0;0;g];        % 3×1

% ---- gyro bias removal as before ----
p = roll_rate_b(:); q = pitch_rate_b(:); r = yaw_rate_b(:);
K = min(round(2.0/dt), numel(p));

% p = p - mean(p(1:K));
% q = q - mean(q(1:K));
% r = r - mean(r(1:K));

% ---- integrate Euler (ZYX) with correct initial angles ----
N = numel(p);
inte_roll  = zeros(N,1); inte_pitch = zeros(N,1); inte_yaw = zeros(N,1);
inte_roll(1)=phi0; inte_pitch(1)=theta0; inte_yaw(1)=psi0;

for k = 2:N
    phi = inte_roll(k-1); th = inte_pitch(k-1);
    T = [ 1,  sin(phi)*tan(th),  cos(phi)*tan(th);
          0,  cos(phi),         -sin(phi);
          0,  sin(phi)/cos(th),  cos(phi)/cos(th) ];
    eul_dot = T * [p(k); q(k); r(k)];
    inte_roll(k)  = phi + eul_dot(1)*dt;
    inte_pitch(k) = th  + eul_dot(2)*dt;
    inte_yaw(k)   = inte_yaw(k-1) + eul_dot(3)*dt;
end

% inte_roll = cumtrapz(roll_rate_b) * dt;
% inte_pitch = cumtrapz(pitch_rate_b) * dt;
% inte_yaw = cumtrapz(yaw_rate_b) * dt;


fig = figure();
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-251525), 180/pi * inte_roll, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('Roll angle from integration of measured roll rate vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-251525), 180/pi * inte_pitch, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('Pitch angle from integration of measured pitch rate vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-251525), 180/pi * inte_yaw, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
ylim([-10 400]); 
title('Yaw angle from integration of measured yaw rate vs Time')
grid on

% ---- per-sample: remove bias -> rotate -> add gravity ----
g_n  = [0;0;-g];
acc_e = zeros(numel(idx),3);
for k = 1:numel(idx)
    phi = inte_roll(k); th = inte_pitch(k); psi = inte_yaw(k);
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Cbn = Rz*Ry*Rx;                  % b -> n

    f_b_corr = f_b(k,:).' - ba_hat;  % remove accel bias in body
    f_n      = Cbn * f_b_corr;       % rotate specific force to ENU
    a_n      = f_n + g_n;            % linear acceleration (ENU)
    acc_e(k,:) = a_n.';
end


figure('Name','Linear acceleration (ENU)'); 

subplot(3,1,1);
plot(0.001 * (data_lab.Time_ms_-251525), acc_e(:,1), 'r'); grid on;
ylabel('a_E (m/s^2)');
title('East (X) linear acceleration vs Time');

subplot(3,1,2);
plot(0.001 * (data_lab.Time_ms_-251525), acc_e(:,2), 'r'); grid on;
ylabel('a_N (m/s^2)');
title('North (Y) linear acceleration vs Time');

subplot(3,1,3);
plot(0.001 * (data_lab.Time_ms_-251525), acc_e(:,3), 'r'); grid on;
ylabel('a_U (m/s^2)');
xlabel('Time (s)');
title('Up (Z) linear acceleration vs Time');



%% ===== Integration: acc -> vel =====
vx = cumtrapz(t, acc_e(:,1));
vy = cumtrapz(t, acc_e(:,2));

k20 = find(t >= 17, 1, 'first');
if ~isempty(k20)
    offset = vx(k20);
    vx(k20:end) = vx(k20:end) - offset;
    x = cumtrapz(t, vy);

    offset = vy(k20);
    vy(k20:end) = vy(k20:end) - offset;
    y = cumtrapz(t, vy);
end

% --- Plot vE (= vx) and vN (= vy) ---
figure('Name','Velocity (ENU)');

subplot(2,1,1);
plot(t, vx, 'r'); grid on; hold on;
xline(17, '--', 'zero-velocity constraint ', 'LabelVerticalAlignment','bottom');
ylabel('v_E (m/s)');
title('East (E) Velocity vs Time');

subplot(2,1,2);
plot(t, vy, 'r'); grid on; hold on;
xline(17, '--', 'zero-velocity constraint ', 'LabelVerticalAlignment','bottom');
ylabel('v_N (m/s)');
xlabel('Time (s)');
title('North (N) Velocity vs Time');


x  = cumtrapz(t, vx);
y  = cumtrapz(t, vy);





% ---- keep only samples up to 52 s for plotting ----
mask = (t >= 18) & (t <= 52);
x_plot = x(mask);
y_plot = y(mask);

% --- rebase: make the position at 18 s the origin (0,0) ---
if ~isempty(x_plot)
    x_plot = x_plot - x_plot(1);
    y_plot = y_plot - y_plot(1);
end

figure; hold on; grid on; axis equal;
plot(x_plot, y_plot, '-', 'Color', [0 0.2 0.9], 'LineWidth', 1.6);
plot(x_plot, y_plot, 'o', 'MarkerSize', 4, ...
     'MarkerEdgeColor', [0 0.2 0.9], 'MarkerFaceColor', 'w');
plot(x_plot(1),  y_plot(1),  'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'DisplayName','start');
plot(x_plot(end), y_plot(end), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName','end');
xlabel('x_{local} (m)'); ylabel('y_{local} (m)');
title('Trajectory with per-sample circle markers');
legend('trajectory','samples','start','end','Location','best');



fig = figure();
subplot(3,1,1)
plot(acc_e(:,1), 'r')
hold on
xlabel('Sample points')
ylabel('accelerater')
title('Acceleration value in Local coordinate vs Time')
grid on
subplot(3,1,2)
plot(acc_e(:,2), 'r')
hold on
xlabel('Sample points')
ylabel('accelerater')
title('Acceleration value in Local coordinate vs Time')
grid on
subplot(3,1,3)
plot(acc_e(:,3), 'r')
hold on
xlabel('Sample points')
ylabel('accelerater')
title('Acceleration value in Local coordinate vs Time')
grid on























