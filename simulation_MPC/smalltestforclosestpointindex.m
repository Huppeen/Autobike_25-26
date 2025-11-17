
figure;

% 左侧纵轴：速度（参考值 + 估计值）
yyaxis left
time = (0:length(out.bbb)-1) * 0.01;  % 假设采样周期为 0.01s，如有实际时间向量请替换
yline(2, '--k', 'Reference Velocity', 'LabelHorizontalAlignment', 'left');  % 恒为2的参考线
hold on
plot(out.bbb(:,1), 'b', 'LineWidth', 1.5);  % Velocity from estimator
ylabel('Velocity (m/s)')

% 右侧纵轴：PI 电流
yyaxis right
plot(out.ccc(:,1), 'r', 'LineWidth', 1.5);  % PI Current
ylabel('Current (A)')

% 公共设置
xlabel('Time [s]')
legend('Reference Velocity', 'Velocity from Estimator', 'PI Current', 'Location', 'best')
title('Velocity and PI Current vs Time')
grid on



plot(out.aaa(:,1));
hold on
plot(out.bbb(:,1));
hold on
plot(out.ccc(:,1));
hold on
legend('Reference Velocity','Velocity from Estimator','PI Current');
xlabel('Time [s]');
ylabel('Position X [m]');
grid on;
title('X-coordinate');




data = readmatrix('data_outside2_1.csv');  

latitude_rad = data(:, 6);
longitude_rad = data(:, 7);

Time = data(:, 7);


X_est = data(:, 17);  % Estimate X
Y_est = data(:, 18);  % Estimate Y


RADIUS_OF_THE_EARTH = 6371000.0;


latitude0 = deg2rad(57);
longitude0 = deg2rad(11);

% latitude_rad = 1.0068389386;
% longitude_rad = 0.2091060593;


X_GPS_g = RADIUS_OF_THE_EARTH * (longitude_rad - longitude0) .* cos(latitude0)
Y_GPS_g = RADIUS_OF_THE_EARTH * (latitude_rad - latitude0)


% writematrix(X_GPS_g, 'X_GPS_gg.csv');

% writematrix(Y_GPS_g, 'Y_GPS_gg.csv');

% writematrix(X_est, 'X_est.csv');

% writematrix(Y_est, 'Y_est.csv');



fprintf('First point X_GPS_g = %.2f m\n', X_GPS_g(1));
fprintf('First point Y_GPS_g = %.2f m\n', Y_GPS_g(1));


figure;
plot(X_GPS_g, Y_GPS_g, 'b-', 'DisplayName', 'GPS Position Reference Trajectory');
hold on;
plot(X_est, Y_est, 'r--', 'DisplayName', 'Estimated Position');
hold on;
xlabel('X (m)');
ylabel('Y (m)');
title('Estimated Position');
legend('Location', 'best');
grid on;


figure;
plot(X_GPS_g, Y_GPS_g, 'b-', 'DisplayName', 'GPS Position');
hold on;
title('Reference Position');
legend('Location', 'best');
grid on;

figure;
plot(X_est, Y_est, 'r--', 'DisplayName', 'Estimated Position');
hold on;
xlabel('X (m)');
ylabel('Y (m)');
title('Estimated Position');
legend('Location', 'best');
grid on;


figure;
closest_point_index = data(:, 2);
plot(closest_point_index, 'DisplayName', 'closest_point_index');


figure;
Reset_Traj = data(:, 16);
plot(Reset_Traj);


figure;
data = readmatrix('eight_data_2023_05.csv');  
Reset_Traj = data(:, 12);

data = readmatrix('eight_data_greenbike_2024_05.csv');  
Reset_Traj = data(:, 11);


plot(Reset_Traj);

