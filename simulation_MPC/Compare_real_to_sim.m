clear all;
close all;
clc;

% Logged Variables from myRIO
% =================================================================================================
% PWM  LatGPS_deg_  LongGPS_deg_  AccelerometerY_rad_s_2_ GyroscopeX_rad_s_  GyroscopeZ_rad_s_
% SteeringAngleEncoder_rad_  SpeedVESC_rad_s_  Time_ms_  GNSSFlag  Input
% ResetTraj  StateEstimateX_m_  StateEstimateY_m_  StateEstimatePsi_rad_  StateEstimateRoll_rad_
% StateEstimateRollrate_rad_s_  StateEstimateDelta_rad_
% StateEstimateVelocity_m_s_  SteerrateInput_rad_s_  Rollref Closestpoint
% Error1  Error2  DpsirefContribution  LateralContribution
% HeadingContribution  StateEstimatorIterations  TrajectoryIterations
% steeringFlag  GPSVelocity_m_s_  E2Limit  E1Limit  GyroscopeY_rad_s_
% SpeedReference_rad_s_  PICurrent_A_  KiSpeed  KpSpeed  InputCurrent_A_
% MotorCurrent_A_  KdBalancing  Vref_m_s_
% =================================================================================================

%% Load data

% sim_data = readtable("bikedata_sim_est.csv");
% data_lab = readtable('Logging_data\Test_session_14_06\data_8.csv');
% Table_traj = readtable('Traj_ref_test\trajectorymat_parking_line.csv');

% sim_data = readtable("reference_output_speedupanddown.csv");
% data_lab_outsidetest = readtable('data_outside2_1.csv');
data_lab_outsidetest = readtable('data_outside2_1.csv');

data_lab = readtable('data_tt.csv');
% data_sensor_corridor_new


% data_another_test2.csv
fig = figure();
subplot(2,1,1)
plot(0.001 * (data_lab.Time_ms_-201298), 180/pi * data_lab.GyroscopeX_rad_s_, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('measured roll rate vs Time')
grid on

subplot(2,1,2)
plot(0.001 * (data_lab.Time_ms_-201298), 180/pi * data_lab.StateEstimateRoll_rad_, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('roll angle from Kalman filter vs Time')
grid on


fig = figure();
plot(0.001 * (data_lab.Time_ms_-201298), 180/pi * data_lab.SteeringAngleEncoder_rad_, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('measured steering angle vs Time')
grid on


% Take into account a valid speed. 
    v=3; 
% set the initial global coordinate system for gps coordinates
    gps_delay = 5;
% Choose The Bike - Options: 'red','black','green','scooter','plastic' 
    bike = 'red';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
    
% selector =1 to cut the data from the moment that steering is on until the end,
% selector=0 to cut the data from a specific point
% selector=2 all the data will be plotted
selector = 2;

% Starting point of simulation in the test
start_time = 0;

%% Prepare data for ploting

% Delete the data before reseting the trajectory and obtain X/Y position
% reset_traj = find(data_lab.ResetTraj==1,1,'last');
% data_lab(1:reset_traj,:) = [];
longitude0 = deg2rad(11);
latitude0 = deg2rad(57);
Earth_rad = 6371000.0;

X = Earth_rad * (data_lab.LongGPS_deg_ - longitude0) * cos(latitude0);
Y = Earth_rad * (data_lab.LatGPS_deg_ - latitude0);

X_outsidetest = Earth_rad * (data_lab_outsidetest.LongGPS_deg_ - longitude0) * cos(latitude0);
Y_outsidetest = Earth_rad * (data_lab_outsidetest.LatGPS_deg_ - latitude0);

% Obtain the relative time of the data
data_lab.Time = (data_lab.Time_ms_ - data_lab.Time_ms_(1))*0.001;
data_lab_outsidetest.Time = (data_lab_outsidetest.Time_ms_ - data_lab_outsidetest.Time_ms_(1))*0.001;



% Obtain the measurements
ay = -data_lab.AccelerometerY_rad_s_2_;
omega_x = data_lab.GyroscopeX_rad_s_;
omega_z = data_lab.GyroscopeZ_rad_s_;
delta_enc = data_lab.SteeringAngleEncoder_rad_;
v_enc = data_lab.SpeedVESC_rad_s_*bike_params.r_wheel;
v_GPS=data_lab.GPSVelocity_m_s_;

% Obtain the measurements
ay_outsidetest = -data_lab_outsidetest.AccelerometerY_rad_s_2_;
omega_x_outsidetest = data_lab_outsidetest.GyroscopeX_rad_s_;
omega_z_outsidetest = data_lab_outsidetest.GyroscopeZ_rad_s_;
delta_enc_outsidetest = data_lab_outsidetest.SteeringAngleEncoder_rad_;
v_enc_outsidetest = data_lab_outsidetest.SpeedVESC_rad_s_*bike_params.r_wheel;
v_GPS_outsidetest =data_lab_outsidetest.GPSVelocity_m_s_;

% Prepare measurement data for the offline kalman
gps_init = find(data_lab.GNSSFlag > gps_delay, 1 );
% data_lab.GNSSFlag   flag   GNSSflag
measurementsGPS = [data_lab.Time X Y];
measurementsGPS(1:gps_init,:) = [];
X(1:gps_init) = [];
Y(1:gps_init) = [];
measurements = [data_lab.Time ay omega_x omega_z delta_enc v_enc];
measurements(1,:) = [];
steer_rate = [data_lab.Time data_lab.SteerrateInput_rad_s_];
steer_rate(1,:) = [];
gpsflag = [data_lab.Time data_lab.GNSSFlag];
% data_lab.GNSSFlag   flag

% Prepare measurement data for the offline kalman
gps_init_outsidetest = find(data_lab_outsidetest.GNSSFlag > gps_delay, 1 );
% data_lab.GNSSFlag   flag   GNSSflag
measurementsGPS_outsidetest = [data_lab_outsidetest.Time X_outsidetest Y_outsidetest];
measurementsGPS_outsidetest(1:gps_init,:) = [];
X_outsidetest(1:gps_init) = [];
Y_outsidetest(1:gps_init) = [];
measurements_outsidetest = [data_lab_outsidetest.Time ay_outsidetest omega_x_outsidetest omega_z_outsidetest delta_enc_outsidetest v_enc_outsidetest];
measurements_outsidetest(1,:) = [];
steer_rate_outsidetest = [data_lab_outsidetest.Time data_lab_outsidetest.SteerrateInput_rad_s_];
steer_rate_outsidetest(1,:) = [];
gpsflag_outsidetest = [data_lab_outsidetest.Time data_lab_outsidetest.GNSSFlag];

% % Translate the trajectory to the point where is reseted
% GPS_offset_X = X(1) - Table_traj.Var1(1);
% GPS_offset_Y = Y(1) - Table_traj.Var2(1);
% Table_traj.Var1(:) = Table_traj.Var1(:) + GPS_offset_X;
% Table_traj.Var2(:) = Table_traj.Var2(:) + GPS_offset_Y;

% % Make the time from the simulation and test match
% sim_data.Time(:,1) = sim_data.Time(:,1) + start_time;

%% Plot 

if selector == 0
        start_point = 2400;
        end_point = length(measurementsGPS)-1;
       elseif selector == 1
             start_point=find(data_lab.SpeedVESC_rad_s_~=0,1,'first');
             end_point = length(measurementsGPS)-1;
        elseif selector == 2
            start_point = 1;
            end_point = length(measurementsGPS)-1;
        elseif selector == 3
            start_point = find(data_lab.steeringFlag~=0,1,'first');
            end_point = length(measurementsGPS)-1;
            
end





fig = figure();
subplot(3,1,1)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
% ylim([-30 10])
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value about the X-axis of Bike frame vs Time')
grid on

subplot(3,1,2)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeY_rad_s_(start_point:end_point), 'r')
hold on
% ylim([-2 6])
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value about the Y-axis of Bike frame vs Time')
grid on
subplot(3,1,3)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeZ_rad_s_(start_point:end_point), 'r')
hold on
% ylim([-10 25])
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('Measured angular rate value about the Z-axis of Bike frame vs Time')
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), 4 * 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
plot(data_lab.Time(start_point:end_point), -180/pi * gradient(data_lab.SteeringAngleEncoder_rad_(start_point:end_point))*20, 'b')
hold on
xlabel('Time (s)')  
ylabel('angle (degree)')
legend('command steerng angle','actual steering angle')
grid on
title('steering angle vs Time')
grid on



fig = figure();
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-276354), data_lab.AccelerometerX_rad_s_2_, 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('Measured acceleration value along the X-axis of IMU frame vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-276354), data_lab.AccelerometerY_rad_s_2_, 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('Measured acceleration value along the Y-axis of IMU frame vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-276354), data_lab.AccelerometerZ_rad_s_2_, 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('Measured acceleration value along the Z-axis of IMU frame vs Time')
grid on


roll_rate_b  = data_lab.GyroscopeX_rad_s_;
pitch_rate_b = data_lab.GyroscopeY_rad_s_;
yaw_rate_b   = data_lab.GyroscopeZ_rad_s_;

dt = 0.05;                            
inte_roll = cumtrapz(roll_rate_b) * dt;
inte_pitch = cumtrapz(pitch_rate_b) * dt;
inte_yaw = cumtrapz(yaw_rate_b) * dt;
fig = figure();
subplot(3,1,1)
plot(0.001 * (data_lab.Time_ms_-168134), 180/pi * inte_roll, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('Roll angle from integration of measured roll rate vs Time')
grid on
subplot(3,1,2)
plot(0.001 * (data_lab.Time_ms_-168134), 180/pi * inte_pitch, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('Pitch angle from integration of measured pitch rate vs Time')
grid on
subplot(3,1,3)
plot(0.001 * (data_lab.Time_ms_-168134), 180/pi * inte_yaw, 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
ylim([0 300]); 
title('Yaw angle from integration of measured yaw rate vs Time')
grid on



fig = figure();
subplot(3,1,1)
plot(data_lab.Time(start_point:end_point), data_lab.AccelerometerX_rad_s_2_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('measured accelerater vs Time')
grid on
subplot(3,1,2)
plot(data_lab.Time(start_point:end_point), data_lab.AccelerometerY_rad_s_2_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('measured accelerater vs Time')
grid on
subplot(3,1,3)
plot(data_lab.Time(start_point:end_point), data_lab.AccelerometerZ_rad_s_2_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('accelerater')
title('measured accelerater vs Time')
grid on



dt = 0.05;
sig1 = 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point);                              
mea = cumtrapz(sig1) * dt;
t = data_lab.Time(start_point:end_point);
fig = figure();
plot(t, mea, 'r', 'DisplayName','integrated steering angle'); 
hold on
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRoll_rad_(start_point:end_point), 'b')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('roll angle vs Time')
hold on
legend('roll angle from integration of measured roll rate','roll angle from Kalman filter')
grid on











fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
plot(data_lab.Time(start_point:end_point), -1 * 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'b')
hold on
plot(data_lab.Time(start_point:end_point), 0.5 * 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'g')
hold on
plot(data_lab.Time(start_point:end_point), -0.5 * 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'k')
hold on
plot(data_lab.Time(start_point:end_point), 2 * 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'c')
hold on
plot(data_lab.Time(start_point:end_point), -2 * 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'm')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
legend('Measured signal','Inverted measured signal', 'Half of the measured signal', 'Half of the inverted measured signal',...
       'Two times the measured signal', 'Two times the inverted measured signal')
title('measured roll rate vs Time')
grid on



fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'r')
hold on
plot(data_lab.Time(start_point:end_point), -1 * 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'b')
hold on
plot(data_lab.Time(start_point:end_point), 0.5 * 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'g')
hold on
plot(data_lab.Time(start_point:end_point), -0.5 * 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'k')
hold on
plot(data_lab.Time(start_point:end_point), 2 * 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'c')
hold on
plot(data_lab.Time(start_point:end_point), -2 * 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'm')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
legend('Measured signal','Inverted measured signal', 'Half of the measured signal', 'Half of the inverted measured signal',...
       'Two times the measured signal', 'Two times the inverted measured signal')
title('measured steering angle vs Time')
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('measured roll rate vs Time')
grid on


fig = figure();
subplot(2,1,1)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('measured roll rate vs Time')
grid on
subplot(2,1,2)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('measured steering angle vs Time')
grid on

fig = figure();
subplot(2,2,1)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('measured roll rate vs Time')
grid on

subplot(2,2,2)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('measured steering angle vs Time')
grid on

subplot(2,2,3)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRoll_rad_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
title('estimated roll angle from KF vs Time')
grid on

subplot(2,2,4)
plot(data_lab.Time(start_point:end_point), data_lab.SpeedVESC_rad_s_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('measured speed from vesc (rad/s)')
grid on


dt = 0.05;
start_point_point = find(data_lab.steeringFlag ~= 0, 1, 'first');
rollRate_all = data_lab.StateEstimateRollrate_rad_s_;
if ~isempty(start_point_point) && start_point_point > 1
    rollRate_all(1:start_point_point-1) = 0;
end
sig = -4 * 180/pi * rollRate_all(start_point:end_point);                              
delta = cumtrapz(sig) * dt;
t = data_lab.Time(start_point:end_point);
fig = figure();
plot(t, delta, 'r', 'DisplayName','integrated steering angle'); 
hold on
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'b')
hold on
ylabel('degree')
xlabel('Time (s)')
legend('steering angle from integration of reference steering rate','measured steering angle')
grid on





fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
hold on
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRollrate_rad_s_(start_point:end_point), 'b')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
legend('roll rate from gyro','roll rate from Kalman filter')
title('roll rate vs Time')
grid on



fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRoll_rad_(start_point:end_point))
xlabel('Time (s)')
ylabel('angle (degree)')
title('estimated roll angle vs Time')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.steeringFlag(start_point:end_point))
xlabel('Time (s)')
ylabel('steering flag')
hold on
grid on


fig = figure();
subplot(3,1,1)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'b')
hold on
plot(t, delta, 'r'); 
hold on
ylabel('angle (degree)')
xlabel('Time (s)')
legend('measured steering angle','steering angle from integration of reference steering rate')
title('steering angle vs Time')
grid on


subplot(3,1,2)
plot(t, mea, 'r'); 
hold on
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRoll_rad_(start_point:end_point), 'b')
hold on
xlabel('Time (s)')
ylabel('angle (degree)')
legend('roll angle from integration of measured roll rate','roll angle from Kalman filter')
grid on
title('roll angle vs Time')

subplot(3,1,3)
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'b')
hold on
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRollrate_rad_s_(start_point:end_point), 'r')
hold on
xlabel('Time (s)')
ylabel('angle rate (degree/s)')
title('roll rate vs Time')
legend('roll rate from gyro','roll rate from Kalman filter')
grid on


fig = figure();
plot(t, sig, 'b', 'DisplayName','reference steering angle rate'); 
grid on
ylabel('degree/s')
xlabel('Time (s)')
title('steering angle rate & integrated angle vs Time')
legend('Location','best')

fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.steeringFlag(start_point:end_point))
xlabel('Time (s)')
ylabel('steering flag')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), -6 * 180/pi * data_lab.StateEstimateRollrate_rad_s_(start_point:end_point), 'b')
hold on 
% plot(data_lab.Time(start_point:end_point), gradient(data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 0.05), 'r')
% ylabel('angle rate (degree/s)')
legend('reference steering angle rate')
xlabel('Time (s)')
grid on
title('steering angle rate vs Time')

fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateDelta_rad_(start_point:end_point), 'b')
hold on 
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'r')
hold on
ylabel('steering angle (degree)')
legend('estimated steering angle','measured steering angle')
xlabel('Time (s)')
grid on
title('estimated steering angle and measured steering angle vs Time')




fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteeringAngleEncoder_rad_(start_point:end_point), 'r')
ylabel('angle (degree)')
legend('measured steering angle')
hold on
xlabel('Time (s)')
grid on
title('steering angle vs Time')



fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.InputCurrent_A_(start_point:end_point), 'b')
ylabel('Current (A)')
hold on
plot(data_lab.Time(start_point:end_point), data_lab.MotorCurrent_A_(start_point:end_point), 'r')
ylabel('Current (A)')
hold on
xlabel('Time (s)')
legend('Input Current to VESC','Current from VESC to Drive Motor')
grid on
title('Input Current and Motor Current vs Time')








fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeZ_rad_s_(start_point:end_point), 'r')
ylabel('angle rate (degree/s)')
legend('measured angle rate')
hold on
xlabel('Time (s)')
grid on
title('angle rate vs Time')



fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRollrate_rad_s_(start_point:end_point), 'b')
hold on 
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.GyroscopeX_rad_s_(start_point:end_point), 'r')
ylabel('angle rate (degree/s)')
legend('estimated roll rate','measured roll rate')
hold on
xlabel('Time (s)')
grid on
title('estimated roll rate and measured roll rate vs Time')



fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.PWM(start_point:end_point))
xlabel('Time (s)')
ylabel('PWM')
hold on
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.Rollref(start_point:end_point), 'b')
xlabel('Time (s)')
ylabel('roll ref (degree)')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateVelocity_m_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('Estimate Velocity (m/s)')
hold on
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.Vref_m_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('Vref (m/s)')
hold on
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRoll_rad_(start_point:end_point))
xlabel('Time (s)')
ylabel('estimated roll angle (degree)')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateDelta_rad_(start_point:end_point))
xlabel('Time (s)')
ylabel('estimated steering angle (degree)')
hold on
grid on




fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.Error1(start_point:end_point))
xlabel('Time (s)')
ylabel('E1')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.Error2(start_point:end_point))
xlabel('Time (s)')
ylabel('E2')
hold on
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.E1Limit(start_point:end_point))
xlabel('Time (s)')
ylabel('E1Limit')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.E2Limit(start_point:end_point))
xlabel('Time (s)')
ylabel('E2Limit')
hold on
grid on

fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimatePsi_rad_(start_point:end_point))
xlabel('Time (s)')
ylabel('estimated psi angle (degree)')
hold on
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.SteerrateInput_rad_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('Steerrate (degree/s)')
hold on
grid on



fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateVelocity_m_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('Estimate Velocity (m/s)')
hold on
grid on



fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.GPSVelocity_m_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('GPS Velocity (m/s)')
hold on
grid on


fig = figure();
plot(data_lab.Time(start_point:end_point), 180/pi * data_lab.StateEstimateRollrate_rad_s_(start_point:end_point))
xlabel('Time (s)')
ylabel('estimated Rollrate (degree/s)')
hold on
grid on





fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.pose_error_m_(start_point:end_point), 'b')
xlabel('Time (s)')
ylabel('pose error (m)')
hold on
grid on

fig = figure();
yyaxis left
plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateVelocity_m_s_(start_point:end_point), 'b')
ylabel('Velocity (m/s)')
hold on
% yline(2, '--k', 'LabelHorizontalAlignment', 'left'); 
ylabel('velocity (m/s)')
legend('Velocity from Estimator')
hold on

yyaxis right
plot(data_lab.Time(start_point:end_point), data_lab.PICurrent_A_(start_point:end_point), 'r')
ylabel('Current (A)')
legend('Velocity from Estimator', 'PI Current')
hold on

xlabel('Time (s)')
grid on
title('Velocity and PI Current vs Time')



% pose error-Vref
fig = figure();
plot((data_lab.Time_ms_(start_point:end_point) - data_lab.Time_ms_(1))/1000, data_lab.SpeedVESC_rad_s_(start_point:end_point))
hold on
xlabel('Time (s)')
ylabel('speed vesc (rad/s)')
grid on

% start_point=find(data_lab.Lapsed~=0,1,'first');
% end_point = length(measurementsGPS)-1;

% pose error-Vref
fig = figure();
subplot(121)
plot(data_lab.Time(start_point:end_point), data_lab.pose_errorM(start_point:end_point))
hold on
xlabel('Time (s)')
ylabel('pose error (m)')
grid on
subplot(122)
plot(data_lab.Time(start_point:end_point), data_lab.Vref_t_m_s_(start_point:end_point))
hold on
xlabel('Time (s)')
ylabel('reference speed (m/s)')
grid on


% pose error-Vref
fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.pose_errorM(start_point:end_point))
hold on
plot(data_lab.Time(start_point:end_point), data_lab.Vref_t_m_s_(start_point:end_point))
ylabel('error (m) and reference speed (m/s)')
hold on
legend('pose error','reference speed');
grid on



% closest point
figure;
plot(data_lab.Time(start_point:end_point), data_lab.Closestpoint(start_point:end_point));
hold on;
% plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), data_lab_outsidetest.Closestpoint(start_point_outsidetest:end_point_outsidetest));
% hold on;
ylabel('Closestpoint');
title('Closestpoint');
legend('Closestpoint index from outside test');
grid on;

% States
fig = figure();
subplot(421)
% plot(sim_data.Time(:,1),sim_data.X_estimated(:,1))
% plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,1)));
% hold on
plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateX_m_(start_point:end_point)-measurementsGPS(1,2))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest)-measurementsGPS_outsidetest(1,2))
hold on
plot(measurementsGPS_outsidetest(start_point_outsidetest:end_point_outsidetest,1),measurementsGPS_outsidetest(start_point_outsidetest:end_point_outsidetest,2)-measurementsGPS_outsidetest(1,2))
hold on

xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test', 'Measurement')

subplot(423)
% plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,2))
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), data_lab.StateEstimateY_m_(start_point:end_point)-measurementsGPS(1,3))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), data_lab_outsidetest.StateEstimateY_m_(start_point_outsidetest:end_point_outsidetest)-measurementsGPS_outsidetest(1,3))
hold on
plot(measurementsGPS_outsidetest(start_point_outsidetest:end_point_outsidetest,1),measurementsGPS_outsidetest(start_point_outsidetest:end_point_outsidetest,3)-measurementsGPS_outsidetest(1,3))
hold on
% plot(measurementsGPS(start_point:end_point,1),measurementsGPS(start_point:end_point,3)-measurementsGPS(1,3))

xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test', 'Measurement')

subplot(425)
% plot(sim_data.Time(:,1), rad2deg(wrapToPi(sim_data.Psi_estimated(:,1))))
% hold on
% plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,3)));
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), rad2deg(wrapToPi(data_lab.StateEstimatePsi_rad_(start_point:end_point))))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), rad2deg(wrapToPi(data_lab_outsidetest.StateEstimatePsi_rad_(start_point_outsidetest:end_point_outsidetest))))
hold on

xlabel('Time (s)')
ylabel('heading (deg)')
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test')

subplot(422)
% plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,4)));
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)),rad2deg(data_lab.StateEstimateRoll_rad_(start_point:end_point)))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest),rad2deg(data_lab_outsidetest.StateEstimateRoll_rad_(start_point_outsidetest:end_point_outsidetest)))
hold on
% plot(sim_data.Time(:,1),rad2deg(sim_data.ref_roll(:,1)))

xlabel('Time (s)')
ylabel('Roll (deg)')
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test')

subplot(424)
% plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,5)));
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), rad2deg(data_lab.StateEstimateRollrate_rad_s_(start_point:end_point)))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest),rad2deg(data_lab_outsidetest.StateEstimateRollrate_rad_s_(start_point_outsidetest:end_point_outsidetest)))
hold on

xlabel('Time (s)')
ylabel('Roll Rate (deg/s)')
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test')

subplot(426)
% plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,6)));
% hold on
% plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,6)));
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)),rad2deg(data_lab.StateEstimateDelta_rad_(start_point:end_point)))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest),rad2deg(data_lab_outsidetest.StateEstimateDelta_rad_(start_point_outsidetest:end_point_outsidetest)))
hold on

xlabel('Time (s)')
ylabel('Steering Angle (deg)')
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test')

subplot(427)
% plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,7));
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), data_lab.StateEstimateVelocity_m_s_(start_point:end_point))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), data_lab_outsidetest.StateEstimateVelocity_m_s_(start_point_outsidetest:end_point_outsidetest))
hold on
% plot(data_lab.Time(start_point:end_point), v_enc(start_point:end_point))
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), v_GPS(start_point:end_point))
hold on

xlabel('Time (s)')
ylabel('velocity (m/s)')
% ylim([-1 5])
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test', 'Measurement from GPS')


fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateVelocity_m_s_(start_point:end_point))
hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), data_lab_outsidetest.StateEstimateVelocity_m_s_(start_point_outsidetest:end_point_outsidetest))
hold on
% plot(data_lab.Time(start_point:end_point), v_enc(start_point:end_point))
% hold on
plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), v_GPS(start_point:end_point))
hold on
xlabel('Time (s)')
ylabel('velocity (m/s)')
% ylim([-1 5])
grid on
legend('Estimation from Offline Test', 'Estimation from Outside Test', 'Measurement from GPS')



% Extract time and X position segment
time_segment = data_lab.Time(start_point:end_point);
x_segment = data_lab.StateEstimateX_m_(start_point:end_point) - measurementsGPS(1,2);

% Find GPS update points (where GNSSFlag changes)
GNSSFlag_segment = data_lab.GNSSFlag(start_point:end_point);  
gps_update_idx = [1; find(diff(GNSSFlag_segment) ~= 0) + 1];

% Plot X position over time
figure;
plot(time_segment, x_segment, 'b-', 'LineWidth', 1.5); % main curve
hold on;

% Mark GPS update points with red circles
plot(time_segment(gps_update_idx), x_segment(gps_update_idx), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% Labels and legend
xlabel('Time (s)');
ylabel('X - GPS initial (m)');
title('X Position Over Time with GPS Updates (Offline Test)');
legend('X position', 'GPS update');
grid on;



% Extract time and X position segment for outsidetest
time_segment_out = data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest);
x_segment_out = data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest) - measurementsGPS_outsidetest(1,2);

% Find GPS update points (where GNSSFlag changes)
GNSSFlag_segment_out = data_lab_outsidetest.GNSSFlag(start_point_outsidetest:end_point_outsidetest);  
gps_update_idx_out = [1; find(diff(GNSSFlag_segment_out) ~= 0) + 1];

% Plot X position over time
figure;
plot(time_segment_out, x_segment_out, 'b-', 'LineWidth', 1.5); % main curve
hold on;

% Mark GPS update points with red circles
plot(time_segment_out(gps_update_idx_out), x_segment_out(gps_update_idx_out), ...
    'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% Labels and legend
xlabel('Time (s)');
ylabel('X - GPS initial (m)');
title('X Position Over Time with GPS Updates (Outside Test)');
legend('X position', 'GPS update');
grid on;



% === Real test segment ===
time_real = data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest);
x_real = data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest) - measurementsGPS_outsidetest(1,2);
gps_idx_real = [1; find(diff(data_lab_outsidetest.GNSSFlag(start_point_outsidetest:end_point_outsidetest)) ~= 0) + 1];

% === Simulated test segment (outsidetest) ===
time_sim_original = data_lab.Time(start_point:end_point);
x_sim = data_lab.StateEstimateX_m_(start_point:end_point) - measurementsGPS(1,2);
gps_idx_sim = [1; find(diff(data_lab.GNSSFlag(start_point:end_point)) ~= 0) + 1];

% === Map simulated time to real test time range ===
% Get start/end time of both segments
t_real_start = time_real(1);                % e.g., 10
t_real_end   = time_real(end);              % e.g., 45
t_sim_start  = time_sim_original(1);        % e.g., 80
t_sim_end    = time_sim_original(end);      % e.g., 337

% Scale and shift simulation time to match real test range
scale = (t_real_end - t_real_start) / (t_sim_end - t_sim_start);
time_sim_mapped = (time_sim_original - t_sim_start) * scale + t_real_start;






% === Plot both on same figure ===
figure;
plot(time_real, x_real, 'b-', 'LineWidth', 1.5); hold on;
plot(time_real(gps_idx_real), x_real(gps_idx_real), 'bo', 'MarkerSize', 6, 'LineWidth', 1.5);

plot(time_sim_mapped, x_sim, 'r-', 'LineWidth', 1.5);
plot(time_sim_mapped(gps_idx_sim), x_sim(gps_idx_sim), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);

% === Labels ===
xlabel('Time (s)');
ylabel('X - GPS initial (m)');
title('X Position Over Time: Outside Test vs. Offline Test');
legend('X from outside test', 'GPS update', 'X from offline test', 'GPS update');
grid on;





% Trajectory
fig = figure();

% Offline Test
plot3(data_lab.StateEstimateX_m_(start_point:end_point) - X(1), ...
      data_lab.StateEstimateY_m_(start_point:end_point) - Y(1), ...
      data_lab.Time(start_point:end_point));
hold on

% Outside Test
plot3(data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest) - X_outsidetest(1), ...
      data_lab_outsidetest.StateEstimateY_m_(start_point_outsidetest:end_point_outsidetest) - Y_outsidetest(1), ...
      data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest));
hold on



GNSSFlag_segment = data_lab.GNSSFlag(start_point:end_point);  
gps_update_idx = [1; find(diff(GNSSFlag_segment) ~= 0) + 1];
x_plot = data_lab.StateEstimateX_m_(start_point:end_point) - X(1);
y_plot = data_lab.StateEstimateY_m_(start_point:end_point) - Y(1);
plot(x_plot(gps_update_idx), y_plot(gps_update_idx), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
hold on

GNSSFlag_segment_outsidetest = data_lab_outsidetest.GNSSFlag(start_point_outsidetest:end_point_outsidetest);  
gps_update_idx_outsidetest = [1; find(diff(GNSSFlag_segment_outsidetest) ~= 0) + 1];
x_plot_outsidetest = data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest) - X_outsidetest(1);
y_plot_outsidetest = data_lab_outsidetest.StateEstimateY_m_(start_point_outsidetest:end_point_outsidetest) - Y_outsidetest(1);
plot(x_plot_outsidetest(gps_update_idx_outsidetest), y_plot_outsidetest(gps_update_idx_outsidetest), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
hold on

view(0, 90)
xlabel('X position (m)')
ylabel('Y position (m)')
axis equal
legend('Estimated from Offline Test', 'Estimated from Outside Test', 'GPS Update Points');
title('Trajectory with GPS Updates')




% Trajectory
fig = figure();
plot3(data_lab.StateEstimateX_m_(start_point:end_point) - X(1), data_lab.StateEstimateY_m_(start_point:end_point) - Y(1), data_lab.Time(start_point:end_point))
hold on
plot3(data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest) - X_outsidetest(1), data_lab_outsidetest.StateEstimateY_m_(start_point_outsidetest:end_point_outsidetest) - Y_outsidetest(1), data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest))
hold on
gps_update_idx = [1; find(diff(GNSSFlag) ~= 0) + 1];
plot(x_est(gps_update_idx), y_est(gps_update_idx), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
view(0,90)
xlabel('X position (m)')
ylabel('Y position (m)')
axis equal
legend('Estimated from Offline Test', 'Estimated from Outside Test');
title('Trajectory')


% Trajectory
fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateX_m_(start_point:end_point))
hold on
% plot(data_lab.Time(start_point:end_point)/(data_lab.Time(start_point)/data_lab_outsidetest.Time(start_point_outsidetest)), data_lab.StateEstimateX_m_(start_point:end_point))
% hold on
plot(data_lab_outsidetest.Time(start_point_outsidetest:end_point_outsidetest), data_lab_outsidetest.StateEstimateX_m_(start_point_outsidetest:end_point_outsidetest))
hold on
xlabel('time (s)')
ylabel('X position (m)')
legend('Estimated X from Offline Test', 'Estimated X from Outside Test');
title('Estimated X')

% Trajectory
fig = figure();
plot3(measurementsGPS(start_point:end_point,2) - X(1),measurementsGPS(start_point:end_point,3) - Y(1),data_lab.Time(start_point:end_point),'*')
hold on
view(0,90)
xlabel('X position (m)')
ylabel('Y position (m)')
axis equal
legend('Measurements');
title('Trajectory')






% ResetTraj and Closestpoint
fig = figure();
plot(data_lab.Time(start_point:end_point), data_lab.ResetTraj(start_point:end_point))
hold on
% plot(data_lab.Time(start_point:end_point), data_lab.Closestpoint(start_point:end_point))
% hold on
xlabel('Time (s)')
ylabel('Boolean Value')
title('ResetTraj Button');
grid on





% X
fig = figure();
subplot(211)
% plot(data_lab.Time(start_point:end_point), data_lab.StateEstimateX_m_(start_point:end_point)-measurementsGPS(1,2))
% hold on
% plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,1));
% hold on
plot(measurementsGPS(start_point:end_point,1),measurementsGPS(start_point:end_point,2)-measurementsGPS(1,2))
hold on
% plot(t_ref,Xref);
% xline(t_ref(end), '--r', 'End of Reference Trajectory', 'LabelOrientation', 'horizontal', ...
%     'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);
% hold on
xlabel('Time (s)')
ylabel('X position (m)')
xlim([0, measurementsGPS(end_point)])
grid on
legend('X from Simulink','GPS measurements', 'X ref')

% Y
subplot(212)
% plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,2));
% hold on
plot(measurementsGPS(start_point:end_point,1),measurementsGPS(start_point:end_point,3)-measurementsGPS(1,3))
hold on
% plot(t_ref,Yref);
% xline(t_ref(end), '--r', 'End of Reference Trajectory', 'LabelOrientation', 'horizontal', ...
%     'LabelVerticalAlignment', 'bottom', 'LineWidth', 1.5);
% hold on
xlabel('Time (s)')
ylabel('Y position (m)')
xlim([0, measurementsGPS(end_point)])
ylim([-1 20])
grid on
legend('Y from Simulink','GPS measurements', 'Y ref')







% Delta contributions
test = rad2deg(-0.008944 .* sign(data_lab.Error1(:,1)) .* min(abs(data_lab.Error1(:,1)),5.66548));
figure()
subplot(311)
hold on;
plot(sim_data.Time(:,1),rad2deg(sim_data.delta_e1(:,1)));
plot(data_lab.Time(start_point:end_point,1),rad2deg(data_lab.LateralContribution(start_point:end_point,1)));
% plot(data_lab.Time(start_point:end_point,1),test(start_point:end_point,1));
xlabel('Time [t]')
ylabel('Angle [Deg]')
legend('simulation','Onlime estimation','test','Location','southeast')
grid on
title('lateral error contribution')

subplot(312)
plot(sim_data.Time(:,1),rad2deg(sim_data.delta_e2(:,1)));
hold on
plot(data_lab.Time(start_point:end_point,1),rad2deg(data_lab.HeadingContribution(start_point:end_point,1)));
xlabel('Time [t]')
ylabel('Angle [Deg]')
legend('simulation','Onlime estimation','Location','southeast')
grid on
title('Heading error contribution')

subplot(313)
plot(sim_data.Time(:,1),rad2deg(sim_data.delta_psi(:,1)));
hold on
plot(data_lab.Time(start_point:end_point,1),rad2deg(data_lab.DpsirefContribution(start_point:end_point,1)));
xlabel('Time [t]')
ylabel('Angle [Deg]')
legend('simulation','Onlime estimation','Location','southeast')
grid on
title('Dpsiref contribution')

% Compare delta_ref and roll_ref
sum_cont = rad2deg(data_lab.DpsirefContribution)+rad2deg(data_lab.HeadingContribution)+rad2deg(data_lab.LateralContribution);
figure();
subplot(211)
hold on;
plot(sim_data.Time(:,1),rad2deg(sim_data.delta_ref(:,1)));
plot(data_lab.Time(start_point:end_point,1),sum_cont(start_point:end_point,1));
xlabel('Time [t]')
ylabel('Angle [Deg]')
legend('simulation','online estimation','Location','southeast')
grid on
title('Delta_{ref}')

subplot(212)
hold on;
plot(sim_data.Time(:,1),rad2deg(sim_data.ref_roll(:,1)));
plot(data_lab.Time(start_point:end_point,1),rad2deg(data_lab.Rollref(start_point:end_point,1)));
xlabel('Time [t]')
ylabel('Angle [Deg]')
legend('simulation','online estimation','Location','southeast')
grid on
title('Roll_{ref}')

% Lateral and heading error
figure();
subplot(211)
hold on;
plot(sim_data.Time(:,1),sim_data.error1(:,1));
plot(data_lab.Time(start_point:end_point,1),data_lab.Error1(start_point:end_point,1));
xlabel('Time [t]')
ylabel('Distance [m]')
legend('simulation','online estimation','Location','southeast')
grid on
title('Lateral error')

subplot(212)
hold on;
plot(sim_data.Time(:,1),rad2deg(sim_data.error2(:,1)));
plot(data_lab.Time(start_point:end_point,1),rad2deg(data_lab.Error2(start_point:end_point,1)));
xlabel('Time [t]')
ylabel('Angle [Deg]')
legend('simulation','online estimation','Location','southeast')
grid on
title('Heading error')




