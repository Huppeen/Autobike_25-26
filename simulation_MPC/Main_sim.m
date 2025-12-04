% TODO describe the purpose of this file,
% what is done if no test is performed, what is done of tests are
% performed?


%% clear the possible remnant on previous running
set(0,'defaulttextinterpreter','none');
clear all; 
clear;
close all;
clc;

%% Simulation Settings and Bike and General Parameters
% Gravitational Acceleration
    gg = 9.81;
% Name of the model
    model = 'Main_bikesim';
% Friction coefficient specific to the red bicycle; values may vary for different bicycles
    friction_coefficient = 0.004;
% Sampling Time
    Ts = 0.01; 
% First closest point selection in reference. Starts at 2 because the one 
% before closest is in the local reference as well
    ref_start_idx = 2; %end of page 64 of Lorenzo's thesis
% Horizon distance [m]
    % hor_dis = 10; %tra cosa?
    hor_dis = 5; %tra cosa?
%Constant Speed [m/s]
     vv = 2;
     v=vv; %for mpc 
% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options:'red', 'black', 'green', 'scooter' or 'plastic' 
    bike = 'green';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% bike model (for Simulink)
    bike_model = 1; % 1 = non-linear model || 2 = linear model
% 0 = Don't run test cases & save measurementdata in CSV || 1 = run test cases || 2 = generate ref yourself
    Run_tests = 0; 
% Take estimated states from a specific time if wanted (0 == initial conditions are set to zero || 1 == take from an online test)
    init = 0;
    time_start = 14.001; % what time do you want to take (if init==1)
% When you have bad GPS signal:
% Set indoor to 1 when you run the bike indoor or you have bad GPS signal
    indoor = 0; % used in Simulink
% Set badGPS=1 to make the GPS signal steady from the beginning of the sim.
    badGPS = 0; % used in Simulink
% Set compare_flag=1 if you want to compare two simulation results
    compare_flag = 0;
% Activate gain scheduling for system matrices and gains that depend on the
% velocity. Implemented on Kalman Filter and Heading dot contribution transfer function
    scheduling = 0;
% Activate the interpolation for the gain scheduling instead of taking
% matrices for nearest speed. Normally yes.
    interpolation = 0;  % used in Simulink (you can only use interpolation if scheduling = 1)
    if scheduling==0, interpolation = 0; end  % must be 0 if no scheduling

%% Reference trajectory generation
% Constant speed of the bicycle in meters per second
Vref_test = 2;            
% only for infinite and circle - radius used
laps = 1;
% Number of the whole reference points
lL = 100; 
% Distance between trajectory points in meters
ref_dis = 1 ;

[Xref,Yref,Psiref,t_ref] = differenttest('3',ref_dis,lL,laps,Vref_test);
fprintf('length(Xref)   = %d\n', length(Xref));
fprintf('length(Yref)   = %d\n', length(Yref));
fprintf('length(Psiref) = %d\n', length(Psiref));
fprintf('length(t_ref)  = %d\n', length(t_ref));
%used to set Vref constant
Vref= vv * ones(1, length(Xref));

% %% Plot trajectory before running (for DEBUG) ----------
% % Example: Label the trajectory every 50 data points
% step = 10; 
% figure;
% plot(Xref, Yref, 'ko', 'MarkerSize', 2); % Plot the path in black
% hold on;
% for i = 1:step:length(Xref)
% % Plot a red circle at the point
% plot(Xref(i), Yref(i), 'ro', 'MarkerFaceColor', 'r'); 
% % Add the time label near the point
% text_label = sprintf('t=%.1f s', t_ref(i));
% text(Xref(i) + 0.2, Yref(i) + 1, text_label, 'FontSize', 8); 
% end
% hold off;
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% title('Trajectory with Time Labels');
% axis equal;
% grid on;
% %% -----------------------------------------------------

%Do not use ? 
%[Psiref, Vref] = Refgeneration_test(Xref, Yref, t_ref);
v_init = Vref(1); % needed for lqr, referenceTest, simulink>atateestimator
Nn = length(Xref); % needed for simulink

%% Init states
offset_x = 0;
offset_y = 0;
offset_heading = 0;
% Initial X and Y positions of bike are the first trajectory point, and
% plus the offset
initial_state.x = Xref(1) + offset_x;
initial_state.y = Yref(1) + offset_y;
% Calculate the initial heading angle using the first two trajectory
% points, then convert it from degrees to radians, and plus the offset
initial_state.heading = deg2rad(atand((Yref(2)-Yref(1))/(Xref(2)-Xref(1)))) + deg2rad(offset_heading);
initial_state.roll = deg2rad(0);
initial_state.roll_rate = deg2rad(0);
initial_state.steering = deg2rad(0);
initial_pose=[initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;

%% Unpacked bike_params
[hh,lr,lf,lambda,cc,mm,h_imu,Tt] = UnpackBike_parameters(bike_params);

%% TransMatrix calculates the transformation matrix based on IMU orientation offsets.
% Inputs:
%   bike_params: Structure containing the following fields:
%       - IMU_x_mod: X position offset of the IMU.
%       - IMU_roll_mod: Roll angle offset in degrees.
%       - IMU_pitch_mod: Pitch angle offset in degrees.
%       - IMU_yaw_mod: Yaw angle offset in degrees.
% Outputs:
%   T: 3x3 transformation matrix that represents the rotation defined by the IMU orientation.
% The transformation matrix is computed using the roll, pitch, and yaw angles converted to radians.
% The matrix is used in Simulink State estimator->Linearized Bicycle Model on Constant Velocity
T = TransMatrix(bike_params);                                             

%% Balancing Controller
% Outer loop -- Roll Tracking
P_balancing_outer = 3.75;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3.5;
I_balancing_inner = 0;
D_balancing_inner = 0; 


%% Calculating gains and matrices which depend on velocity, based on velocity vector which is created below. 
if scheduling
    V_min= min(Vref(:));
    V_max= max(Vref(:));
    V_min=min([V_min,V_max-0.3]); % This is to make sure there is a non-zero interval for the scheduling
    v_max=max([V_max, V_min+0.6]);  % This can be improved. Interval is set ad-hoc
    if (V_max-V_min)<0.05 
        disp('Warning, no speed variation in Vref, scheduling matrices becomes identical'); 
        disp('Simulation does not work in this case.');
    end
else % No scheduling, constant matrices calculated for one fixed speed
    V_min= Vref(2);
    V_max= V_min;
end


% construct vector of velocities for which linear Kalman filter is
% obtained, ie, matrices for each velocity
V_stepSize=0.1; % design choice

V_n=ceil((V_max-V_min)/V_stepSize)+1; % number of velocities for which linearized matrices are calculated.
V=linspace(V_min,V_max,V_n);

V=round(V,1);

K_GPS=zeros(V_n,7,7);
K_noGPS=zeros(V_n,7,7);

counter=zeros(V_n,1);
A_d=zeros(V_n,7,7);
B_d=zeros(V_n,7,1);
C=zeros(V_n,7,7);
D=zeros(V_n,7,1);

A_t=zeros(V_n,1);
B_t=zeros(V_n,1);
C_t=zeros(V_n,1);
D_t=zeros(V_n,1);

% Q & R are calculated in a different file
load('Q_and_R_backup_red_bike.mat');

format long
for i=1: V_n
    % Kalman filtering for both cases - with/without GPS - 
    [K_GPS(i,:,:),K_noGPS(i,:,:),counter,A_d(i,:,:),B_d(i,:,:),C(i,:,:),D(i,:,:)] = KalmanFilter(V(i),hh,lr,lf,lambda,gg,cc,h_imu,Ts,Q,R);

    % Transfer function in heading in wrap traj
    num = 1;
    den = [lr/(lr+lf), V(i)/(lr+lf)];
    [A_t(i,:), B_t(i,:), C_t(i,:), D_t(i,:)] = tf2ss(num,den);
end
K_GPS=permute(K_GPS,[1,3,2]);
K_noGPS=permute(K_noGPS,[1,3,2]);
A_d=permute(A_d,[1,3,2]);
C=permute(C,[1,3,2]);


% Storing all the calculated matrices and gains.
GainsTable = table(V',K_GPS,K_noGPS,A_d,B_d,C,D, 'VariableNames', {'V','K_GPS','K_noGPS','A_d','B_d','C','D'});

 
% looks like the controller is not speed dependent, one fixed speed
%% The LQR controller
[k1,k2,e1_max,e2_max] = LQRcontroller(v_init,lr,lf);

%% MPC Trajectory controller
max_permissible_e1=100;
max_permissible_e2=100;
a = lr;
b = lr+lf;

N_outer=50; %prediction horizont
%penalty matrices 
Q_outer=[1 0; 0 1]*1e-2;  % penalty on state deviation
Pf_outer=[3 0; 0 100];  % penalty on final prediction step, i.e. "how important to reach"
R_outer=1;          % penalty on control signal 

% Q_outer=eye(2)*1e-2;  % penalty on state deviation
% Pf_outer=eye(2)*1e8;  % penalty on final prediction step, i.e. "how important to reach"
% R_outer=2;            % penalty on control signal 

% x=[e1 e2]'
A_outer=[0 v;0 0];
B_outer=[a*v/b;v/b];

C_outer=eye(2);
D_outer=zeros(1,2)';

sys_outer = ss(A_outer,B_outer,C_outer,D_outer);

% Discretization
sys_dis_outer = c2d(sys_outer,Ts);
Ad_outer = sys_dis_outer.A;
Bd_outer = sys_dis_outer.B;
Cd_outer = sys_dis_outer.C;
Dd_outer = sys_dis_outer.D;

%mpc 
stateOfConstraint_outer=[1 0;
                   -1 0;
                   0 1;
                   0 -1];
stateConstraintVal_outer=[max_permissible_e1;
                    max_permissible_e1;
                    max_permissible_e2;
                    max_permissible_e2];
inputConstraint_outer=[1;
                -1];

inputConstraintVal_outer=deg2rad(20);%max_permissible_str-deg2rad(10); % harder constraint on outer?

[rowInputConstraint_outer,colInputConstraint_outer]=size(inputConstraint_outer);
[rowStateOfConstraint_outer,colStateOfConstraint_outer]=size(stateOfConstraint_outer);

%Obs. constraints have the form Fx +Gu <=h, different from MPC course...
%Size of F:
%   - Cols: cols in x constr. times prediction steps (N) 
%   - Rows: rows in x constr. times N + rows in u constr.
%           times N
F_outer=[kron([eye(N_outer)],stateOfConstraint_outer);
    zeros(rowInputConstraint_outer*N_outer,colStateOfConstraint_outer*N_outer)];
%Size of G:
%   - Cols: cols in u constr. times N
%   - Rows: rows in x constr. times N + rows in u constr.
%           times N
G_outer=[zeros(N_outer*rowStateOfConstraint_outer,N_outer);kron(eye(N_outer),[1; -1])];

%Size of h_mpc:
%   -Cols: 1
%   -Rows: rows in x constr. times N + rows in u constr.
%           times N
h_mpc_outer=[kron([ones(1*N_outer,1)],stateConstraintVal_outer);
    ones(N_outer*rowInputConstraint_outer,1)*inputConstraintVal_outer];

mpc_outer_params=struct();
mpc_outer_params.Ad=Ad_outer;
mpc_outer_params.Bd=Bd_outer;
mpc_outer_params.Cd=Cd_outer;
mpc_outer_params.Dd=Dd_outer;
mpc_outer_params.N=N_outer;
mpc_outer_params.Q=Q_outer;
mpc_outer_params.Pf=Pf_outer;
mpc_outer_params.R=R_outer;
mpc_outer_params.F=F_outer;
mpc_outer_params.G=G_outer;
mpc_outer_params.h=h_mpc_outer;

%% Transfer function for heading in wrap traj
%feed forward transfer function for d_psiref to steering reference (steering contribution for heading changes)

% Discretize the ss 
% % Used in Simulink
Ad_t = eye(1)+Ts*A_t;% A_t and B_t are calculated on gains table section above.
Bd_t = B_t*Ts;


%% Combine matrix Ad_t, Bd_t, C_t, (D_t), and V'into a single matrix (linearizedMatrices).
%  The matrix is used in Simulink trajectory controller;
linearizedMatrices=[Ad_t, Bd_t, C_t, D_t, V'];

%% The following variables are needed in the Simulink model
% Define a cell array, including variable and field in a structure
required_vars = {'gg','Ts','badGPS','Xref','Yref','t_ref','k1','k2','e1_max','interpolation', ...
                 'linearizedMatrices',...
                 ...% given by function: LoadBikeParameters
                 'bike_params.lr','bike_params.lf','bike_params.lambda', ...
                 'bike_model','bike_params.Xgps_mod','bike_params.Ygps_mod','bike_params.Hgps_mod', ...
                 'bike_params.Xgps','bike_params.Ygps','bike_params.Hgps','bike_params.h', ...
                 'bike_params.c','bike_params.m','bike_params.IMU_height','bike_params.r_wheel', ...
                 'bike_params.drive_motor_gear_rat',...
                 ... % given in this file: Main_sim
                 'P_balancing_outer','P_balancing_inner'};

% Call the function to check required variables
% checkRequiredVars(required_vars);
% function checkRequiredVars(required_vars)
    % Check if the required variables are present in the MATLAB workspace.
    % Input:
    %   required_vars - A cell array of variable names (or structure fields) to check.

    for i = 1:length(required_vars)
        var_name = required_vars{i};    
        % Check if it's a top-level variable
        if exist(var_name, 'var')
            % If top-level variable exists but is empty
            if isempty(eval(var_name))
                warning('The variable "%s" has not been successfully assigned.', var_name);
            end
        else
            % Check if it's a field in a structure
            parts = strsplit(var_name, '.');
            if numel(parts) == 2 && exist(parts{1}, 'var') && isfield(eval(parts{1}), parts{2})
                % If it is a structure field but is empty
                if isempty(eval([parts{1} '.' parts{2}]))
                    warning('The field "%s" has not been successfully assigned.', var_name);
                end
            else
                % If neither a top-level variable nor a structure field
                warning('The variable or field "%s" does not exist or has not been successfully assigned.', var_name);
            end
        end
    end



%% Start the Simulation
if Run_tests == 0 || Run_tests == 2
% tic
% try 
%     Results = sim(model); % If no error occurs, MATLAB skips the catch.
%     catch error_details %note: the model has runned for one time here
% end
% toc
% ==============================
tic
    Results = sim(model); % If no error occurs, MATLAB skips the catch.
toc
% ==============================
% Simulation Messages and Warnings
% if Results.stop.Data(end) == 1
%     disp('Message: End of the trajectory has been reached');
% end

%% Plotting
% If you want to compare two different simulation results, then change the
% name of 'bikedata_sim_real_states.csv' and 'bikedata_sim_est.csv' to
% 'bikedata_sim_real_states_1.csv' and 'bikedata_sim_est_1.csv after
% running main_sim.m the first time and change compare_flag to 1 before
% running main_sim.m the second time.
%PlottingResults(test_curve,Results,compare_flag);


Tnumber = 'No test case: General simulation run';

Plot_bikesimulation_results(Tnumber, [Xref,Yref,Psiref], Results, compare_flag, t_ref, Vref, bike_params);
end


%% Test cases for validation
TestCases(Run_tests,hor_dis,Ts,initial_pose);
%%