 function [] = Plot_bikesimulation_results(Tnumber, test_curve, Results, compare_flag, t_ref, Vref, bike_params)

% Input Parameter:
% Tnumber is the name of the figure
% Ts is the sampling time, useless now
% test_curve is the reference path (Xg, Yg, Heading)
% Results contains all the "To Workspace" signals from Simulink

% If you want to add stored signals from previous simulation, read the 
% instruction to do that. Now the code only allows to do that for 
% Trajectory (line 23) and Roll angle (line 122).



    if compare_flag == 0
        %% Trajectory
        figure('Name', Tnumber);
        trajPlot = subplot(3,2,[1,3,5]);
        hold on;
        % Plot the reference path/trajectory
        plot3(test_curve(:,1), test_curve(:,2), 1:length(test_curve(:,1)), 'o');
        % Plot the real bike states (Data1=X, Data2=Y)
        plot3(Results.bike_states.Data(:,1), Results.bike_states.Data(:,2), Results.bike_states.Time(:,1));
        % Plot the estimated bike states from Kalman filter (Data1=X, Data2=Y)
        plot3(Results.estimated_states.Data(:,1), Results.estimated_states.Data(:,2), Results.estimated_states.Time(:,1),'--r');
        % xlim([0 80])
        % ylim([-1 60])  
        view(0,90)
        legend('Ref', 'True', 'Est', 'Location', 'northwest');
        xlabel('X-dir [m]');
        ylabel('Y-dir [m]');              
        axis equal
        grid on;
        title('Trajectory');
        

        % Lateral error plot
        subplot(3,2,2);
        hold on;
        p1 = plot(Results.e1.Time, Results.e1.Data);
        xlabel('Time [s]');
        ylabel('Distance [m]');
        title('Lateral error');
        grid on;

        % Heading error plot
        subplot(3,2,4);
        hold on;
        p2 = plot(Results.e2.Time, rad2deg(Results.e2.Data));
        xlabel('Time [s]');
        ylabel('Angle [deg]');
        title('Heading error');
        grid on;

        % Closest point index selection plot
        subplot(3,2,6);
        hold on;
        p3 = plot(Results.ids.Time, Results.ids.Data);
        p4 = plot(Results.closest_point.Time, Results.closest_point.Data - 1);
        p5 = plot(Results.closestpoint_heading.Time, Results.closestpoint_heading.Data - 2);
        legend('Selected idx Total', 'Selected idx in local', 'Selected idx for heading', 'Location', 'northwest');
        xlabel('Time [s]');
        ylabel('Index [-]');
        title('Closest point index selection');
        grid on;

        % Link the axes of the three subplots for synchronized zoom
        linkaxes([p1.Parent, p2.Parent, p3.Parent], 'x');

        % Enable Data Cursor mode and set the callback for point selection
        dcm = datacursormode(gcf);
        set(dcm, 'Enable', 'on', 'UpdateFcn', @(obj, event_obj) syncSelectedPoint(obj, event_obj, Results, trajPlot));
        
        % Enable Zoom mode and set the callback for adding arrows during zoom
        zoomHandle = zoom(gcf);
        set(zoomHandle, 'ActionPostCallback', @(src, event) zoomCallbackWithArrow(event, trajPlot, Results));
        
        hold off;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure('Name',Tnumber);        
        % X, Y, Psi
        subplot(3,2,1)
        hold on;
        p1 = plot(Results.ref_states.Time(:,1),Results.ref_states.Data(:,1));
        p2 = plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,1));
        p3 = plot(Results.estimated_states.Time(:,1),Results.estimated_states.Data(:,1));
        legend('Ref X','True X','Estimated X');
        xlabel('Time [s]');
        ylabel('Position X [m]');
        % ylim([-50 50])
        % xlim([-100 100])
        grid on;
        title('X-coordinate');
        
        subplot(3,2,3);
        hold on;
        p4 = plot(Results.ref_states.Time(:,1),Results.ref_states.Data(:,2));                
        p5 = plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,2));              
        p6 = plot(Results.estimated_states.Time(:,1),Results.estimated_states.Data(:,2));    
        legend('Ref Y','True Y','Estimated Y');
        xlabel('Time [s]');
        ylabel('Position Y [m]');
        % ylim([-50 50])
        % xlim([-100 100])
        grid on;
        title('Y-coordinate');
        
        subplot(3,2,5)
        hold on;
        p7 = plot(Results.ref_states.Time(:,1),rad2deg(Results.ref_states.Data(:,3)));
        p8 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,3)));
        p9 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,3)));
        p10 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,3)));
        
        legend('Ref psi','True psi','Estimated psi');
        xlabel('Time [t]');
        ylabel('Angle [Deg]');
        % ylim([-50 50])
        % xlim([-100 100])
        grid on;
        title('Heading');
        
        % Roll and Roll_rate
        subplot(3,2,2)
        hold on;
        p11 = plot(Results.roll_ref.Time(:,1),rad2deg(Results.roll_ref.Data(:,1)));
        p12 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,4)));
        p13 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,4)));
            
        % If you want to plot other data you should change the legend as you wish
        legend('Roll Ref','True Roll','Est. Roll');
        xlabel('Time [s]');
        ylabel('Angle [Deg]');
        % ylim([-3 3])
        % xlim([-100 100])
        grid on;
        title('Roll');
        
        subplot(3,2,4)
        hold on
        p14 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,5)));
        p15 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,5)));
        legend('True Roll rate','Estimated Roll rate');
        xlabel('Time [s]');
        ylabel('Angle rate [Deg/s]');
        %ylim([-3 3])
        % xlim([0 0])
        grid on;
        title('Rollrate');
        
        % Steer angle and rate
        subplot(3,2,6)
        hold on;
        p16 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,6)));
        p17 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,6)));
        yline(rad2deg(bike_params.max_steer_angle), '--r');
        xlabel('Time [s]')
        ylabel('Angle [Deg]')
        yyaxis right
        plot(Results.ref_states.Time(:,1),rad2deg(Results.ref_states.Data(:,5)))
        ylabel('Rate [Deg/s]')
        legend('True Steer angle','Estimated Steer angle', 'Max Steering Angle','Steer rate')
        % ylim([-3 3])
        % xlim([0 0])
        grid on
        title('Steer angle')
        linkaxes([p1.Parent, p4.Parent, p7.Parent, p11.Parent, p14.Parent, p16.Parent], 'x');

        
        figure('Name', Tnumber);        
        subplot(211);
        hold on;
        %p1 = plot(Results.delta_psi.Time(:,1), rad2deg(Results.delta_psi.Data(:,1)));
        %p2 = plot(Results.delta_ref.Time(:,1), rad2deg(Results.delta_ref.Data(:,1)));
        xlabel('Time [s]');
        ylabel('Angle [Deg]');
        legend('delta psi', 'delta_{ref}', 'Location', 'southeast');
        grid on;
        title('Steer contributions');        
       
        subplot(212);
        hold on;
        p3 = plot(Results.roll_ref.Time(:,1), rad2deg(Results.roll_ref.Data(:,1)));
        %p4 = plot(Results.delta_ref.Time(:,1), rad2deg(Results.delta_ref.Data(:,1)));
        xlabel('Time [s]');
        ylabel('Angle [Deg]');
        legend('roll_{ref}', 'delta_{ref}', 'Location', 'southeast');
        grid on;
        title('Delta_{ref} vs Roll_{ref}');
        
        % Link the axes of the two subplots for synchronization.
        linkaxes([p1.Parent, p3.Parent], 'x');


        figure('Name', Tnumber); 
        hold on;
        plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,7));
        hold on;
        plot(t_ref,Vref);
        xlabel('Time [s]');
        ylabel('Speed [m/s]');
        legend('Estimated Velocity', 'V_{ref}', 'Location', 'southeast');
        grid on;
        title('Velocity');        

        figure('Name', Tnumber); 
        subplot(211);
        hold on;
        plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,1));
        hold on;        
        plot(Results.ref_states.Time(:,1),Results.ref_states.Data(:,1));
        xlabel('Time [s]');
        ylabel('X [m]');
        legend('Estimated X', 'X_{ref}', 'Location', 'southeast');
        grid on;
        title('X coordinate');   

        subplot(212);
        hold on;
        plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,2));
        hold on;
        plot(Results.ref_states.Time(:,1),Results.ref_states.Data(:,2));
        xlabel('Time [s]');
        ylabel('Y [m]');
        legend('Estimated Y', 'Y_{ref}', 'Location', 'southeast');
        grid on;
        title('Y coordinate');   

        figure('Name', Tnumber); 
        subplot(2,1,1);
        hold on;
        plot(Results.longitudinal_error.Time, Results.longitudinal_error.Data);
        xlabel('Time [s]');
        ylabel('Distance [m]');
        grid on;
        title('Longitudinal Error'); 

        % Lateral error plot
        subplot(2,1,2);
        hold on;
        plot(Results.e1.Time, Results.e1.Data);
        xlabel('Time [s]');
        ylabel('Distance [m]');
        title('Lateral error');
        grid on;
        
        
        
 elseif compare_flag == 1
        %% Trajectory
        figure('Name',Tnumber);
        subplot(3,2,[1,3,5]); %[1,3,5] means that they are just one figure
        hold on;
        % Plot the reference path/trajectory:
        plot3(test_curve(:,1),test_curve(:,2),1:length(test_curve(:,1)),'o');
        % Plot the real bike states (Data1=X, Data2=Y):
        plot3(Results.bike_states.Data(:,1),Results.bike_states.Data(:,2),Results.bike_states.Time(:,1));
        % Plot the estimated bike states from Kalman filter (Data1=X, Data2=Y):
        plot3(Results.estimated_states.Data(:,1),Results.estimated_states.Data(:,2),Results.estimated_states.Time(:,1));
        data_1_real = readtable('bikedata_sim_real_states_1.csv');
        plot3(data_1_real.X,data_1_real.Y,data_1_real.Time);
        data_1_est = readtable('bikedata_sim_est_1.csv');
        plot3(data_1_est.X_estimated,data_1_est.Y_estimated,data_1_est.Time);
        view(0,90)
        % If you want to plot other data you should change the legend as you wish
        legend('Ref','True', 'Est','True 2nd', 'Est 2nd', 'Location','northwest');
        xlabel('X-dir [m]');
        ylabel('Y-dir [m]');
        % ylim([traj_plot.ymin traj_plot.ymax])
        % xlim([traj_plot.xmin traj_plot.xmax])
        axis equal
        % xlim([-50 50])
        % ylim([-10 10]
        grid on;
        title('Trajectory');
        
        
        subplot(3,2,2)
        p1 = plot(Results.error1.Time,Results.error1.Data)
        xlabel('Time [s]')
        ylabel('Distance [m]')
        title('Lateral error')
        
        subplot(3,2,4)
        hold on
        p2 = plot(Results.error2.Time,rad2deg(Results.error2.Data))
        % plot(Results.dpsiref_steer.Time,rad2deg(Results.dpsiref_steer.Data))
        xlabel('Time [s]')
        ylabel('Angle [deg]')
        grid on
        title('Heading error')
        
        subplot(3,2,6)
        hold on
        p3 = plot(Results.ids.Time,Results.ids.Data)
        p4 = plot(Results.closest_point.Time,Results.closest_point.Data-1)
        p5 = plot(Results.closestpoint_heading.Time,Results.closestpoint_heading.Data-2)
        legend( 'Selected idx Total','Selected idx in local','Selected idx for heading','Location','northwest')
        xlabel('Time [s]')
        ylabel('Index [-]')
        grid on
        title('Closest point index selection')

        linkaxes([p1.Parent, p2.Parent, p3.Parent], 'x');
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure('Name',Tnumber);
        
        % X, Y, Psi
        subplot(3,2,1)
        hold on;
        p1 = plot(Results.ref_states.Time(:,1),Results.ref_states.Data(:,1));
        p2 = plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,1));
        p3 = plot(Results.estimated_states.Time(:,1),Results.estimated_states.Data(:,1));
        legend('Ref X','True X','Estimated X');
        xlabel('Time [t]');
        ylabel('Position X [m]');
        % ylim([-50 50])
        % xlim([-100 100])
        grid on;
        title('X-coordinate');
        
        subplot(3,2,3);
        hold on;
        p4 = plot(Results.ref_states.Time(:,1),Results.ref_states.Data(:,2));                
        p5 = plot(Results.bike_states.Time(:,1),Results.bike_states.Data(:,2));              
        p6 = plot(Results.estimated_states.Time(:,1),Results.estimated_states.Data(:,2));    
        legend('Ref Y','True Y','Estimated Y');
        xlabel('Time [t]');
        ylabel('Position Y [m]');
        % ylim([-50 50])
        % xlim([-100 100])
        grid on;
        title('Y-coordinate');
        
        subplot(3,2,5)
        hold on;
        p7 = plot(Results.ref_states.Time(:,1),rad2deg(Results.ref_states.Data(:,3)));
        p8 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,3)));
        p9 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,3)));
        p10 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,3)));
        
        legend('Ref psi','True psi','Estimated psi');
        xlabel('Time [t]');
        ylabel('Angle [Deg]');
        % ylim([-50 50])
        % xlim([-100 100])
        grid on;
        title('Heading');
        
        % Roll and Roll_rate
        subplot(3,2,2)
        hold on;
        p11 = plot(Results.roll_ref.Time(:,1),rad2deg(Results.roll_ref.Data(:,1)));
        p12 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,4)));
        p13 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,4)));
        p14 = plot(data_1_real.Time,rad2deg(data_1_real.Roll));
        p15 = plot(data_1_est.Time,rad2deg(data_1_est.Roll_estimated));         
        legend('Roll Ref','True Roll','Est. Roll','True Roll 2nd','Est. Roll 2nd');
        xlabel('Time [t]');
        ylabel('Angle [Deg]');
        % ylim([-3 3])
        % xlim([-100 100])
        grid on;
        title('Roll');
        
        subplot(3,2,4)
        hold on
        p16 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,5)));
        p17 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,5)));
        legend('True Roll rate','Estimated Roll rate');
        xlabel('Time [t]');
        ylabel('Angle rate [Deg/s]');
        %ylim([-3 3])
        % xlim([0 0])
        grid on;
        title('Rollrate');
        
        % Steer angle and rate
        subplot(3,2,6)
        hold on;
        p18 = plot(Results.bike_states.Time(:,1),rad2deg(Results.bike_states.Data(:,6)));
        p19 = plot(Results.estimated_states.Time(:,1),rad2deg(Results.estimated_states.Data(:,6)));
        xlabel('Time [t]')
        ylabel('Angle [Deg]')
        yyaxis right
        plot(Results.ref_states.Time(:,1),rad2deg(Results.ref_states.Data(:,5)))
        ylabel('Rate [Deg/s]')
        legend('True Steer angle','Estimated Steer angle', 'Steer rate')
        % ylim([-3 3])
        % xlim([0 0])
        grid on
        title('Steer angle')

        linkaxes([p1.Parent, p4.Parent, p7.Parent, p11.Parent, p16.Parent, p18.Parent], 'x');


        
        figure('Name', Tnumber);
        subplot(211)
        hold on;
        p1 = plot(Results.delta_e1.Time(:,1),rad2deg(Results.delta_e1.Data(:,1)));
        p2 = plot(Results.delta_e2.Time(:,1),rad2deg(Results.delta_e2.Data(:,1)));
        %p3 = plot(Results.delta_psi.Time(:,1),rad2deg(Results.delta_psi.Data(:,1)));
        %p4 = plot(Results.delta_ref.Time(:,1),rad2deg(Results.delta_ref.Data(:,1)));
        xlabel('Time [t]')
        ylabel('Angle [Deg]')
        legend('delta e_1','delta e_2','delta psi','delta_{ref}','Location','southeast')
        grid on
        title('Steer contributions')
        
        subplot(212)
        hold on;
        p5 = plot(Results.roll_ref.Time(:,1),rad2deg(Results.roll_ref.Data(:,1)));
        %p6 = plot(Results.delta_ref.Time(:,1),rad2deg(Results.delta_ref.Data(:,1)));
        xlabel('Time [t]')
        ylabel('Angle [Deg]')
        legend('roll_{ref}','delta_{ref}','Location','southeast')
        grid on
        title('Delta_{ref} vs Roll_{ref}')

        linkaxes([p1.Parent, p5.Parent], 'x');

    else 
         msg = 'compare_flag must be 1 or 0';
         error (msg)
    end
end