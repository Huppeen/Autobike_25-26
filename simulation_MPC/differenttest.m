function [Xref,Yref,Psiref,t_ref] = differenttest(testnumber,ref_dis,lL,laps,Vref_test)
    switch testnumber   
        case '1'            
            % Generate a straight-line trajectory with the function (ReferenceGenerator)
            [Xref,Yref,Psiref] = ReferenceGenerator('line',ref_dis,lL,laps);
            % Time vector calculated based on trajectory length and speed
            t_ref = linspace(0, (lL-1)/Vref_test, lL)';
        case '2' 
            % Generate a straight-line with a sharp turn trajectory with the function (ReferenceGenerator)
            [Xref,Yref,Psiref] = ReferenceGenerator('sharp_turn',ref_dis,lL,laps);
            % Time vector calculated based on trajectory length and speed
            t_ref = linspace(0, (lL-1)/Vref_test, lL)';
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
    end
end
