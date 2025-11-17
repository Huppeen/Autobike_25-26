function smalltest()

    % Show all subplots and buttons initially
    draw_all_subplots();

    % Draw 7 subplots and place buttons in the 8th slot
    function draw_all_subplots()
        clf;
        fig = figure('Name', 'Interactive Subplot Viewer', 'NumberTitle', 'off');
        numPlots = 7;
        ax = gobjects(numPlots,1);  % Store subplot handles

        % Plot the first 7 subplots
        for i = 1:numPlots
            ax(i) = subplot(4,2,i);
            plot(rand(10,1));  % Example data
            title(['Plot ' num2str(i)]);
        end

        % Use the 8th subplot area for buttons
        ax_btn = subplot(4,2,8);
        btn_pos = get(ax_btn, 'Position');  % Get position for placing buttons
        delete(ax_btn);  % Remove axes, keep space

        % Create buttons labeled 1~7
        for i = 1:numPlots
            uicontrol('Style', 'pushbutton', ...
                      'String', num2str(i), ...
                      'Units', 'normalized', ...
                      'Position', [btn_pos(1) + 0.012 + (i-1)*(btn_pos(3)/numPlots), ...
                                   btn_pos(2) + btn_pos(4)/3, ...
                                   btn_pos(3)/numPlots - 0.005, ...
                                   btn_pos(4)/2.2], ...
                      'FontSize', 9, ...
                      'Callback', @(src, event) zoom_in_subplot(i));
        end
    end

    % Show the selected subplot in full size
    function zoom_in_subplot(idx)
        clf;
        axes('Position', [0.07, 0.15, 0.85, 0.8]);
        plot(rand(10,1));  % Example data
        title(['Zoomed Plot ' num2str(idx)], 'FontSize', 14);

        % Back button to restore all subplots
        uicontrol('Style', 'pushbutton', ...
                  'String', 'Back', ...
                  'Units', 'normalized', ...
                  'Position', [0.88, 0.93, 0.1, 0.05], ...
                  'Callback', @(src, event) draw_all_subplots());
    end
end
