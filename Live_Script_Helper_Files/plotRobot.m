function plotRobot(angle_back, angle_front)
    %Plot the robot
    length_back = 90; %the length (mm) of the first segment
    length_front = 90; %the length (mm) of the second segment

    angle_front = deg2rad(angle_front);
    angle_back = deg2rad(angle_back);
    
    %Position of first joint 
    x_1 = length_back*cos(angle_back);
    y_1 = length_back*sin(angle_back);

    %position of the second joint
    x_EE = length_back*cos(angle_back) + length_front*cos(angle_front+angle_back);
    y_EE = length_back*sin(angle_back) + length_front*sin(angle_front+angle_back);

    % Find or create the figure
    robot_figure = findobj('Tag', 'robot_figure');
    if isempty(robot_figure)
        robot_figure = figure;
        set(robot_figure, 'Tag', 'robot_figure');
    else
        figure(robot_figure);
    end

    % Update the first segment
    first_segment = findobj('Tag','first_segment');
    if isempty(first_segment)
        first_segment = plot([0 x_1], [0 y_1],'r');
        set(first_segment,'Tag','first_segment');
    else
        set(first_segment, 'XData', [0 x_1], 'YData', [0 y_1]);
    end
    hold on

    % Update the second segment
    second_segment = findobj('Tag','second_segment');
    if isempty(second_segment)
        second_segment = plot([x_1 x_EE], [y_1 y_EE],'b');
        set(second_segment,'Tag','second_segment');
    else
        set(second_segment, 'XData', [x_1 x_EE], 'YData', [y_1 y_EE]);
    end

    % Update the first joint position
    endpoint_1 = findobj('Tag','endpoint_1');
    if isempty(endpoint_1)
        endpoint_1 = plot(x_1, y_1, '-o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', 'black', ...
            'MarkerFaceColor', 'black');
        set(endpoint_1, 'Tag', 'endpoint_1');
    else
        set(endpoint_1, 'XData', x_1, 'YData', y_1);
    end

    % Update the end-effector position
    endpoint_2 = findobj('Tag','endpoint_2');
    if isempty(endpoint_2)
        endpoint_2 = plot(x_EE, y_EE, '-o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', 'green', ...
            'MarkerFaceColor', 'green');
        set(endpoint_2, 'Tag', 'endpoint_2');
    else
        set(endpoint_2, 'XData', x_EE, 'YData', y_EE);
    end

    % Plot Joints (this part remains the same as before)
    plot(0,0,'-o','MarkerSize',8,...
        'MarkerEdgeColor','black',...
        'MarkerFaceColor','black');
    
    endpoint_1 = plot(x_1,y_1,'-o','MarkerSize',8,...
        'MarkerEdgeColor','black',...
        'MarkerFaceColor','black');
    set(endpoint_1,'Tag','endpoint_1');

    endpoint_2 = plot(x_EE,y_EE,'-o','MarkerSize',8,...
        'MarkerEdgeColor','green',...
        'MarkerFaceColor','green');
    set(endpoint_2,'Tag','endpoint_2');

    %Plot settings
    axis equal
    grid on
    xlim(1.2*[-length_front-length_back length_front+length_back])
    ylim(1.2*[-length_front-length_back length_front+length_back])

    %Print joint angles
    caption = sprintf('angle back = %.1f °, angle front = %.1f °', rad2deg(angle_back), rad2deg(angle_front));        
    title(caption);

    %Delete marker if exist
    marker_to_delete = findobj('Tag', 'myMarker');
    delete(marker_to_delete);
end
