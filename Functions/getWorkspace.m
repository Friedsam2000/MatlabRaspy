function [x_boundary, y_boundary] = getWorkspace()

% Joint angle limits
angle_back_min = 0;
angle_back_max = 180;
angle_front_min = -90;
angle_front_max = 90;

% Initialize arrays for storing the end-effector positions
x_positions = [];
y_positions = [];

%% TODO: Calculate sample points of where endeffector of the robot can reach. 
%%       Store the points in the x_positions and y_positions array.

% Sampling resolution
resolution = 1; % degrees

% Iterate over possible angles
for angle_back = angle_back_min:resolution:angle_back_max
    for angle_front = angle_front_min:resolution:angle_front_max
        % Calculate the end-effector position
        [x_pos, y_pos] = forwardKinematics(angle_back, angle_front);
        % Store the end-effector position
        x_positions(end + 1) = x_pos;
        y_positions(end + 1) = y_pos;
    end
end

%% END TODO

% Calculate the boundary of the the workspace
% The boundary function in MATLAB computes the boundary of a set of points in a 2D plane.
% In this case, boundary_points returns the indices of the points that form the boundary of the input points.
boundary_points = boundary(x_positions', y_positions');

% Get te actual points composing the boundary
x_boundary = x_positions(boundary_points);
y_boundary = y_positions(boundary_points);

end
