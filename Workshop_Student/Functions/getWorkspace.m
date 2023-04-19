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



%% END TODO

% Calculate the boundary of the the workspace
% The boundary function in MATLAB computes the boundary of a set of points in a 2D plane.
% In this case, boundary_points returns the indices of the points that form the boundary of the input points.
boundary_points = boundary(x_positions', y_positions');

% Get te actual points composing the boundary
x_boundary = x_positions(boundary_points);
y_boundary = y_positions(boundary_points);

end
