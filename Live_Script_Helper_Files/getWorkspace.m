function [x_boundary, y_boundary] = getWorkspace()

% Joint angle limits
angle_back_min = 0;
angle_back_max = 180;
angle_front_min = -90;
angle_front_max = 90;

% Sampling resolution
resolution = 1; % degrees

% Initialize arrays for storing the end-effector positions
x_positions = [];
y_positions = [];

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

% Calculate the boundary of the workspace
boundary_points = boundary(x_positions', y_positions');

x_boundary = x_positions(boundary_points);
y_boundary = y_positions(boundary_points);

end
