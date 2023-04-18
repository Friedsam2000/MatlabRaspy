function J = getJacobi(angle_back,angle_front)

    L1 = 90; %mm
    L2 = 90; %mm

    angle_back_rad = deg2rad(angle_back);
    angle_front_rad = deg2rad(angle_front);


    %Remember the forward kinematic euations are:
    % x_EE = L1 * cos(angle_back) + L2 * cos(angle_back + angle_front);
    % y_EE = L1 * sin(angle_back) + L2 * sin(angle_back + angle_front);


    %Differentiation with respect to the angles gives:
    
     J = [-(L1*sin(angle_back_rad) + L2*sin(angle_back_rad + angle_front_rad)), -L2*sin(angle_back_rad + angle_front_rad);
              L1*cos(angle_back_rad) + L2*cos(angle_back_rad + angle_front_rad),  L2*cos(angle_back_rad + angle_front_rad)];


end

