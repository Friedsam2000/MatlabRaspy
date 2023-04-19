function J = getJacobi(angle_back,angle_front)

    L1 = 90; %mm
    L2 = 90; %mm

    q1 = deg2rad(angle_back);
    q2 = deg2rad(angle_front);


    %Remember the forward kinematic euations are:
    % x_EE = L1 * cos(q1) + L2 * cos(q1 + q2);
    % y_EE = L1 * sin(q1) + L2 * sin(q1 + q2);

    
    %% TODO: Perform a partial differentiation with respect to q = [q1,q2];
    % J = ...


     %% END TODO



end

