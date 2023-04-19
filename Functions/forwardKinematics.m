function [x_EE, y_EE] = forwardKinematics(angle_back,angle_front)

            angle_back = deg2rad(angle_back);
            angle_front = deg2rad(angle_front);

            l1 = 90; % mm lenght of back segment
            l2 = 90; % mm length of front segment
            
            %Calculate the position of the endeffektor using sine and
            %cosine
            

            %% TODO: Derive a trignometic calculation of the endeffector
            %position (x_EE, y_EE). Checkout the coordinate system in the
            %live script
            x_EE = l1*cos(angle_back) + l2*cos(angle_back+angle_front);
            y_EE = l1*sin(angle_back) + l2*sin(angle_back+angle_front);

            %% END TODO

end