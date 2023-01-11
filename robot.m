classdef robot < handle
    properties (SetAccess=protected, GetAccess=public)
        
        %Read Only
        
        r = 0;
        front_servo = 0;
        back_servo = 0;
        min_angles = struct("back_servo",1,"front_servo",-89); % the minimum servo angles
        max_angles = struct("back_servo",179,"front_servo",89); % the minimum servo angles
        length_back = 9; %the length (cm) of the first segment
        length_front = 9 %the length (cm) of the second segment
        workspace = 0; % define a workspace (consisting of boundary coordinates) dimension (k,2)
        q; % joint configuration, q(1) --> back_servo, q(2) --> front_servo
        offsets = struct("back_servo",0,"front_servo",0); % the angular offsets (deg) for both servos, configure at start
        
    end  
    
    methods
        %% Methods that change the robots properties
        function obj = connectRaspi(obj)
            %This function tries to connect to the raspi and sets r,
            %front_servo, back_servo
              if isnumeric(obj.r)
                    disp("raspi not connected, trying to connect...");
                    obj.r = raspi;
                    obj.front_servo = servo(obj.r,23);
                    obj.back_servo = servo(obj.r,22);
                    obj.front_servo.MaxPulseDuration = 0.00250;
                    obj.front_servo.MinPulseDuration = 0.0005;
                    obj.back_servo.MaxPulseDuration = 0.00250;
                    obj.back_servo.MinPulseDuration = 0.0005;
                    
                    obj.calculateWorkspace;
                    obj.setEndeffektorPosition(0,obj.length_back+obj.length_front);
                    disp("Set initial position");
                    disp("Please use the the setOffset method to calibrate the initial position", ...
                         "Until the arm is fully stretched straight forward");
                    
              else
                  disp("raspi already connected")
              end
        end
        
        function obj = setFrontAngle(obj, angle)
            %this function tries to set the angle of the front servo and
            %sets q(2)
            
            
            if angle > obj.max_angles.front_servo || angle < obj.min_angles.front_servo
                disp("Front Angle could not be set, out of range");
                return
            end
            
            %subtract offset
            angle_with_offset = angle - obj.offsets.front_servo;

            %min max the angle with offset to max range
            angle_with_offset = min(max(angle_with_offset,obj.min_angles.front_servo),obj.max_angles.front_servo);

            %set property
            obj.q(2) = deg2rad(angle_with_offset);

            %write position to servo (convert angle for servo)
            obj.front_servo.writePosition(90+angle_with_offset);
    
        end
        
        function obj = setBackAngle(obj, angle)
            %this function tries to set the angle of the back servo and
            %sets q(1)
            
            if angle > obj.max_angles.back_servo || angle < obj.min_angles.back_servo
                disp("Back Angle could not be set, out of range");
                return
            end
            
            %subtract offset
            angle_with_offset = angle - obj.offsets.back_servo;

            %min max the angle with offset to max range
            angle_with_offset = min(max(angle_with_offset,obj.min_angles.back_servo),obj.min_angles.back_servo);

            %set property
            obj.q(1) = deg2rad(angle_with_offset);

            %write position to servo
            obj.front_servo.writePosition(angle_with_offset);
           
    
        end
        
        function obj = calculateWorkspace(obj)
            %this function calculates the boundary of all points that the
            %endeffektor can reach, sets workspace
            
            
                step_size = 1; %deg

                n_reachable_coordinates = length(obj.min_angles.back_servo:step_size:obj.max_angles.back_servo)...
                    *length(obj.min_angles.front_servo:step_size:obj.max_angles.front_servo);


                new_reachable_coordinates = zeros(2,n_reachable_coordinates);

                i = 1;
                for q_1 = obj.min_angles.back_servo:step_size:obj.max_angles.back_servo
                    for q_2 = obj.min_angles.front_servo:step_size:obj.max_angles.front_servo

                        [xEE, yEE] = obj.forwardKinematics(deg2rad([q_1;q_2]));

                        new_reachable_coordinates(:,i) = [xEE; yEE];
                        i = i + 1;
                    end
                end
                
                % get indices of boundary
                k = boundary(new_reachable_coordinates(1,:)',new_reachable_coordinates(2,:)');
                % define a workspace (consisting of boundary coordinates) dimension (k,2)
                obj.workspace = [new_reachable_coordinates(1,k)' new_reachable_coordinates(2,k)']; 
                        
                disp("Finished calculating workspace");

        end
        
        function obj = setOffset(offset_front,offset_back)
            
            obj.offsets.front_servo = offset_front;
            obj.offsets.back_srevo = offset_back;
            
            obj.setEndeffektorPosition(0,obj.length_back+obj.length_front);
            disp("Reset initial position");
            
            
            
        end
        
        %% Methods that don't change the robots properties
        function inWorkspace = checkInWorkspace(obj,x,y)
            %This function checks if a given point is located inside the
            %calculated workspace
            
            if ~obj.workspace
                disp("workspace not calculated, run calculateWorkspace first");
                inWorkspace = 0;
                return 
            end
            
            inWorkspace = inpolygon(x,y,obj.workspace(:,1),obj.workspace(:,2));
            
        end
        
        function [xEE, yEE] = forwardKinematics(obj,q)
            %Returns the Endeffektor postion for a given Joint
            %Configuration
            
            a1 = obj.length_back;
            a2 = obj.length_front;
            
            xEE = a1*cos(q(1)) + a2*cos(q(1)+q(2));
            yEE = a1*sin(q(1)) + a2*sin(q(1)+q(2));
            
            
        end
        
        function [q_1_calc, q_2_calc] = inverseKinematics(obj,x_EE,y_EE)
            %Returns the Joint Configuration for a given Endeffektor
            %Position (2 Solutions in general). Checks first if the Endeffektor position is in the
            %calculated workspace
            
            if ~obj.checkInWorkspace(x_EE,y_EE)
                q_1_calc = NaN;
                q_2_calc = NaN;
                disp("Point not in calculated workspace");
                return
            end

            a1 = obj.length_back;
            a2 = obj.length_front;
            
            x = x_EE;
            y = y_EE;
            
            q_1_calc(2) = acos((x^2 + y^2 - a1^2-a2^2)/(2*a1*a2));
            q_1_calc(1) = atan(y/x) - atan((a2*sin(q_1_calc(2)))/(a1+a2*cos(q_1_calc(2))));
            
            q_2_calc(2) = -acos((x^2+y^2-a1^2-a2^2)/(2*a1*a2));
            q_2_calc(1) = atan(y/x) + atan((a2*sin(q_2_calc(2)))/(a1+a2*cos(q_2_calc(2))));
        end
        
        function setEndeffektorPosition(obj,x_EE,y_EE)
            
            % Check if point is in workspace
            if ~obj.checkInWorkspace(x_EE,y_EE)
                disp("Point not in calculated workspace");
                return
            end
            
            % Calculate IK, ignore second solution
            [q_1, ~] = obj.inverseKinematics(x_EE,y_EE);
            
            % Set the calculated angles
            obj.setFrontAngle(q_1(2))
            obj.setBackAngle(q_1(2))
            
        end
        
        %% Methods for plotting
        function plotRobot(obj)
            
            %Position of first joint (first solution)
            x_1 = obj.length_back*cos(obj.q(1));
            y_1 = obj.length_back*sin(obj.q(1));

            x_EE = obj.length_back*cos(obj.q(1)) + obj.length_front*cos(obj.q(1)+obj.q(2));
            y_EE = obj.length_back*sin(obj.q(1)) + obj.length_front*sin(obj.q(1)+obj.q(2));


            %% Draw solution
            %Plot first segment
            plot([0 x_1], [0 y_1],'r')
            hold on
            %Plot second segment
            plot([x_1 x_EE], [y_1 y_EE],'b')
            %Plot Joints
            plot(0,0,'-o','MarkerSize',8,...
                'MarkerEdgeColor','black',...
                'MarkerFaceColor','black')
            plot(x_1,y_1,'-o','MarkerSize',8,...
                'MarkerEdgeColor','black',...
                'MarkerFaceColor','black')
            plot(x_EE,y_EE,'-o','MarkerSize',8,...
                'MarkerEdgeColor','green',...
                'MarkerFaceColor','green')

            %Plot settings
            axis equal
            grid on
            xlim([-2*obj.length_front 2*obj.length_front])
            ylim([-1 2*obj.length_front+1])

            %Print joint angles
            caption = sprintf('alpha = %.1f, beta = %.1f        x = %.2f, y = %.2f', rad2deg(obj.q(1)), rad2deg(obj.q(2)), x_EE, y_EE);
            title(caption, 'FontSize', 10);

            
        end     
        
        function plotWorkspace(obj)
            %This function plots the calculated workspace
            
            if ~obj.workspace
                disp("workspace not calculated, run calculateWorkspace first");
                return
            end
            
            plot(obj.workspace(:,1),obj.workspace(:,2));
            
        end

    end
end
