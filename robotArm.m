classdef robotArm < handle
    
    %% Private Properties
    properties (SetAccess=protected, GetAccess=protected)
        front_servo = 0;
        back_servo = 0;
        min_angles = struct("back_servo",1,"front_servo",-89); % the minimum servo angles
        max_angles = struct("back_servo",179,"front_servo",89); % the minimum servo angles
        length_back = 9; %the length (cm) of the first segment
        length_front = 9 %the length (cm) of the second segment
        workspace = 0; % define a workspace (consisting of boundary coordinates) dimension (k,2)
        workspace_figure = struct("WindowState",'closed');
        raspberry_pi = 0;
        desired_position_counter = 0;

    end 
    
    %% Read-Only Properties
    properties (SetAccess=protected, GetAccess=public)
        
        q; % joint configuration, q(1) --> back_servo, q(2) --> front_servo

    end
    
    %% Settable Properties
    properties (SetAccess=public, GetAccess=public, SetObservable)
        
        control_mode = 1; % 0 = Analytic ; 1 = P Controller with Jacobian
        offsets = struct("back_servo",0,"front_servo",0); % the angular offsets (deg) for both servos, configure at start

    end
    
    %% Public Methods
    methods
        %% Constructor
            
        function obj = robotArm(evtobj)
            obj.connectRaspi();
            addlistener(obj,'control_mode','PostSet',@(src,evt) obj.ModeChangedCallback(src,evt));
        end

        
        
        %% Setup
        function obj = connectRaspi(obj)
            %This function tries to connect to the raspi and sets r,
            %front_servo, back_servo
              if isnumeric(obj.raspberry_pi)
                    disp("raspi not connected, trying to connect...");
                    try
                    obj.raspberry_pi = raspi;            
                    obj.front_servo = servo(obj.raspberry_pi,23);
                    obj.back_servo = servo(obj.raspberry_pi,22);
                    obj.front_servo.MaxPulseDuration = 0.00250;
                    obj.front_servo.MinPulseDuration = 0.0005;
                    obj.back_servo.MaxPulseDuration = 0.00250;
                    obj.back_servo.MinPulseDuration = 0.0005;    
                    
                    catch ME
                        disp(ME.message)
                        disp("Could not connect to raspi, running in simulation mode, run connectRaspi if you want to connect.");
                    end
                    
                    fprintf("--------------------------------------------------------------------------------------------------------------\n");
                    obj.calculateWorkspace;
                    obj.setAngleBack(rad2deg(pi/3));
                    obj.setAngleFront(rad2deg(pi/3));
                    disp("Set initial position (non singularity): q = [pi/3; pi/3];");
                    obj.plotRobotInWorkspace;
              else
                  disp("raspi already connected")
              end
              

        end
        
        %% Interaction  
        function setEndeffektorPosition_P_Controller(obj,x_EE,y_EE,P)
            %Set the endeffektor Position of the robot using the pinv of jacobian (experimental)
            % Using a P Controller
            % This method does not require an analytical solution to the
            % inverse kinematic.
            
            % Check if point is in workspace
            if ~obj.checkInWorkspace(x_EE,y_EE)
                disp("Point not in calculated workspace");
                return
            end
            
            
            tolerance = 0.1; %cm
            
            %Defines the control loop frequency f = 1/dt
            dt = 0.01; %s
            
            %Defines the max time befor it aborts
            max_convergence_time = 0.5;
            
            %Control loop
            t = 0;
            
            %get current desired position count
            current_desired_position_counter = obj.desired_position_counter;
            while 1
                
                %compute current endeffektor pos
                [x_Cur, y_Cur] = obj.forwardKinematics(obj.q);
                
                %compute distance
                distance = sqrt((x_Cur-x_EE)^2+(y_Cur-y_EE)^2);
                
                
                %stop when position is in tolerance
                if (distance < tolerance)
                    disp("Desired Endeffetor Position Reached");
                    break;
                end
                
                %Stop when max time has passed
                if (t>max_convergence_time)
                    disp("Did not converge in time");
                    break;
                end
                
                if current_desired_position_counter < obj.desired_position_counter
                    break;
                end
                
                %Calculate the Control error
                position_error = [x_EE-x_Cur; y_EE-y_Cur];
                
                %P Controller
                x_dot = P*position_error; %Desired task space velocity
                        
                %Convert to Joint Space
                J = obj.getJacobian();  
                q_dot = pinv(J) * x_dot; %Desired joint space velocity
                
                %Set the updated Angles by integrating the desired joint
                %space velocity
                obj.setAngleBack(rad2deg(obj.q(1)+dt*q_dot(1)));
                obj.setAngleFront(rad2deg(obj.q(2)+dt*q_dot(2)));
                    
                %Propagate real and simulated time
                pause(dt);
                t = t + dt;
                
                %Update Plot
                updateRobotInPlot(obj)

            end

        end
        
        function setEndeffektorPosition_Analytic(obj,x_EE,y_EE)
            %Set the endeffektor Position of the robot using inverse
            %Kinematics, checks if point is in workspace first
            
            % Check if point is in workspace
            if ~obj.checkInWorkspace(x_EE,y_EE)
                disp("Point not in calculated workspace");
                return
            end
            
            % Calculate IK
            [q_1_calc, q_2_calc] = obj.inverseKinematics(x_EE,y_EE);
            
            %check if q_1 is out of bounds
            if rad2deg(q_1_calc(1)) > obj.max_angles.back_servo || rad2deg(q_1_calc(1)) < obj.min_angles.back_servo || rad2deg(q_1_calc(2)) > obj.max_angles.front_servo || rad2deg(q_1_calc(2)) < obj.min_angles.front_servo
                % use q_2 instead (second solution to IK
                disp("using alterative solution");
                obj.setAngleFront(rad2deg(q_2_calc(2)));
                obj.setAngleBack(rad2deg(q_2_calc(1)));
            else
                % use q_1
                obj.setAngleFront(rad2deg(q_1_calc(2)));
                obj.setAngleBack(rad2deg(q_1_calc(1)));
            end

            updateRobotInPlot(obj)
            
        end
        
        function followLine(obj,x_start,x_end,y)
            %Follow a horizontal line
            
            step_size = 0.5; % cm
            for x = x_start:step_size:x_end
                
                obj.setEndeffektorPosition_Analytic(x,y);
                
                updateRobotInPlot(obj)
                
            end
            
        end
               
    end
    
     %% Private Methods 
    methods (Access=private)
        %% Analytics
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
            
            %Define Variables
            L_1 = obj.length_back;
            L_2 = obj.length_front;
            XE = x_EE;
            YE = y_EE;
            
            %IK equations from https://de.mathworks.com/help/symbolic/derive-and-apply-inverse-kinematics-to-robot-arm.html
            %first solution
            q_1_calc(1) = atan((L_1.*YE.*2.0-(L_1.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)-(L_2.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)+(XE.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)+(YE.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)+(L_1.*L_2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)).*2.0)./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0))./(L_1.*XE.*2.0+L_1.^2-L_2.^2+XE.^2+YE.^2)).*2.0;
            q_1_calc(2) = atan(sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)).*-2.0;
            %second solution
            q_2_calc(1) = atan((L_1.*YE.*2.0+(L_1.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)+(L_2.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)-(XE.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)-(YE.^2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)-(L_1.*L_2.*sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0)).*2.0)./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0))./(L_1.*XE.*2.0+L_1.^2-L_2.^2+XE.^2+YE.^2)).*2.0;
            q_2_calc(2) = atan(sqrt((-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0).*(L_1.^2+L_2.^2-XE.^2-YE.^2+L_1.*L_2.*2.0))./(-L_1.^2-L_2.^2+XE.^2+YE.^2+L_1.*L_2.*2.0)).*2.0;
        end
        
        function J = getJacobian(obj)
            %Calculates the jacobian (x_dot = J*q_dot)
        
            l_1 = obj.length_back;
            l_2 = obj.length_front;
            alpha = obj.q(1);
            beta = obj.q(2);
            
            J = [-l_1*sin(alpha)-l_2*sin(alpha+beta),-l_2*sin(alpha+beta);
                  l_1*cos(alpha)+l_2*cos(alpha+beta), l_2*cos(alpha+beta)];
        end
        
        function inWorkspace = checkInWorkspace(obj,x,y)
            %Checks if a given point is located inside the
            %calculated workspace
            
            if ~obj.workspace
                disp("workspace not calculated, run calculateWorkspace first");
                inWorkspace = 0;
                return 
            end
            
            inWorkspace = inpolygon(x,y,obj.workspace(:,1),obj.workspace(:,2));
        end
        
        %% Other Methods
        
        function ModeChangedCallback(obj,src,evt)
            
            obj.updateRobotInPlot;
        end


        function obj = setAngleFront(obj, angle)
            %DOES NOT UPDATE PLOT ITSELF
            %this function tries to set the angle of the front servo and
            %sets q(2)
                   
            
            %check if commanded angle is within bounds
            if angle > obj.max_angles.front_servo || angle < obj.min_angles.front_servo
                disp("Warning: Front Angle out of bounds, limiting");
            end
            
            %subtract offset
            angle_with_offset = angle - obj.offsets.front_servo;

            %min max the angle_with_offset to max range
            angle_with_offset = min(max(angle_with_offset,obj.min_angles.front_servo),obj.max_angles.front_servo);            %min max the angle to max range
           
            %min max the angle to max range
            angle = min(max(angle,obj.min_angles.front_servo),obj.max_angles.front_servo);
            
            %set property
            obj.q(2) = deg2rad(angle);

            %write position to servo if connected(convert angle for servo)
            if ~isnumeric(obj.back_servo)
                obj.front_servo.writePosition(90-angle_with_offset);
            end
            
    
        end
        
        function obj = setAngleBack(obj, angle)
            %DOES NOT UPDATE PLOT ITSELF
            %this function tries to set the angle of the back servo with offset and
            %sets q(1)
            
            %check if  angle is within bounds, give warning
            if angle > obj.max_angles.back_servo || angle < obj.min_angles.back_servo
                disp("Warning: Back Angle out of bounds, limiting");
            end

            %subtract offset
            angle_with_offset = angle - obj.offsets.back_servo;

            %min max the angle_with_offset to max range
            angle_with_offset = min(max(angle_with_offset,obj.min_angles.back_servo),obj.max_angles.back_servo);

            %min max the angle to max range
            angle = min(max(angle,obj.min_angles.back_servo),obj.max_angles.back_servo);
            
            %set property
            obj.q(1) = deg2rad(angle);

            %write position to servo if connected
            if ~isnumeric(obj.back_servo)
                obj.back_servo.writePosition(angle_with_offset);
            end
            

    
        end
        
        function obj = calculateWorkspace(obj)
            %this function calculates the boundary of all points that the
            %endeffektor can reach, sets workspace
            
            
                step_size = 1; %mm

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
                        
                disp("Finished calculating workspace.");

        end
        
        function pointClickCallback(obj,~, ~,~)
            %A callback when the workspace figure is clicked
            
            %increment desired position counter
            obj.desired_position_counter = obj.desired_position_counter + 1;
    
            pt = get(gca,'CurrentPoint');
            fprintf('Clicked: %.1f %.1f\n', pt(1,1), pt(1,2));
            if obj.control_mode
                    obj.setEndeffektorPosition_P_Controller(pt(1,1),pt(1,2),15); %P Mode
            else
                    obj.setEndeffektorPosition_Analytic(pt(1,1),pt(1,2)); %Analytical mode
            end

        end
        
         %% Plotting Methods
        function plotRobot(obj)
            %Plot the robot
            
            %Position of first joint 
            x_1 = obj.length_back*cos(obj.q(1));
            y_1 = obj.length_back*sin(obj.q(1));

            %position of the second joint
            [x_EE,y_EE] = obj.forwardKinematics(obj.q);

            %Plot first segment
            first_segment = plot([0 x_1], [0 y_1],'r');
            set(first_segment,'Tag','first_segment');
            hold on
            %Plot second segment
            second_segment = plot([x_1 x_EE], [y_1 y_EE],'b');
            set(second_segment,'Tag','second_segment');

            %Plot Joints
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
            xlim(1.2*[-obj.length_front-obj.length_back obj.length_front+obj.length_back])
            ylim(1.2*[-obj.length_front-obj.length_back obj.length_front+obj.length_back])

            %Print joint angles
            caption = sprintf('q_1 = %.1f °, q_2 = %.1f °       x_E_E = %.2f cm, y_E_E = %.2f cm', rad2deg(obj.q(1)), rad2deg(obj.q(2)), x_EE, y_EE);
            
            if obj.control_mode == 0
                add_caption = '        Controller: Analytic Controller';
            else
                add_caption = '        Controller: P Controller';
            end
            
            caption = strcat(caption, add_caption);
            
            title(caption);
            
            
  
            
            %Print text
            text(0, -0.9*obj.length_back, 'Click in the workspace to move the robot', ...
              'VerticalAlignment', 'bottom', ...
              'HorizontalAlignment', 'center','fontsize',13,'color','blue');
        end     
        
        function obj = plotWorkspace(obj)
            %Plot the calculated workspace
            
            if ~obj.workspace
                disp("workspace not calculated, run calculateWorkspace first");
                return
            end
            if ~strcmp(obj.workspace_figure.WindowState,'normal')
                obj.workspace_figure = figure(1);
                obj.workspace_figure.Position = [300 0 1100 950];
                set(obj.workspace_figure,'WindowButtonDownFcn',@obj.pointClickCallback);
            end
            plot(obj.workspace(:,1),obj.workspace(:,2));
        end
         
        function plotRobotInWorkspace(obj)
            %Plot both workspace and robot for interaction
            
            hold off
            obj.plotWorkspace;
            hold on
            obj.plotRobot;         
        end

        function updateRobotInPlot(obj)
            
            %Position of first joint 
            x_1 = obj.length_back*cos(obj.q(1));
            y_1 = obj.length_back*sin(obj.q(1));

            %position of the second joint
            [x_EE,y_EE] = obj.forwardKinematics(obj.q);
            
            %Get the handles of the robot graphics
            endpoint_1 = findobj(gca, 'Tag', 'endpoint_1');
            endpoint_2 = findobj(gca, 'Tag', 'endpoint_2');
            first_segment = findobj(gca, 'Tag', 'first_segment');
            second_segment = findobj(gca, 'Tag', 'second_segment');
            
            %Get handle of title
            title_handle = get(gca, 'title');
            
            
            %Update the robot graphics with the new position
            set(endpoint_1,'XData',x_1,'YData',y_1);
            set(endpoint_2,'XData',x_EE,'YData',y_EE);
            set(first_segment, 'XData',[0 x_1], 'YData',[0 y_1]);
            set(second_segment, 'XData',[x_1 x_EE], 'YData',[y_1 y_EE]);
 
            caption = sprintf('q_1 = %.1f °, q_2 = %.1f °       x_E_E = %.2f cm, y_E_E = %.2f cm', rad2deg(obj.q(1)), rad2deg(obj.q(2)), x_EE, y_EE);
            
            if obj.control_mode == 0
                add_caption = '        Controller: Analytic Controller';
            else
                add_caption = '        Controller: P Controller';
            end
            
            caption = strcat(caption, add_caption);
            
            set(title_handle,'string',caption);
            
            drawnow
            
        end
        
    end
end


        

