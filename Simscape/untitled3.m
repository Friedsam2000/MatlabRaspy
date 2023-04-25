classdef untitled3

    properties
        min_angles = struct("back_servo",1,"front_servo",-89); 
        max_angles = struct("back_servo",179,"front_servo",89); 
        workspace = [0 0];
        workspace_figure = struct("WindowState",'closed');
        desired_position_counter = 0;
        length_front;
        length_back;
        q1;
        q2;
        offsets = struct("back_servo",0,"front_servo",0);
    end

    methods
        function obj = untitled3(robot)
            obj.min_angles = struct("back_servo",robot.Bodies{1,1}.Joint.PositionLimits(1),...
                "front_servo",robot.Bodies{1,2}.Joint.PositionLimits(1)); 
            obj.max_angles = struct("back_servo",robot.Bodies{1,1}.Joint.PositionLimits(2),...
                "front_servo",robot.Bodies{1,2}.Joint.PositionLimits(2));

            obj.length_front = sqrt(robot.Bodies{1,2}.Joint.JointToParentTransform(1,4)^2 +...
                robot.Bodies{1,2}.Joint.JointToParentTransform(2,4)^2);
            obj.length_back = sqrt(robot.Bodies{1,3}.Joint.JointToParentTransform(1,4)^2 + ...
                robot.Bodies{1,3}.Joint.JointToParentTransform(2,4)^2);
        end

        function [xEE, yEE] = forwardKinematics(obj,q1,q2)
            a1 = obj.length_back;
            a2 = obj.length_front;
            
            xEE = a1*cos(q1) + a2*cos(q1+q2);
            yEE = a1*sin(q1) + a2*sin(q1+q2);
        end

        function [q_1_calc, q_2_calc] = inverseKinematics(obj,x_EE,y_EE)            
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

        function updateRobotInPlot(obj)                        
            %Position of first joint 
            x_1 = obj.length_back*cos(obj.q1);
            y_1 = obj.length_back*sin(obj.q1);

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
 
            caption = sprintf('q_1 = %.1f °, q_2 = %.1f °       x_E_E = %.2f cm, y_E_E = %.2f cm',...
                rad2deg(obj.q1), rad2deg(obj.q2), x_EE, y_EE);            
            set(title_handle,'string',caption);            
            drawnow           
        end

        function obj = setAngleFront(obj, angle)                       
            %min max the angle to max range
            angle = min(max(angle,obj.min_angles.front_servo),obj.max_angles.front_servo);
            
            %set property
            obj.q2 = deg2rad(angle);            
        end

        function obj = setAngleBack(obj, angle)
            %min max the angle to max range
            angle = min(max(angle,obj.min_angles.back_servo),obj.max_angles.back_servo);
            
            %set property
            obj.q1 = deg2rad(angle); 
        end

        function setEndeffektorPosition(obj,x_EE,y_EE)
            [q_1_calc, q_2_calc] = obj.inverseKinematics(x_EE,y_EE);
            
            if rad2deg(q_1_calc(1)) > obj.max_angles.back_servo || rad2deg(q_1_calc(1)) < obj.min_angles.back_servo || rad2deg(q_1_calc(2)) > obj.max_angles.front_servo || rad2deg(q_1_calc(2)) < obj.min_angles.front_servo
                disp("using alterative solution");
                obj.setAngleFront(rad2deg(q_2_calc(2)),1);
                obj.setAngleBack(rad2deg(q_2_calc(1)),1);
            else
                obj.setAngleFront(rad2deg(q_1_calc(2)),1);
                obj.setAngleBack(rad2deg(q_1_calc(1)),1);
            end

            updateRobotInPlot(obj)
        end

        function pointClickCallback(obj)
            obj.desired_position_counter = obj.desired_position_counter + 1;
    
            pt = get(gca,'CurrentPoint');
            fprintf('Clicked: %.1f %.1f\n', pt(1,1), pt(1,2));
            obj.setEndeffektorPosition(pt(1,1),pt(1,2)); 
        end

        function obj = calculateWorkspace(obj)
            step_size = 1; %mm
            n_reachable_coordinates = length(obj.min_angles.back_servo:step_size:obj.max_angles.back_servo)...
                *length(obj.min_angles.front_servo:step_size:obj.max_angles.front_servo);
            new_reachable_coordinates = zeros(2,n_reachable_coordinates);

            i = 1;
            for q_1 = obj.min_angles.back_servo:step_size:obj.max_angles.back_servo
                for q_2 = obj.min_angles.front_servo:step_size:obj.max_angles.front_servo
                    xEE = obj.length_back*cos(q_1) + obj.length_front*cos(q_1+q_2);
                    yEE = obj.length_back*sin(q_1) + obj.length_front*sin(q_1+q_2);
                    new_reachable_coordinates(:,i) = [xEE; yEE];
                    i = i + 1;
                end
            end
            
            k = boundary(new_reachable_coordinates(1,:)',new_reachable_coordinates(2,:)');
            obj.workspace = [new_reachable_coordinates(1,k)' new_reachable_coordinates(2,k)']; 
        end

        function plotRobot(obj)
            %Position of first joint 
            x_1 = obj.length_back*cos(obj.q1);
            y_1 = obj.length_back*sin(obj.q1);

            %position of the second joint
            [x_EE,y_EE] = obj.forwardKinematics(obj.q1,obj.q2);

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
            caption = sprintf('q_1 = %.1f °, q_2 = %.1f °       x_E_E = %.2f cm, y_E_E = %.2f cm',...
                rad2deg(obj.q1), rad2deg(obj.q2), x_EE, y_EE);      
            title(caption);    
            
            %Print text
            text(0, -0.9*obj.length_back, 'Click in the workspace to move the robot', ...
              'VerticalAlignment', 'bottom', ...
              'HorizontalAlignment', 'center','fontsize',13,'color','blue');
        end    


        function plotRobotInWorkspace(obj)
            calculateWorkspace(obj);
            
            figure
            plot(obj.workspace(:,1),obj.workspace(:,2));
            % hold on
            % plotRobot(obj);      
        end
    end
end