clc
clf

%% check if raspy is connected, trys to connect

global l_1 l_2 min_angle max_angle front_servo back_servo


[r] = connectRaspy();

l_1 = 9;
l_2 = 9;
min_angle = 1;
max_angle = 179;



%% Main


plotRobotFromPosition(0,l_1*2);
plotAvailableWorkspace();


%% function definitions

%set the angle of a servo -> send to servo
function setAngle(servo, angle)
    
    global min_angle max_angle
    
    %offset for front servo
    if(servo.MaxPulseDuration == 0.00249)
        angle = 180 - angle;
        offset =-5;
    end
    
    %offset for back servo
    if(servo.MaxPulseDuration == 0.00250)
        offset=-1;
    end
    
    position_with_offset = angle - offset;
    
    %make sure to not command an angle out of range
    position_with_offset = min(max(position_with_offset,min_angle),max_angle);
   
    %write position to servo
    servo.writePosition(position_with_offset);
    
end

%check if raspy is connected, if not try to connect, setup front_servo and
%back servo
function [r] = connectRaspy()

    global front_servo back_servo

        raspi_connected = evalin( 'base', 'exist(''r'',''var'') == 1' );
    if ~raspi_connected
        disp("raspi not connected, trying to connect...");
        r = raspi;
        front_servo = servo(r,23);
        back_servo = servo(r,22);
        front_servo.MaxPulseDuration = 0.00249;
        front_servo.MinPulseDuration = 0.0005;

        back_servo.MaxPulseDuration = 0.00250;
        back_servo.MinPulseDuration = 0.0005;
    else
        r = evalin( 'base', 'r');
        back_servo = evalin( 'base', 'back_servo');
        front_servo = evalin( 'base', 'front_servo');
    end
    

end

%try to set the angles of the servo according to a desired end-effector
%position (integers), checks if position is in workspace
function success = setEndPosition(x, y)

    global l_1 l_2 min_angle max_angle front_servo back_servo

    x = round(x);
    y = round(y);
    
    %check if position is contained in reachable coordinates
    reachable_coordinates = getReachableCoordinates();
    if ~ismember([x,y], reachable_coordinates, 'rows')
        disp('Endpoint not in available workspace');
        success = 0;
        return;
    end
    
    syms x_2 y_2 alpha_sym beta_sym

    % equations for endeffector position
    e1= x_2 == l_1 * cos(alpha_sym) + l_2*cos(alpha_sym+beta_sym-pi/2);
    e2= y_2 == l_1 * sin(alpha_sym) + l_2*sin(alpha_sym+beta_sym-pi/2);
    
    %solve analytically (could be replaced by numeric solver)
    [alpha_eqn,beta_eqn]=solve(e1,e2,alpha_sym,beta_sym);
    
    %plug in desired position of endefector and solve numerically for alpha
    %and beta -> two solutions
    x_2 = x;
    y_2 = y;
    alpha_sym = rad2deg(double(subs(alpha_eqn)));
    beta_sym = rad2deg(double(subs(beta_eqn)));
    
    %pick solution that satisfies min max angle requirements
    if ~(beta_sym(1) < min_angle || beta_sym(1) > max_angle || alpha_sym(1) < min_angle || alpha_sym(1) > max_angle)
        
            setAngle(front_servo, beta_sym(1));
            setAngle(back_servo,alpha_sym(1));
            success = 1;

    elseif ~(beta_sym(2) < min_angle || beta_sym(2) > max_angle || alpha_sym(2) < min_angle || alpha_sym(2) > max_angle)
             setAngle(front_servo, beta_sym(2));
             setAngle(back_servo,alpha_sym(2));
             success = 1;

    else
        success = 0;
    
    end
    
    
end

%compute the worspace (reachable_coordinates of endeffector)
function reachable_coordinates = getReachableCoordinates()
    
    global min_angle max_angle l_1 l_2 
    
    n = length(min_angle:1:max_angle)*2;
    
    reachable_coordinates = zeros(n,2);
    i = 1;
    for alpha = min_angle:1:max_angle
        for beta = min_angle:1:max_angle
            
             x_2 = l_1 * cosd(alpha) + l_2*cosd(alpha+beta-90);
             y_2 = l_1 * sind(alpha) + l_2*sind(alpha+beta-90);
            
             if abs(beta-90) < 45
                 if x_2 > 0
                     x_2 = floor(x_2);
                 end
                 if x_2 < 0
                     x_2 = ceil(x_2);
                 end

                  if y_2 > 0
                      y_2 = floor(y_2);
                  end
                  if y_2 < 0
                      continue
                 end
             else
                 if x_2 > 0
                     x_2 = ceil(x_2);
                 end
                 if x_2 < 0
                     x_2 = floor(x_2);
                 end

                  if y_2 > 0
                      y_2 = ceil(y_2);
                  end
                  if y_2 < 0
                      continue
                  end
             end
            
            reachable_coordinates(i,1) = x_2;
            reachable_coordinates(i,2) = y_2;
            i = i+1;
        end
    end
    
    reachable_coordinates = unique( reachable_coordinates(:,[1 2]), 'rows');

end

%plot workspace
function plotAvailableWorkspace()
    reachable_coordinates = getReachableCoordinates();

    f=figure(1);
    set(f,'WindowButtonDownFcn',@mytestcallback)
    scatter(reachable_coordinates(:,1), reachable_coordinates(:,2));

end

%plot robot with configuration
function plotRobotFromAngles(alpha,beta)

    global l_1 l_2
    
    alpha = deg2rad(alpha);
    beta = deg2rad(beta);
    
    alpha = real(alpha);
    beta = real(beta);
    
        %Position of first joint (first solution)
    x_1 = l_1*cos(alpha);
    y_1 = l_2*sin(alpha);
    
    x_2 = l_1*cos(alpha) + l_2*cos(alpha+beta-pi/2);
    y_2 = l_1*sin(alpha) + l_2*sin(alpha+beta-pi/2);
    
    

    %% Draw solution
    %Plot first segment
    plot([0 x_1], [0 y_1],'r')
    hold on
    %Plot second segment
    plot([x_1 x_2], [y_1 y_2],'b')
    %Plot Joints
    plot(0,0,'-o','MarkerSize',8,...
        'MarkerEdgeColor','black',...
        'MarkerFaceColor','black')
    plot(x_1,y_1,'-o','MarkerSize',8,...
        'MarkerEdgeColor','black',...
        'MarkerFaceColor','black')
    plot(x_2,y_2,'-o','MarkerSize',8,...
        'MarkerEdgeColor','green',...
        'MarkerFaceColor','green')

    %Plot settings
    axis equal
    grid on
    xlim([-2*l_2 2*l_2])
    ylim([-1 2*l_2+1])

    %Print joint angles
    caption = sprintf('alpha = %.1f, beta = %.1f        x = %.2f, y = %.2f', rad2deg(alpha), rad2deg(beta), x_2, y_2);
    title(caption, 'FontSize', 10);


end

function plotRobotFromPosition(x,y)

    global l_1 l_2 min_angle max_angle

    syms x_2 y_2 alpha_sym beta_sym

    e1= x_2 == l_1 * cos(alpha_sym) + l_2*cos(alpha_sym+beta_sym-pi/2);
    e2= y_2 == l_1 * sin(alpha_sym) + l_2*sin(alpha_sym+beta_sym-pi/2);
    
    [alpha_eqn,beta_eqn]=solve(e1,e2,alpha_sym,beta_sym);

    x_2 = x;
    y_2 = y;
    
    alpha_sym = rad2deg(double(subs(alpha_eqn)));
    beta_sym = rad2deg(double(subs(beta_eqn)));


    if ~(beta_sym(1) < min_angle || beta_sym(1) > max_angle || alpha_sym(1) < min_angle || alpha_sym(1) > max_angle)
        
            plotRobotFromAngles(alpha_sym(1),beta_sym(1))

    elseif ~(beta_sym(2) < min_angle || beta_sym(2) > max_angle || alpha_sym(2) < min_angle || alpha_sym(2) > max_angle)
        
            plotRobotFromAngles(alpha_sym(2),beta_sym(2))

    else
                disp('Endpoint not in available workspace');
    end

end

function mytestcallback(x, ysrc,~)

    
    pt = get(gca,'CurrentPoint');
    fprintf('Clicked: %d %d\n', round(pt(1,1)),round(pt(1,2)));
    if setEndPosition(round(pt(1,1)), round(pt(1,2)))
        
        clf
        plotRobotFromPosition(round(pt(1,1)),round(pt(1,2)));
        plotAvailableWorkspace
    end
    

    
    
end
