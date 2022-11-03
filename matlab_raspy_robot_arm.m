clear all
clc

syms x_2 y_2 alpha beta l

%% Solve analytically

e1= x_2 == l * cos(alpha) + l*cos(alpha+beta);
e2= y_2 == l * sin(alpha) + l*sin(alpha+beta);

[alpha_eqn,beta_eqn]=solve(e1,e2,alpha,beta);

t = 0;
dt = 0.01;
while 1
    
    %sim time
    t = t + dt;
    
    %% Plug in values
    %length of one segment
    l = 1;

    %Desired position of Endeffector
    x_2 = l*sin(5*t);
    y_2 = l*cos(10*t);

    %% Solve numerically

    %Numerically calculate alpha and beta
    alpha = subs(alpha_eqn);
    beta = subs(beta_eqn);

    %Position of first joint (first solution)
    x_1 = l*cos(alpha(1));
    y_1 = l*sin(alpha(1));


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
    xlim([-2*l 2*l])
    ylim([-2*l 2*l])



    %Print joint angles
    caption = sprintf('alpha = %d, beta = %d        x = %.2f, y = %.2f', rad2deg(alpha(1)), rad2deg(beta(1)), x_2, y_2);
    title(caption, 'FontSize', 10);
    
    %Animation
    pause(dt)
    hold off
end


