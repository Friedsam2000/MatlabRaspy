
%% connection
% clear 
% r = raspi

%% setup

configurePin(r,22,'PWM')
f = 50;
writePWMFrequency(r, 22, f)

% working range : 6° to 184°

t = 0;
dt = 2/f;

period_time = 5; %s
while 1
    t = t+dt;
    
    angle = 90 + 80*sin(((2*pi)/period_time)*t);
    disp(angle);
    set_angle(angle,r);
    
    pause(dt);
end


function [] = set_angle(angle,r)
    
    offset = -6;
    angle = angle + offset;
    angle = min(max(angle,0),180);
    
    min_duty = 0.02; % 0
    max_duty = 0.12; % 180
    duty = min_duty + (angle/180)*(max_duty-min_duty);
    writePWMDutyCycle(r, 22, duty);

end