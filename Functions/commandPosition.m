function commandPosition(back_servo,front_servo,angle_back,angle_front)

    back_servo.writePosition(angle_back);
    front_servo.writePosition(90-angle_front); % <- the 90 degrees is due to our choice of coordinates

end

