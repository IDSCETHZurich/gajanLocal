% odometry 

% robot states w.r.t inertial frame
robot_x_i = 0.0;
robot_y_i = 0.0;
robot_theta_i = 0.0;

robotState = [];
robot_description_r = 0.258 / 2.0;

for i=1:size(data,1)
    dist = data(i,1)/1000.0;
    angle = data(i,2)/1000.0;
    
    robot_theta_i = robot_theta_i + angle/robot_description_r;
    robot_x_i = robot_x_i + dist * cos(robot_theta_i);
    robot_y_i = robot_y_i + dist * sin(robot_theta_i);	
    
    robotState = [robotState; robot_x_i, robot_y_i, robot_theta_i];
end

