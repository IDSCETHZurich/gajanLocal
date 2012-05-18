% odometry 

% robot states w.r.t inertial frame
robot_x_i = 0.0;
robot_y_i = 0.0;
robot_theta_i = 0.0;

robotState = [];
robot_description_r = 0.25 / 2.0;

for i=1:size(data,1)
    d_dist = data(i,1)/1000.0;
    d_angle = data(i,2)/1000.0;
    
    robot_theta_i = robot_theta_i + 2*d_angle/robot_description_r;
    robot_x_i = robot_x_i + d_dist * cos(robot_theta_i);
    robot_y_i = robot_y_i + d_dist * sin(robot_theta_i);	
    
    range = (1.494*(10^9)/data(i,3) + 58.47)/1000.0;
    range = range ;
    obstacle_x = robot_x_i - range*cos(robot_theta_i);
    obstacle_y = robot_y_i - range*sin(robot_theta_i);
    
    robotState = [robotState; 
        robot_x_i, robot_y_i, robot_theta_i, obstacle_x, obstacle_y, range];
end

plot(robotState(:,4), robotState(:,5), 'x',robotState(:,1), robotState(:,2), 'r')
axis equal
