clc; clear;
global dir g
g = 9.81;

%%continuous time 
options = odeset('Events',@eventTrigger);
initialState = [0; 0];
[T,X,TE] = ode45(@dynamics, [0 100], initialState, options);

%%Discrete time 
dT = 0.01; 
Td = [0]; 
Xd = initialState'; 
x_old = initialState;

%%Discretization
x_max = pi; 
x_min = -x_max; 
v_max = 8;
v_min = -v_max;

dSteps = 5000;

x_step = (x_max - x_min)/dSteps; 
v_step = (v_max - v_min)/dSteps; 

%% Loop
for i=0+dT:dT:7.5
    Td = [Td; i];
    % calculate input
    dir=2;
    if (x_old(1)<0)
        dir = -2;
    end
    x_new = dynamicsD(dT,x_old);
    
    %discritization
     x_new(1) = floor(x_new(1)/v_step)*v_step;
     x_new(2) = floor(x_new(2)/x_step)*x_step;

    Xd = [Xd; x_new'];
    x_old = x_new;
    
end


% Value Iteration


