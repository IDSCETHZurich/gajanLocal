function Xkp1 = dynamicsD(dT,Xk,uk)
    % dT: discrete time interval
    % Xt: discrete state at t=t
    % uk: input along the road
    
    % Gravitational acceleration
    g = 9.81;

    vk = Xk(1);
    xk = Xk(2);
     
    % Acceleration along horizontal direction
    uxk = uk/sqrt(1+sin(xk)^2);
    
    % Force/acceleration (unit mass assuption)
    fk = -(vk^2 * sin(xk) * cos(xk) + g * sin(xk)) / (1 + sin(xk)^2)  ...
        + uxk / (1 + sin(xk)^2);
    
    % State update
    vkp1 = vk + dT*fk; 
    xkp1 = xk + dT*vk + 0.5*(dT^2)*fk;
    
    Xkp1 = [vkp1; xkp1];
end