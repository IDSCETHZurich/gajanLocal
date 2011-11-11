function Xkp1 = dynamicsD(dT,Xk)
    global dir;
    g = 9.81;

    vk = Xk(1);
    xk = Xk(2);
     
    uk = dir/sqrt(1+sin(xk)^2);
    fk = -(vk^2 * sin(xk) * cos(xk) + g * sin(xk)) / (1 + sin(xk)^2)  ...
        + uk / (1 + sin(xk)^2);
    
    vkp1 = vk + dT*fk; 
    xkp1 = xk + dT*vk + 0.5*(dT^2)*fk;
    
    Xkp1 = [vkp1; xkp1];
end