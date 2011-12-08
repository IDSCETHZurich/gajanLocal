function X_dot = dynamics(~,X)
    global g
    x_dot = X(1);
    x = X(2);
    
    dir=2;
    if (x_dot<0)
        dir = -2;
    end

    u = dir/sqrt(1+sin(x)^2);
    X_dot = [ -(x_dot^2 * sin(x) * cos(x) + g * sin(x)) / (1 + sin(x)^2)  ...
        + u / (1 + sin(x)^2);
        x_dot];
end