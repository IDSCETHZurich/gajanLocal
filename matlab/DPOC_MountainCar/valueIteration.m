clc; clear;
global g
g = 9.81;


%%Discretization
v_max = 8;
v_min = -v_max;
x_max = pi+0.1; 
x_min = -x_max; 

dSteps = 50;

v_step = (v_max - v_min)/dSteps; 
x_step = (x_max - x_min)/dSteps; 

J = zeros(dSteps+1, dSteps+1);

iterationSteps = 200;
for k=1:iterationSteps
    for i =2:(dSteps)
        for j = 2:dSteps %Border Js are zeros. No need to iterate
            currentX = [v_min + i*v_step; x_min + j*x_step];
            
            nextX_pI = f(currentX,+2);
            nextX_nI = f(currentX,-2);
                
            JvipI = floor((nextX_pI(1)-v_min)/v_step)+1;
            JxipI = floor((nextX_pI(2)-x_min)/x_step)+1;

            JvinI = floor((nextX_nI(1)-v_min)/v_step)+1;
            JxinI = floor((nextX_nI(2)-x_min)/x_step)+1;
            
            %Boundary Checks
            if (JvipI<1) JvipI = 1; end 
            if (JvinI<1) JvinI = 1; end            
            if (JxipI<1) JxipI = 1; end 
            if (JxinI<1) JxinI = 1; end
            
            if (JvipI>dSteps) JvipI = dSteps + 1; end 
            if (JvinI>dSteps) JvinI = dSteps + 1; end            
            if (JxipI>dSteps) JxipI = dSteps + 1; end 
            if (JxinI>dSteps) JxinI = dSteps + 1; end
            
            J(i,j) = min([gc(currentX,+2) ...
                + J(JvipI, JxipI); % u = +2
                gc(currentX,-2) ...
                + J(JvinI, JxinI)]); % u = -2
        end
    end
    surf(J);
    display(['iteration: ', num2str(k)])
end







