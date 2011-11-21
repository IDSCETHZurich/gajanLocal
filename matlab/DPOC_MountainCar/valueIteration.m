clc; clear;
global g v_max v_min x_max x_min dSteps dT v_step x_step 
g = 9.81;

%%Discretization
v_max = 8;
v_min = -v_max;
x_max = pi; 
x_min = -x_max; 

dSteps = 50;
dT = 0.1;

v_step = (v_max - v_min)/dSteps; 
x_step = (x_max - x_min)/dSteps; 

J = zeros(dSteps * dSteps,1);

iterationSteps = 50;
for k=1:iterationSteps
    for i =1:dSteps
        for j = 1:dSteps 
            %Border Js are zeros. No need to iterate
            
            currentX = [v_min + (i-1)*v_step + 0.5*v_step;
                x_min + (j-1)*x_step + 0.5*x_step];
            
            %calculate the next states
            nextX_pI = dynamicsD(dT,currentX,+2);
            nextX_nI = dynamicsD(dT,currentX,-2);
         
            %finding the corresponding indices
            JvipI = floor((nextX_pI(1)-v_min)/v_step)+1;
            JxipI = floor((nextX_pI(2)-x_min)/x_step)+1;

            JvinI = floor((nextX_nI(1)-v_min)/v_step)+1;
            JxinI = floor((nextX_nI(2)-x_min)/x_step)+1;
            
            %Boundary checks
            updateJ = 1;
            if (JvipI<1) updateJ=updateJ*0; end 
            if (JvinI<1) updateJ=updateJ*0; end            
            if (JxipI<1) updateJ=updateJ*0; end 
            if (JxinI<1) updateJ=updateJ*0; end
            
            if (JvipI>dSteps) updateJ=updateJ*0; end 
            if (JvinI>dSteps) updateJ=updateJ*0; end            
            if (JxipI>dSteps) updateJ=updateJ*0; end 
            if (JxinI>dSteps) updateJ=updateJ*0; end
            
            %Update value
            if updateJ==1
                J(dSteps*(i-1) + j) = min([gc(currentX,+2) ...
                    + J(dSteps*(JvipI-1)+JxipI); % u = +2
                    gc(currentX,-2) ...
                    + J(dSteps*(JvinI-1)+JxinI)]); % u = -2
            end
        end
    end
    display(['iteration: ', num2str(k)])
end

surf(reshape(J,dSteps,dSteps)');




