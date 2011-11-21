clc; clear;
global g v_max v_min x_max x_min dSteps dT v_step x_step u_max
u_max = +2;

%%Discretization
v_max = 7;
v_min = -v_max;
x_max = pi; 
x_min = -x_max; 

dSteps = 50;
dT = 0.5;

v_step = (v_max - v_min)/dSteps; 
x_step = (x_max - x_min)/dSteps; 

J = zeros(dSteps^2,1);
u = ones(dSteps^2,1);
I = eye(dSteps^2);
g = zeros(dSteps^2,1);

iterationSteps = 1;
for k=1:iterationSteps
    %% Policy Evaluation
    for i =1:dSteps
        for j = 1:dSteps 
            currentX = [v_min + (i-1)*v_step + 0.5*v_step;
                x_min + (j-1)*x_step + 0.5*x_step];
            g(dSteps*(i-1) + j)=gc(currentX,0);
        end
    end
    pij = transMatrix(u);
    J = (I-pij)\g;
    
    %% Policy Improvement
    for i =1:dSteps
        for j = 1:dSteps 
            %Border Js are zeros. No need to iterate
            
            currentX = [v_min + (i-1)*v_step + 0.5*v_step;
                x_min + (j-1)*x_step + 0.5*x_step];
            
            %calculate the next states
            nextX_pI = dynamicsD(dT,currentX,+u_max);
            nextX_nI = dynamicsD(dT,currentX,-u_max);
         
            %finding the corresponding indices
            JvipI = floor((nextX_pI(1)-v_min)/v_step)+1;
            JxipI = floor((nextX_pI(2)-x_min)/x_step)+1;

            JvinI = floor((nextX_nI(1)-v_min)/v_step)+1;
            JxinI = floor((nextX_nI(2)-x_min)/x_step)+1;
            
            %Boundary checks
            if (JvipI<1) JvipI = 1; end 
            if (JvinI<1) JvinI = 1; end            
            if (JxipI<1) JxipI = 1; end 
            if (JxinI<1) JxinI = 1; end
            
            if (JvipI>dSteps) JvipI = dSteps; end 
            if (JvinI>dSteps) JvinI = dSteps; end            
            if (JxipI>dSteps) JxipI = dSteps; end 
            if (JxinI>dSteps) JxinI = dSteps; end
            
            %Update value
            [~,u(dSteps*(i-1) + j)] = min([gc(currentX,+2) ...
                + J(dSteps*(JvipI-1)+JxipI); % u = +u_max
                gc(currentX,-2) ...
                + J(dSteps*(JvinI-1)+JxinI)]); % u = -u_max
        end
    end
    
    display(['iteration: ', num2str(k)])
end


surf(reshape(J,dSteps,dSteps)');




