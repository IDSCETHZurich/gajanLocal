function pij  = transMatrix( u )
    global v_min x_min dSteps dT v_step x_step u_max
    
    % u contains elements 1=umax and 2=-umax
    
    pij = zeros(dSteps^2, dSteps^2);
    diele = 0;
    
    for i =1:dSteps
        for j = 1:dSteps 
            
            currentX = [v_min + (i-1)*v_step + 0.5*v_step;
                x_min + (j-1)*x_step + 0.5*x_step];
            
            
            currentUele = u( dSteps*(i-1)+j );
            if currentUele == 1
                currentU = u_max;
            elseif currentUele == 2
                currentU = -u_max;
            else
                display('error:u contains unexpected element');
            end
            
            %calculate the next states
            nextX = dynamicsD(dT,currentX,currentU);
         
            %finding the corresponding indices
            Jvi = floor((nextX(1)-v_min)/v_step)+1;
            Jxi = floor((nextX(2)-x_min)/x_step)+1;

            %Boundary checks
            if (Jvi<1) Jvi = 1; end            
            if (Jxi<1) Jxi = 1; end 
            if (Jvi>dSteps) Jvi = dSteps; end 
            if (Jxi>dSteps) Jxi = dSteps; end 
            
            if dSteps*(i-1) + j == dSteps*(Jvi-1)+Jxi
                if abs(nextX(2)) < pi
                    display('diagonal element found')
                    display(diele);
                    display(currentX);
                    display(currentU);
                    display(nextX);
                    diele = diele +1;
                end
            end
            pij(dSteps*(i-1) + j, dSteps*(Jvi-1)+Jxi) = 1;
                
            

        end
    end
end

