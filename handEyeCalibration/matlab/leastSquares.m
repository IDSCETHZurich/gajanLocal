% LS solution

M = zeros(3,3);

for i = 1:10
    for j=(i+1):10
        rbi1 = eval(['rotRB',num2str(i)]);
        rbi2 = eval(['rotRB',num2str(j)]);
        Bi = inv(rbi2)*rbi1;

        cbi1 = eval(['rotCB',num2str(i)]);
        cbi2 = eval(['rotCB',num2str(j)]);
        Ai = cbi2*inv(cbi1);
        
        alpha = getLogTheta(Ai);
        beta = getLogTheta(Bi);
        M = M + beta*alpha';
    end
end

[V,D] = eig(M'*M);
Lambda = diag([sqrt(1/D(1,1)),sqrt(1/D(2,2)), sqrt(1/D(3,3))]);

x_est = V * Lambda * inv(V) * M';

display(x_est);

% for i = 1:9
%     rbi1 = eval(['rotRB',num2str(i)]);
%     rbi2 = eval(['rotRB',num2str(i+1)]);
%     Ai = inv(rbi2)*rbi1;
%     
%     cbi1 = eval(['rotCB',num2str(i)]);
%     cbi2 = eval(['rotCB',num2str(i+1)]);
%     Bi = cbi2*inv(cbi1);
%     
%     display( Ai * x_est -  x_est * Bi); 
% end