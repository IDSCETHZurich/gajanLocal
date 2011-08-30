%generateData.m 
numberOfData = 10; 

Rx =[   
    0.0306    0.9995    0.0106;
   -0.9989    0.0310   -0.0346;
   -0.0349   -0.0095    0.9993];

rx = [
   -0.0264;
    0.1703;
   -0.0200];

tool_camera_T = [Rx rx; 0 0 0 1]
base_CB_T = [eye(3) ones(3,1); 0 0 0 1]

for i=1:numberOfData
    base_tool_T = [qr(randn(3)), rand(3,1); 0 0 0 1];
    camera_CB_T = inv(tool_camera_T)*inv(base_tool_T)*base_CB_T;
    %display(camera_CB_T);
    
end
