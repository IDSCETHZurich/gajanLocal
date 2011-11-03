clear; clc;
global sys u;

%% Continous Time sub-system
l = 0.95;
g = 9.81;
Ac = [0 g/l 0 0;
   1 0 0 0;
   0 0 0 0;
   0 0 1 0];
Bc = [-1; 0; 1; 0];
Cc = [0 1 0 1];
Dc = 0;


Q = diag([100,200,1,2]);
R = 0.002;

[Kc,~,~] = lqr(Ac, Bc, Q, R, zeros(4,1));
display(Kc);


%% Descretize the sub-system

%Sampling time
dT = 1/50; % 50 Hz control

sys = ss(Ac,Bc,Cc,Dc);

sysd = c2d(sys,dT);

Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;
Dd = sysd.d;

[Kd,~,~] = dlqr(Ad, Bd, Q, R, zeros(4,1)); 
display(Kd);
% x0 = [0 0.05 0 0]';
% x = [0, x0'];
% for T=0:dT:4    
%    u = - K*x0;
%    [~,Y] = ode45(@pendDynamics,[0 dT],x0);
%    [i, ~] = size(Y);
%    x0 = Y(i,:)';
%    x = [x; T, x0'];
% end