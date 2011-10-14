clear; clc;
%% Continous Time sub-system
l = 0.95;
g = 9.81;
Ac = [0 g/l 0 0;
   1 0 0 0;
   0 0 0 0;
   0 0 1 0];
Bc = [1; 0; 1; 0];
Cc = [0 1 0 1];
Dc = 0;


%% Descretize the sub-system

%Sampling time
Ts = 20/1000; % 20 ms

sys = ss(Ac,Bc,Cc,Dc);
sysd = c2d(sys,Ts);

Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;
Dd = sysd.d;

Q = diag([1,2,1,2]);
R = 1;

[K,~,~] = dlqr(Ad, Bd, Q, R, zeros(4,1));