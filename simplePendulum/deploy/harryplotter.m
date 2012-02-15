clear; clc;
load('stateLog.txt')
[t,~] = size(stateLog);

figure
plot(stateLog(1:t,[1:4 9 11]))
grid on
axis([0 t -2 2])
title('X-Axis')
legend('xp dot', 'xp', 'xr dot', 'xr', 'ux', 'xr cmd')

figure
plot(stateLog(1:t,[5:8 10 12]))
grid on
axis([0 t -2 2])
title('Y-Axis')
legend('yp dot', 'yp', 'yr dor', 'yr', 'uy', 'yr cmd')
