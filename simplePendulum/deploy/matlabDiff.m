pendPos = stateLog(:,2);
plot(pendPos)

pendVel = [];
for i=2:size(pendPos,1)
    pendVel = [pendVel;(pendPos(i)-pendPos(i-1))/0.0114];
end

v =smooth(pendVel);

hold on
plot(pendVel,'r')