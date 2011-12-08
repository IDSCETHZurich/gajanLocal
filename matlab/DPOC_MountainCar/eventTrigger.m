function [value,isterminal,direction] = eventTrigger(~,X)
    value = abs(X(2))-pi;
    isterminal = 1;
    direction = 0;
end