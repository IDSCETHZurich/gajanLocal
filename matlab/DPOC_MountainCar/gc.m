function cost = gc(x,~)
    if x(2) >= pi || x(2) <= -pi
        cost = 0;
    else 
        cost = 1;
    end
end