function tf = isUnit(Q)  % have to be very precise since there is rounding error
    if abs(norm(Q) - 1) < 1e-15  % floating point precision (32 bit)
        tf = true;
    else 
        tf = false;
    end
end