function T_inv = invertSE3(T)
    R = T(1:3, 1:3); p = T(1:3 , 4);
    pinv = (-R')*p;
    T_inv = cat(1, [R' pinv], [0 0 0 1]);
end