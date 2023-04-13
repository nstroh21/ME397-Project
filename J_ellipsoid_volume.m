function vol = J_ellpsoid_volume(J)
    if det(J*J') < 1e-10
        vol = 0;
    else 
        vol = sqrt(det(J*J'));
    end 
end