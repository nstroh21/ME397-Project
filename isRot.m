function isRot = isRot(R)
    r0 = R(:,1); r1 = R(:,2); r2 = R(:,3);
    i = [1,0,0]'; j = [0,1,0]'; k = [0,0,1]';
    check = dot(r0,r1) + dot(r0, r2) + dot(r1,r2); 
    if (abs(det(R)-1) > 1e-13) isRot = false; disp('determinant'); disp(det(R));
    elseif (check > 1e-13) isRot = false; disp('orthogonal'); disp(R); disp(check); % numerical error possible, careful
    else isRot = true;
    end
end