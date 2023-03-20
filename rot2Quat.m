function Q = rot2Quat(R)
%=================== Rotation Matrix to Quaternion Representation (Unique)=======%
% Hard-code formulas given in class
    Q = [1,0,0,0]; R  % display only
    if isRot(R) == false
            disp('Input to function was not a valid rotation matrix');
    else
        Q(1) = sqrt(R(1,1) + R(2,2) + R(3,3) + 1)/2;
        Q(2) = sign(R(3,2) - R(2,3))*sqrt(R(1,1) - R(2,2) - R(3,3) + 1)/2;
        Q(3) = sign(R(1,3) - R(3,1))*sqrt(R(2,2) - R(3,3) - R(1,1) + 1)/2;
        Q(4) = sign(R(2,1) - R(1,2))*sqrt(R(3,3) - R(1,1) - R(2,2) + 1)/2;
    end
end

% Another way is to convert to axis-angle first and then to quaternion
function Q = rot2Quat2(R)
    theta, w = Rot(R);
    Q = zeros(4); Q(1) = cos(theta)/2 ; Q(2:4) = sin(theta/2).*w
end