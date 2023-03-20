function R = quat2Rot(Q)
%================ Quaternions to Rotation Matrices =============================%
% this way is hard-coding, and we are choosing 0 to pi
  if isa(Q, 'quaternion') % we can accept either format of quaternion
       [q0,q1,q2,q3] = parts(Q); 
       Q = [q0,q1,q2,q3];
  else 
      q0 = Q(1); q1 = Q(2); q2 = Q(3); q3 = Q(4);  
  end
  if (isUnit(Q) == false) % it must be a unit quaternion or else return err message and identity
    disp("Error: input must be a unit quaternion. Returning Identity")
    R = eye(3); 
  else
    R = [[(q0^2 + q1^2 - q2^2 - q3^2), 2*(q1*q2 - q0*q3)           ,          2*(q0*q2 + q1*q3) ]
         [2*(q0*q3 + q1*q2)          , (q0^2 - q1^2 + q2^2 - q3^2) ,          2*(q2*q3 - q0*q1) ]
         [2*(q1*q3 - q0*q2)          , 2*(q0*q1 + q2*q3)           , (q0^2 - q1^2 - q2^2 + q3^2)] ];
  end 
end