function J_body = Jb_quick(Tbs, J_space)
% Given Forward Kinematics and jacobian have already been calculated in
% space frame, a quick computation to retrieve J_body
A = adjoint(Tbs);
J_body = A*J_space;
end