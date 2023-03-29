function J_body = spaceToBody(J_space)
    % Verifies the Space Jacobian and Body Jacobian Functions agree
    %  by comparing the output of these functions to the Adjoint transofrmation
    %  formula which, given one jacobian, automatically gives us the other
    % This transformation takes the space jacbian and returns body form
    J_body = adjoint(M)*J_space;
end