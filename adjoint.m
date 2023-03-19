%%%%% Defintiion ==================================================%%%%%%
% Shorthand notation to get an adjoint representation of 4x4 matrix
% function expects matrix to be a member of SE3
function Adj = adjoint(T)
    %UNTITLED12 Summary of this function goes here
    p = skewSym(T(1:3,4));
    R = T(1:3,1:3); Z = zeros(3);
    Adj = cat(1,cat(2,R,Z), cat(2, p*R, R));
end