function [outputArg1,outputArg2] = ellipsoid_plot_linear(J)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
J_velo = J(4:6,:);
disp("linear manipulation magnitudes")
[vecs, eigs] = eig(J_velo*J_velo');
disp(eigs)
disp(vecs)
condition = max(eigs)/min(eigs);
isotropy = sqrt(condition);

outputArg1 = 1;
outputArg2 = 1;
end