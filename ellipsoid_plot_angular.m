function [outputArg1,outputArg2] = ellipsoid_plot_angular(J)
%UNTITLED Summary of this function goes here
%   Given either form of Jacobian, plots manipulability ellipsoids
% at a given configuration of the robot
J_omega = J(1:3,:)
disp("angular manipulation magnitudes")
disp(eig(J_omega*J_omega'));

outputArg1 = 1;
outputArg2 = 1;
end