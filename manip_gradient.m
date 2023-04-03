function grad = manip_gradient(myrobot, q)
% takes in the joint vector q and calculates numerical partial derivatives
% returns gradient vector specifically of manipulability measure f =
% sqrt(det(J*J')) 

% I believe we could skip sqrt func but oh weel expensive part is jacobian
% use centered finite differences

% (f(x(1 + √ε)) - f(x(1 - √ε))) / 2x√ε -- implement this finite difference at each component


eps = 1e-10;                     % epsilon, close-sih to "ulp" which is 1 unit from machine precision
grad = zeros(length(q),1);
delta = zeros(length(q),1);

for i = 1:length(q)
    if q(i) == 0
        h = eps
    else
        h = q(i)*sqrt(eps);         % formula from stack exchange which is supposed to be stable even when close to 0
    end
    % positive half
    delta(i) = h;               % set a delta at each component iteratively
    q_step = q + delta; 
    J = J_space(myrobot,q_step)
    J*J'
    f_plus = sqrt(det(J*J'))
    
    % negative half
    delta(i) = -h; 
    q_step = q + delta; 
    J = J_space(myrobot,q_step);     
    f_minus = sqrt(det(J*J'))
    
    disp(h)
    grad(i) = (f_plus - f_minus)/(2*h);
    disp(grad(i))
    % reset deltas after calculation:
    delta(i) = 0.0;

end