function [T_final, q] = J_transpose_kinematics(robot, Tsd, guess)

    % Trying to implement Jacobian Transpose method. K positive
    % semi-definite matrix or alpha both erffectively do the same thing
    % (the provide weighting to the equation J_transpose*error

    q = guess;                   % initial, won't actually move robot              
    err = 30; alpha = 1; K = 100*eye(6);      %requirement is positive definite (some better weighting? -- 3 arbitrary)
    % Now I think we just iteratively move robot until error converges to 0
    iterations = 0;
    while err > 1e-4                                          % double precision
        Tsb = FK_space(robot,q) ;                             % confirmed on 2Chainz,
        Tbs = invertSE3(Tsb); Tbd = Tbs*Tsd;                  % confrimed on 2Chainz, SCARA,
        [alpha, S] = se3Logarithm(Tbd); Vb = alpha .* S;      % Retrieve body twist = Current Error level
        Js = J_space(robot, q); Jb = adjoint(Tbs)*Js;         % J_body
        if err < 1e-4
            break
        end
        if iterations > 2000
            disp('timed out')
            break
        end
        if rank(Jb) ~= length(Jb(1,:))
            % if abs(det(J)) < 1e-11  % Only square
            disp(length(Jb(1,:))); disp(rank(Jb));
            disp("Matrix is not full rank, use another method");
            T_final = zeros(4,4);
            break;
        else  % robot update step                               
            % treat err = Vb;  % Using alpha scalar method from paper
            num = Vb *(Jb*Jb'*Vb'); 
            denom = (Jb*Jb'*Vb')'*(Jb*Jb'*Vb');
            alpha = num/denom;  q_delta = alpha.*(Jb'*Vb')';
            q = q + q_delta; iterations = iterations + 1; err = norm(Vb); 
%             disp("angles: "); disp(q*(180/pi)) 
        end   
    end
    disp('err: '); disp(norm(Vb));
    disp("iters");disp(iterations);
    T_final = Tsb;
end