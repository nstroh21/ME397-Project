function [T_final, q] = J_transpose_kinematics(robot, Tsd)
    %UNTITLED7 Summary of this function goes here
    %   Detailed explanation goes here
    
    % Tsd is the desired final location, still need to do all the twist vector
    % stuff

    q = robot.joints         % initial, won't actually move robot
    %q = q';
    J = J_space(robot, q);   % J_space
    K = 3*eye(6);              % requirement is positive definite (some better weighting? -- 3 arbitrary)
    err = 30;
    % Now I think we just iteratively move robot until error converges to 0
    iterations = 0;
    while err > 1e-4                                          % double precision
        Tsb = FK_space(robot,q) ;                             % confirmed on 2Chainz,
        Tbs = invertSE3(Tsb); Tbd = Tbs*Tsd;                  % confrimed on 2Chainz, SCARA,
        %disp("same?"); disp(Tsd);
        [alpha, S] = se3Logarithm(Tbd); Vb = alpha .* S;      % Retrieve body twist = Current Error level
        disp(err);
        Js = J_space(robot, q); Jb = adjoint(Tbs)*Js;       % J_body?
        if err < 1e-4
            break
        end
        if iterations > 100
            disp('timed out')
            break
        end
        if rank(Jb) ~= length(Jb(1,:))
            % if abs(det(J)) < 1e-11  % Only square
            disp(length(Jb(1,:))); disp(rank(Jb));
            disp("Matrix is not full rank, use another method")
            T_final = zeros(4,4);
            break;
        else  % robot update step                               
            % treat err = Vb;
            q_delta = (Jb'*K*Vb')';       % print for now to ensure not too large
            q = q + q_delta; 
            iterations = iterations + 1;
            err = norm(Vb);
        end   
    end 
    T_final = Tsb;
end