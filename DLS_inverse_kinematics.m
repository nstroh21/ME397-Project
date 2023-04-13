function [T_final, jointVec, iterations, manip] = DLS_inverse_kinematics(robot, Tsd, guess)

    if guess == 0 
        q = robot.getJoints(); 
    else
        q = guess; 
    end
    iterations = 0;ymm = 0;
    % KEY IDEA: DLS uses optimization near the singularities to move away,
    % dampen motion of ivnerse kinematics

    while iterations < 50
        %% Obtain the updated Transform Matrics and Vb (error vector  
        Tsb = FK_space(robot,q) ;                               % check, space to end effec  
        Tbs = invertSE3(Tsb); Tbd = Tbs*Tsd;                    % check, body to space, Tbd should map the change
        [alpha, S] = se3Logarithm(Tbd); Vb = alpha .* S;        % Retrieve body twist
        wb = Vb(1:3); vb = Vb(4:6);
        %% Check Convergence %%        
        if norm(wb) < 1e-8                                
            if norm(vb) < 1e-8
                iterations = 1000;
                break
            end
        else
            %disp(norm(vb) + norm(wb));                           
        end

        %% Jacobian at Current Config  -- Check Manipulability to Swith into DLS Mode %%
        Js = J_space(robot, q); Jb = adjoint(Tbs)*Js;  % body jacobian
        integrate = ymm + J_ellipsoid_volume(Jb);
        ymm = J_ellipsoid_volume(Jb);
        iso = J_isotropy; condition = J_condition(Jb)
        % Arbitrary chose this value from testing KUkA volume metric, in general -- tune this
        
        if condition > 10      % min is 1/10th the maximum direction
            k = max(log(condition-10), 1) 
        end
        A = J*J' + (k^2).*eye(6); f = A/Vb;
        q = (q' + J'*f)';                   %disp("angles: "); disp(q*(180/pi))   % display q for testing
        iterations = iterations + 1;
        %disp("Body Twist"); disp(Vb); disp("Norms:"); disp(norm(wb)); disp(norm(vb));  % display for assignment
        
    end 
    % If Loop Breaks we have converged on final :
    jointVec = q;
    if iterations >= 500
        disp("timed out")
    end
    manip = integrate;
    T_final = Tbs;
end