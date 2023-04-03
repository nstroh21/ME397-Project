function output1 = redundancy_resolution(inputArg1,inputArg2)
% Run the numerical kinematics loop, but we want to update q differently
% per iteration ... use the update equation:
%  q_dot + P*q_dot_zero 
% where P is the projection matrix (I - J_dagJ) and q_dot_zero is going to
% be defined based on w(q). Ie it is the solution of an unconstrained
% optimization that maximizes some secondary cost function. The reason it
% is uncosntrained is because Projection matrix encodes constraint in the
% null space of the robot Jacobian

% Question : We have the form J_dag * v_e is the solution of minimizing
% velocities, however is this not identical to Secant Method from inverse
% kinematics ?

% J_dag  + P*q_0  -- equation


while iterations < 15   
        %% Obtain the updated Transform Matrics and Vb (error vector  
        Tsb = FK_space(robot,q)                                % check, space to end effec  
        Tbs = invertSE3(Tsb); Tbd = Tbs*Tsd;                   % check, body to space, Tbd should map the change
        %disp('target'); disp(Tsd); disp("id?"); disp(Tbs*Tsb); % one check is to print out Tsd and compare to ensure it's actually working
        [alpha, S] = se3Logarithm(Tbd); Vb = alpha .* S;       % Retrieve body twist
        wb = Vb(1:3); vb = Vb(4:6);
        %% Check Convergence %%                                               
        if norm(wb) < 1*10e-7                                 
            if norm(vb) < 1*10e-7
                iterations = 1000;
                break
            end
        else
            disp(norm(vb) + norm(wb));                           
        end

        %% Jacobian at Current Config - check %%
        Js = J_space(robot, q); Jb = adjoint(Tbs)*Js; 
        %Jb = Jb_quick(Tbs, Js);                           % Shortcut Method in absence of forward kinematics in body frame 
        if length(Jb(1,:)) == length(Jb(:,1))              % Rank and Square Test
            disp('square'); disp(Jb);
            if det(Jb) > 1e-1
                disp('well posed');
                qerr = Jb/Vb;                              % Use actual inverse
                q = (q' + qerr)';
                iterations = iterations + 1;
                continue
            end
        end
        disp('use pseudo')
        Jp = pinv(Jb);
        q = (q' + Jp*Vb')'
        iterations = iterations + 1
    end 
    % If Loop Breaks we have converged on final :
    jointVec = q;
    T_final = FK_space(robot, jointVec);
end





end