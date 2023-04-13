function [T_final, jointVec, iterations, manip, iso] = redundancy_resolution(robot,Tsd, guess)

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
    n = robot.numJoints;
    I = eye(n); iterations = []; i = 1; k = .05;
    q = guess; ymm = []; iso = [];

    while i < 500  
            %% Obtain the updated Transform Matrics and Vb (error vector  
            Tsb = FK_space(robot,q) ;                              % check, space to end effec  
            Tbs = invertSE3(Tsb); Tbd = Tbs*Tsd;                   % check, body to space, Tbd should map the change
            %disp('target'); disp(Tsd); disp("id?"); disp(Tbs*Tsb); % one check is to print out Tsd and compare to ensure it's actually working
            [alpha, S] = se3Logarithm(Tbd); Vb = alpha .* S;       % Retrieve body twist
            wb = Vb(1:3); vb = Vb(4:6);
            %% Check Convergence %%                                               
            if norm(wb) < 1e-5                                 
                if norm(vb) < 1e-5
                    break
                end
            else
                disp(norm(vb) + norm(wb));                           
            end
    
            %% Jacobian at Current Config - check %%
            Js = J_space(robot, q);  Jb = adjoint(Tbs)*Js;
            isot = J_isotropy(Jb);
            if isot ~= inf
                ymm = [ymm J_ellipsoid_volume(Jb)];
                iso = [iso isot];
                iterations = [iterations, i];
            end

    
            %Jb = Jb_quick(Tbs, Js);                           % Shortcut Method in absence of forward kinematics in body frame 
%             if length(Jb(1,:)) == length(Jb(:,1))              % Rank and Square Test
%                 disp('square'); disp(Jb);
%                 if det(Jb) > 1e-1
%                     disp('well posed');
%                     qerr = Jb/Vb;                              % Use actual inverse
%                     q = (q' + qerr)';
%                     iterations = iterations + 1;
%                     continue
%                 end
           % end  % per iteration use the update equation: q_dot + P*q_dot_zero 
            
            Jp = pinv(Jb); P = (I - Jp*Jb); q_dot_zero = manip_gradient(robot,q,Tbs); %disp("P: "); disp(P); disp("Gradient");disp(q_dot_zero);
            q = (q' + k*Jp*Vb' + P*q_dot_zero)'; %disp("angles: "); disp(q*(180/pi));    % update should be changed due to redundancy                  
            i = i+1;
            %disp("Body Twist"); disp(Vb); disp("Norms:"); disp(norm(wb)); disp(norm(vb)); disp(Vb); 
        end 
        % If Loop Breaks we have converged on final :
        jointVec = q; manip = ymm;
        T_final = FK_space(robot, jointVec);
        if i > 500
            disp('timed out')
        end 
end