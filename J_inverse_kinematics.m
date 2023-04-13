function [T_final, jointVec, iterations, manip, iso] = J_inverse_kinematics(robot, Tsd, guess)

    % VERIFIED : Pass a desired end configuration, and get the Robot's current Pose ... Run iterative commands to
    % bring robot to desired pose, output a vector of intermediate motions
    % Assume the input is in SE3 coordinates ... put in a check to confirm it

    %% Minimize Velocities in Contrained Opt For Demo? Should Produce smooth Robot Motion

    % initial guess (current pose -- maybe use randomization)
    %q = robot.getJoints(); 
    q = guess; ymm = []; iso = []; k = .05;   % Dampening Factor
    maxiter = 500; iterations = []; i = 0; ymm = [];
    iso_max = 100;
    
    % KEY IDEA: The Body Twist Vector is interpreted as the total error to
       %   be minimized by our algorithm  Vb = x_d - x_curr (in end effec  coordinates)
    
    while i < maxiter 
        %% Obtain the updated Transform Matrics and Vb (error vector  
        Tsb = FK_space(robot,q) ;                               % check, space to end effec  
        Tbs = invertSE3(Tsb); Tbd = Tbs*Tsd;                    % check, body to space, Tbd should map the change
        %disp('target'); disp(Tsd); disp("id?"); disp(Tbs*Tsb); % one check is to print out Tsd and compare to ensure it's actually working
        [alpha, S] = se3Logarithm(Tbd); Vb = alpha .* S;        % Retrieve body twist
        wb = Vb(1:3); vb = Vb(4:6);
        %% Check Convergence %%        
        if norm(wb) < 1e-5                              
            if norm(vb) < 1e-5
                disp("err: ") ; disp(norm(Vb));
                break
            end
        else
            %disp(norm(vb) + norm(wb));                           
        end

        %% Jacobian at Current Config - check %%
        Js = J_space(robot, q); Jb = adjoint(Tbs)*Js;
        
        % Collect Some metrics
        isot = J_isotropy(Jb);
        if isot ~= inf
            ymm = [ymm J_ellipsoid_volume(Jb)];
            iso = [iso isot];
            iterations = [iterations, i];
        end

        % Make the Update
        Jp = pinv(Jb); q = (q' + k*Jp*Vb')'; %disp("angles: "); disp(q*(180/pi))   % display q for testing
        i = i+1;
        %disp("Body Twist"); disp(Vb); disp("Norms:"); disp(norm(wb)); disp(norm(vb));  % display for assignment
    end 
    
    % If Loop Breaks we have converged on final :
    jointVec = q;
    T_final = FK_space(robot, jointVec); 
    
    if iterations >= maxiter
        disp(q);
        disp("timed out")
    end
    manip = ymm;
end



%% SCRATCH

%         if length(Jb(1,:)) == length(Jb(:,1))              % Rank and Square Test
%             %disp('square'); disp(Jb);
%             if det(Jb) > 1e-1
%                 disp('well posed');
%                 qerr = Jb/Vb;                              % Use actual inverse
%                 q = (q' + alpha*qerr)';
%                 iterations = iterations + 1;
%                 continue
%             end
%         end
