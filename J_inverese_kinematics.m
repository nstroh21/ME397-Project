function [T_final, jointVec] = J_inverese_kinematics(robot, Tsd)

    % DEBUGGING NEEDED  : Pass a desired end configuration, and get the Robot's current Pose ... Run iterative commands to
    % bring robot to desired pose, output a vector of intermediate motions
    
    % Assume the input is in SE3 coordinates ... put in a check to confirm it
    
    % initial guess (current pose -- maybe use randomization)
    q = robot.getJoints(); 
    disp("check initial");
    Tsb = FK_space(robot,q)
    Tbs = invertSE3(Tsb) 
    Js = J_space(robot, q);
    Jb = adjoint(Tbs)*Js
    
    % Find the body transform from current to goal in space coords
    iterations = 0
    guess = [0, pi/6];
    q = guess
    % KEY IDEA: The Body Twist Vector is interpreted as the total error to
       %   be minimized by our algorithm  Vb = x_d - x_curr (in end effec  coordinates)
    
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
                qerr = Jb/Vb;         % Use actual inverse
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