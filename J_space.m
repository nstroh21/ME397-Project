%%%% DEFINITION of Jacobian_space ==============================================================%%%%%%%%
% "Robot-Agnostic." It takes in a robot object, reads the robot class for whatever screw axes defined
% takes in a robot object and gets the Jacobian for the current
% configuration
%% Next Step would be to have the robot object store the configuration as a property
function J = J_space(robot, angles)
    steps = length(angles); j = 1;
    effec = FK_space(robot, angles); % run forward kinematics to set all the screw axes 
    try
        numJoints = robot.numJoints
        screws = robot.screws
        M = robot.M
    catch
        disp("Err: Must input the same number of angles as joints in your robot")
    end
    if length(angles) ~= numJoints
        disp("Err: Must input the same number of angles as joints in your robot")
    end
    J = zeros(6,numJoints);
    while j <= steps
        S = screws(j,:); %row vector
        jointType = robot.jointTypes(j);
        if j == 1
            T = eye(4);
            J(:,j) = S;
            j = j + 1;
        else
            % T = FK_space(robot, angles(1:j-1))  % this way is buggy too
            % -- just leave out, each screw axis needs to have it's own home position defined
            th = angles(j-1)
            if jointType == 0
                E = chaslesMozzi(screws(j-1,:), th, 0); % pure rotation
            else
                E = chaslesMozzi(screws(j-1,:), th, 1); % pure translation
            end
            disp(j); 
            screws(j-1,:)
            angles(j-1)
            E = chaslesMozzi(screws(j-1,:), angles(j-1), 0)  
            Tj = T*E;
            disp("Test: ")
            disp(Tj*M)
            disp(S')
            J(:,j) = adjoint(Tj)*S';
            j = j + 1;
            T = Tj;
        end
    end
end