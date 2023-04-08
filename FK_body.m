%%% Calculates the body form of the specified robot's forward kinematics.
%%% Robot must be defined in the body form.
%
% ARGUMENTS: robot: robot object to calculate FK for
%            joints: 1D vector containing all joint angles in order of
%            definition
% OUTPUTS:   T_body: configuration of the spatial frame wrt body frame.

function T_body = FK_body(robot, joints)
    steps = length(joints);
    j = 1;

    if isequal(robot.frame, 'space')
        disp('Robot must be defined in the body frame.')
        return;
    end
    try
        screws = robot.screws;
        numJoints = robot.numJoints;
        M = robot.M;
    catch
        disp("Define 'myrobot' object before calling this function")
    end

    if steps > numJoints
        disp("Error: More angles were passed than joints defined for robot")
        disp(numJoints)
        disp(steps)
    end

    while j <= steps
        th = joints(j); 
        S = screws(j,:); 
        jointType = robot.jointTypes(j);
        
        if jointType == 0
            T = chaslesMozzi(S, th, 0); % pure rotation
        else
            T = chaslesMozzi(S, th, 1); % pure translation
        end 

        if j == 1
            T_body = M*T;
        else
            T_body = T_body*T;
        end
        j = j + 1;
    end
end

