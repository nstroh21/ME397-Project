%%% Checks to see if a robot is currently in a singluarity configuration.
%
% ARGUMENTS: robot: robot object to check for singularity
%            angles: vector of joint angles for the robot. To use the
%            angles defined in the robot class, pass in [].
% OUTPUTS:   s: boolean value indicating whether the robot is in singularity

function s = is_singularity(robot, angles)
    if isequal(angles, [])
        angles = robot.joints;
    end

    J = J_space(robot, angles);

    if det(J) == 0 || rank(J) ~= robot.numJoints
        s = true;
    else
        s = false;
    end