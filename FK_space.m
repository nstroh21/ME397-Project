%%%% DEFINITION of FK_space ==============================================================%%%%%%%%
% "robot-agnostic." It takes in a robot object, reads the robot class for whatever screw axes defined
% and claculates product of exponentials up to the "end" frame. It assumes the final frame is based on the # of joint 
% coordinates are passed, therefore if a joint is not intended to move, pass "0" for that joint

function T_space  = FK_space(robot, joints)
    steps = length(joints)
    j = 1;
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
        % STEP 1: Using (omega, q) derive screw axis (w, -w x q), this method is
        % actually in the robot class (screws set at the time of definition)
        
        % STEP 2: Use ChaslesMozzi to get T matrix and take product
        % rotating joint = 0 , prismatic joint = 1
        th = joints(j); S = screws(j,:); jointType = robot.jointTypes(j);
        if jointType == 0
            T = chaslesMozzi(S, th, 0); % pure rotation
        else
            T = chaslesMozzi(S, th, 1); % pure translation
        end 
        if j == 1
            T_space = T;
        end
        if j == steps
                T_space = T_space*T*M;  % Full Forward Kinematics to the End Effector, assumes that all joints are defined or 0
        else
            T_space = T_space*T;
        end
        j = j+1;
    end
end