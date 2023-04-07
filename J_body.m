function Jb = J_body(robot, angles)
    i = length(angles);

    % Sanity checks
    if isequal(robot.frame, 'space')
        disp('Robot must be defined in the body frame.')
        return;
    end
    try
        numJoints = robot.numJoints;
        screws = robot.screws;
    catch
        disp("Err: Must input the same number of angles as joints in your robot");
    end
    if length(angles) ~= numJoints
        disp("Err: Must input the same number of angles as joints in your robot");
    end

    Jb = zeros(6, numJoints);
    while i > 0
        S = screws(i, :);
        if i == length(angles)
            T = eye(4);
            Jb(:, i) = S;
            i = i - 1;
        else
            E = chaslesMozzi(-screws(i+1, :), angles(i+1), 0);
            Tj = T*E;
            Jb(:, i) = adjoint(Tj)*S';
            i = i - 1;
            T = Tj;
        end
    end
end



        