classdef myrobot    %  < hw1 could "inherit" hw1 functions but doesn't make a lot of sense
                    % more likely it should be a handle ... < handle
    
    %   Instead I include the functions I need from HW1 in robot definition

    properties
        frame = "space";    %
        M;                  %Home Configuration of the end effector 
        numJoints;          %scalar to track number of joints in the system
        jointTypes;         %Binary List for each joint type, either 0 or 1
        homeVectors;        %set of 6D vectors describing position and orientation of joints
        screws;             %Set of 6D vectors describing screw axes derived from homeVectors
        joints;
        pose;
        %partner
    end

    methods
        function obj = myrobot(frame,M,numJoints)   % home config and table of screws, Constructor
            obj.frame = frame;
            obj.M = M;
            obj.pose = M;
            obj.numJoints = numJoints;
            obj.jointTypes = zeros(numJoints);  % 0 for rotation and 1 for prismatic
            obj.screws = zeros(numJoints,6);
            obj.joints = zeros(1,numJoints);
            formatSpec = 'Robot Created With %d Joints in %s Frame'; 
            fprintf(formatSpec,numJoints,frame);
        end

        function obj = addJoint(obj,jointNumber, type, V, VType)
            %Overwrties joint configuration if exists, V = homeVector position
             obj.jointTypes(jointNumber) = type;
            if VType == "pose"
                obj.homeVectors(jointNumber,:) = V;
                disp("Joint Added, Pose:");
                disp(obj.homeVectors);
                v = -1.*skewSym(V(1:3))*V(4:6)';
                screw = [V(1:3),v'];
                obj.screws(jointNumber,:) = screw;
                disp("Screws Updated: ");
                disp(obj.screws);
            %setScrew(obj, vector, jointNumber); % code below sets the screw
            else
                obj.screws(jointNumber,:) = V;
                disp("Joint Added, Pose Unknown, Screws Updated: ");
                disp(obj.screws);
            end
        end
        function HOME(obj)
            disp("Wherever your robot was, it is reset to home position");
            obj.joints = zeros(1,numJoints);
            % reset screw axes
            for i = 1:obj.numJoints
                obj.addJoint(i, obj.jointTypes(i), homeVectors(i), "pose");
            end
        end
        function MOVE(obj, thetas)
            % set robot to position, uniquely defined by joints
            % The Jacobian will not automatically be known ... (add this
            % feature ?) 
            obj.joints = thetas;
        end
        function getScrews(obj)
            disp("This function can call FK_space to set screw vectors");
        end
        function printHome(obj)
            disp(obj.M);
        end
        function thetas = getJoints(obj)
            thetas = obj.joints;
        end
        function J_space_setScrews(obj,J)
            % If you have a space Jacobian, you can set screw axes here,
            % use carefully as column order is expected to be correct
            cols = length(J(1,:));
            for i = 1:cols
                obj.screws(i) = J(:,i);  % try -- dimensions must match
            end
        end
        function printPose(obj)
            disp(obj.pose);
        end
    end
end