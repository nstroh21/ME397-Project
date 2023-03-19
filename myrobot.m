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
            disp("Robot Created With 6 Joints in Space Frame");
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
        function getScrews(obj)
            disp("This function can call FK_space to set screw vectors")
        end
        function printHome(obj)
            disp(obj.M)
        end
        function printPose(obj)
            disp(obj.pose)
        end
    end
end