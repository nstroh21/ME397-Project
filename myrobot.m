classdef myrobot    %  < hw1 could "inherit" hw1 functions but doesn't make a lot of sense
                    % more likely it should be a handle ... < handle
    
    %   Instead I include the functions I need from HW1 in robot definition

    properties
        frame = "space";    %
        M;                  %Home Configuration of the end effector 
        numJoints;          %scalar to track number of joints in the system
        jointTypes;         %Binary List for each joint type, either 0 = "rot" , 1 = "prismatic"
        homeVectors;        %set of 6D vectors describing position and axis of rotation of joints at HOME positon. NOTE: THESE ARE CONSTANT
        screws;             %Set of 6D vectors describing screw axes derived from homeVectors, these update inside MOVE function
        joints;             % Vector 
        pose;
        %partner
    end

    methods
        % home config and table of screws, Constructor
        function obj = myrobot(frame,M,numJoints)   
            obj.frame = frame;
            obj.M = M;
            obj.pose = M;
            obj.numJoints = numJoints;
            obj.jointTypes = zeros(numJoints);  % 0 for rotation and 1 for prismatic
            obj.screws = zeros(numJoints,6);
            obj.joints = zeros(1,numJoints);
            formatSpec = 'Robot created with %d joints in %s frame.\n'; 
            fprintf(formatSpec,numJoints,frame);
        end

        % Only ever call this when you build it
        function obj = addJoint(obj,jointNumber, type, hv, VType)
            % Overwrties joint configuration if exists, V = homeVector position
            % Home vector = [A1 A2 A3 B1 B2 B3], where An = orientation of
            % the joint and Bn = location of the joint's origin, both in
            % the relevant base frame
             obj.jointTypes(jointNumber) = type;
            if VType == "homeVector"  
%             if isequal(obj.frame,  'space')
                % this adds a home vector
                obj.homeVectors(jointNumber,:) = hv;
                disp("Joint Added, Pose:");
                disp(obj.homeVectors);
                % calculates the screw axis
                v = -1.*skewSym(hv(1:3))*hv(4:6)';
                if type == 0
                    screw = [hv(1:3),v'];
                else
                    screw = hv;
                end
                % updates screw matrix
                obj.screws(jointNumber,:) = screw;
                disp("Screws Updated: ");
                disp(obj.screws);
            %setScrew(obj, vector, jointNumber); % code below sets the screw
            else 
                obj.screws(jointNumber,:) = hv;
                disp("Joint Added, Pose Unknown, Screws Updated: ");
%                 disp(obj.screws);
                
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

        function MOVE(obj, thetas)  % Decided we move one Joint at a time

            % TO DO: take in one joint, update joint vector and
            % corresponding screw axis
            % Technically: check if movement is legal (joint limits)

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