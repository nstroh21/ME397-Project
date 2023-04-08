clear

%% Forward Kinematics: Example with 6R open chain - Spatial Form

M = [1 0 0 0;
     0 1 0 3;
     0 0 1 0;
     0 0 0 1];

robotSpace = myrobot('space', M, 6);
hv1 = [0 0 1 0 0 0];
robotSpace = robotSpace.addJoint(1, 0, hv1, 'homeVector');
hv2 = [0 1 0 0 0 0];
robotSpace = robotSpace.addJoint(2, 0, hv2, 'homeVector');
hv3 = [-1 0 0 0 0 0];
robotSpace = robotSpace.addJoint(3, 0, hv3, 'homeVector');
hv4 = [-1 0 0 0 1 0];
robotSpace = robotSpace.addJoint(4, 0, hv4, 'homeVector');
hv5 = [-1 0 0 0 2 0];
robotSpace = robotSpace.addJoint(5, 0, hv5, 'homeVector');
hv6 = [0 1 0 0 0 0];
robotSpace = robotSpace.addJoint(6, 0, hv6, 'homeVector');

FK_space(robotSpace, [0 0 0 0 0 0])
FK_space(robotSpace, [0 pi 0 pi 0 pi])


%% Forward Kinematics: Example with 6R open chain - Body Form

M = [1 0 0 0;
     0 1 0 3;
     0 0 1 0;
     0 0 0 1];

robotBody = myrobot('body', M, 6);
hv1 = [0 0 1 0 -3 0];
robotBody = robotBody.addJoint(1, 0, hv1, 'homeVector');
hv2 = [0 1 0 0 0 0];
robotBody = robotBody.addJoint(2, 0, hv2, 'homeVector');
hv3 = [-1 0 0 0 -3 0];
robotBody = robotBody.addJoint(3, 0, hv3, 'homeVector');
hv4 = [-1 0 0 0 -2 0];
robotBody = robotBody.addJoint(4, 0, hv4, 'homeVector');
hv5 = [-1 0 0 0 -1 0];
robotBody = robotBody.addJoint(5, 0, hv5, 'homeVector');
hv6 = [0 1 0 0 0 0];
robotBody = robotBody.addJoint(6, 0, hv6, 'homeVector');

FK_body(robotBody, [0 0 0 0 0 0])
FK_body(robotBody, [0 pi 0 pi 0 pi])

%% Forward Kinematics: WAM Arm - Body Form

L1 = 0.550; L2 = 0.300; L3 = 0.060; W1 = 0.045;
M = [1 0 0 0;
     0 1 0 0;
     0 0 1 L1+L2+L3;
     0 0 0 1];

robot_WAM = myrobot('body', M, 7);
hv1 = [0 0 1 0 0 -(L1 + L2 + L3)];
robot_WAM = robot_WAM.addJoint(1, 0, hv1, 'homeVector');
hv2 = [0, 1, 0, 0, 0, -(L1 + L2 + L3)];
robot_WAM = robot_WAM.addJoint(2, 0, hv2, 'homeVector');
hv3 = [0 0 1 0 0 -(L1 + L2 + L3)];
robot_WAM = robot_WAM.addJoint(3, 0, hv3, 'homeVector');
hv4 = [0, 1, 0, W1, 0, -(L3 + L2)];
robot_WAM = robot_WAM.addJoint(4, 0, hv4, 'homeVector');
hv5 = [0 0 1 0 0 -L3];
robot_WAM = robot_WAM.addJoint(5, 0, hv5, 'homeVector');
hv6 = [0 1 0 0 0 -L3];
robot_WAM = robot_WAM.addJoint(6, 0, hv6, 'homeVector');
hv7 = [0 0 1 0 0 -L3];
robot_WAM = robot_WAM.addJoint(7, 0, hv7, 'homeVector');

joints = [0 pi/4 0 -pi/4 0 -pi/2 0]
FK_body(robot_WAM, joints)

%% Forward Kinematics: KUKA Robot - Body Form

M = [1 0 0 1.245;
     0 1 0 0;
     0 0 1 1.270;
     0 0 0 1];

robot_kuka = myrobot('body', M, 6);
hv1 = [1 0 0 0 0 0];
hv2 = [0 -1 0 -0.215 0 0];
hv3 = [1 0 0 -0.660 0 0];
hv4 = [0 -1 0 -0.995 0 0]; 
hv5 = [0 -1 0 -0.995 0 -0.770];
hv6 = [0 0 1 -1.245 0 -1.270];

robot_kuka = robot_kuka.addJoint(1, 0, hv6, 'homeVector');
robot_kuka = robot_kuka.addJoint(2, 0, hv5, 'homeVector');
robot_kuka = robot_kuka.addJoint(3, 0, hv4, 'homeVector');
robot_kuka = robot_kuka.addJoint(4, 0, hv3, 'homeVector');
robot_kuka = robot_kuka.addJoint(5, 0, hv2, 'homeVector');
robot_kuka = robot_kuka.addJoint(6, 0, hv1, 'homewVector');


FK_body(robot_kuka, [0 0 0 0 0 0])
q1 = [0 0 pi/2 0 0 0]
FK_body(robot_kuka, q1)
q2 = [0 -pi/2 pi/2 0 0 0]
FK_body(robot_kuka, q2)
q3 = [0 -pi/2 pi/2 pi/2 0 0]
FK_body(robot_kuka, q3)
q4 = [0 -pi/2 pi/2 pi/2 0 pi/2] 
FK_body(robot_kuka, q4)

%% Forward Kinematics: KUKA robot - space form
M = [1 0 0 1.245;
     0 1 0 0;
     0 0 1 1.270;
     0 0 0 1];

pose_end = eye(3);
q_end = [1.245 0 1.270]'
%
M = cat(1, [pose_end q_end], [0 0 0 1]);
kuka  = myrobot("space", M, 6)

% Use Home Vectors: (omega, q) where q points to a point on the screw axis.
% omega is the direction of the axis of rot. q is pointing to origins of
% joint frames in this case
v1 = [0  0  1   0      0    .159];
v2 = [0 -1  0  .250    0    .500];  
v3 = [0 -1  0  .250    0    1.27];  
v4 = [1  0  0  .585    0    1.27];
v5 = [0 -1  0  1.03    0    1.27];
v6 = [1  0  0  1.245   0    1.27];


kuka = kuka.addJoint(1, 0, v1, "pose");
kuka = kuka.addJoint(2, 0, v2, "pose");
kuka = kuka.addJoint(3, 0, v3, "pose");
kuka = kuka.addJoint(4, 0, v4, "pose");
kuka = kuka.addJoint(5, 0, v5, "pose");
kuka = kuka.addJoint(6, 0, v6, "pose");

q1 = [0 0 pi/2 0 0 0]
J_space(kuka, q1)
q2 = [0 -pi/2 pi/2 0 0 0]
J_space(kuka, q2)
q3 = [0 -pi/2 pi/2 pi/2 0 0]
J_space(kuka, q3)
q4 = [0 -pi/2 pi/2 pi/2 0 pi/2] 
J_space(kuka, q4)



%% Jacobian: Example with SCARA robot - Space Form

M = [1 0 0 0;
     0 1 0 3;
     0 0 1 0;
     0 0 0 1];

robotSpace = myrobot('space', M, 3);
hv1 = [0 0 1 0 0 0];
robotSpace = robotSpace.addJoint(1, 0, hv1, 'homeVector');
hv2 = [0 0 1 1 0 0];
robotSpace = robotSpace.addJoint(2, 0, hv2, 'homeVector');
hv3 = [0 0 1 2 0 0];
robotSpace = robotSpace.addJoint(3, 0, hv3, 'homeVector');
% hv4 = [0 0 1 2 0 0];
% robotSpace = robotSpace.addJoint(4, 1, hv4, 'homeVector');

joints = [pi, 0, 0];
J_space(robotSpace, joints)

%% Jacobian: Example with SCARA robot - Body Form

M = [1 0 0 0;
     0 1 0 3;
     0 0 1 0;
     0 0 0 1];

robotSpace = myrobot('body', M, 4);
hv1 = [0 0 1 0 0 0];
robotSpace = robotSpace.addJoint(1, 0, hv1, 'homeVector');
hv2 = [0 0 1 -1 0 0];
robotSpace = robotSpace.addJoint(2, 0, hv2, 'homeVector');
hv3 = [0 0 1 -2 0 0];
robotSpace = robotSpace.addJoint(3, 0, hv3, 'homeVector');
hv4 = [0 0 0 0 0 1];
robotSpace = robotSpace.addJoint(4, 1, hv4, 'homeVector');



joints = [pi, pi/2, 0, 2];
body = J_body(robotSpace, joints)

%% Jacobian: Example with KUKA robot - body form

M = [1 0 0 1.245;
     0 1 0 0;
     0 0 1 1.270;
     0 0 0 1];

robot_kuka = myrobot('body', M, 6);
hv1 = [1 0 0 0 0 0];
hv2 = [0 -1 0 -0.215 0 0];
hv3 = [1 0 0 -0.660 0 0];
hv4 = [0 -1 0 -0.995 0 0]; 
hv5 = [0 -1 0 -0.995 0 -0.770];
hv6 = [0 0 1 -1.245 0 -1.270];

robot_kuka = robot_kuka.addJoint(1, 0, hv6, 'homeVector');
robot_kuka = robot_kuka.addJoint(2, 0, hv5, 'homeVector');
robot_kuka = robot_kuka.addJoint(3, 0, hv4, 'homeVector');
robot_kuka = robot_kuka.addJoint(4, 0, hv3, 'homeVector');
robot_kuka = robot_kuka.addJoint(5, 0, hv2, 'homeVector');
robot_kuka = robot_kuka.addJoint(6, 0, hv1, 'homeVector');

body = J_body(robot_kuka, [0 0 0 pi/2 pi pi])

% % robot_kuka.screws'
% adjoint(inv(M))*body
space = J_space(kuka, [0 0 0 pi/2 pi pi]);
% % kuka.screws'
adjoint(inv(M))*space

%% Analytical singularity
% syms L1 L2
% M = [1 0 0 L1+L2;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];
% 
% % robot_2link = myrobot('space', M, 2);
% s1 = [0 0 1 0 0 0];
% % robot_2link = robot_2link.addJoint(1, 0, hv1, 'pose');
% s2 = [0 0 1 0 -L1 0];
% % robot_2link = robot_2link.addJoint(2, 0, hv2, 'pose');
% 
% j = 1;
% J = zeros(6, 2);
% screws = cat(1, s1, s2);
% screws = [0 0 1 0 0 0;
%           0 0 1 0 -L1 0];
% angles = [0, pi/2];
% while j <= 2
%     S = screws(j,:); %row vector
%     if j == 1
%         T = eye(4);
%         J(:,j) = S;
%         j = j + 1;
%     else
%         E = chaslesMozzi(screws(j-1,:), angles(j-1), 0)  ;
%         Tj = T*E;
% 
%         J(:,j) = adjoint(Tj)*S';
%         j = j + 1;
%         T = Tj;
%     end
% end

M = [1 0 0 1.245;
     0 1 0 0;
     0 0 1 1.270;
     0 0 0 1];

robot_kuka = myrobot('body', M, 6);
hv1 = [1 0 0 0 0 0];
hv2 = [0 -1 0 -0.215 0 0];
hv3 = [1 0 0 -0.660 0 0];
hv4 = [0 -1 0 -0.995 0 0]; 
hv5 = [0 -1 0 -0.995 0 -0.770];
hv6 = [0 0 1 -1.245 0 -1.270];

robot_kuka = robot_kuka.addJoint(1, 0, hv6, 'homeVector');
robot_kuka = robot_kuka.addJoint(2, 0, hv5, 'homeVector');
robot_kuka = robot_kuka.addJoint(3, 0, hv4, 'homeVector');
robot_kuka = robot_kuka.addJoint(4, 0, hv3, 'homeVector');
robot_kuka = robot_kuka.addJoint(5, 0, hv2, 'homeVector');
robot_kuka = robot_kuka.addJoint(6, 0, hv1, 'homeVector');

is_singularity(robot_kuka, [1, 0, pi/2, 0, 1, 0])
is_singularity(robot_kuka, [1, -pi/2, pi/2, 0, 1, 0])
is_singularity(robot_kuka, [1, -pi/2, pi/4, 0, 1, 0])

