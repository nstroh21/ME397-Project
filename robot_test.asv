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

robotSpace = myrobot('body', M, 3);
hv1 = [0 0 1 0 0 0];
robotSpace = robotSpace.addJoint(1, 0, hv1, 'homeVector');
hv2 = [0 0 1 1 0 0];
robotSpace = robotSpace.addJoint(2, 0, hv2, 'homeVector');
hv3 = [0 0 1 2 0 0];
robotSpace = robotSpace.addJoint(3, 0, hv3, 'homeVector');
% hv4 = [0 0 1 2 0 0];
% robotSpace = robotSpace.addJoint(4, 1, hv4, 'homeVector');

joints = [pi, 0, 0];
J_body(robotSpace, joints)


