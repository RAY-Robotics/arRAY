%% ROBOT SETUP SCRIPT: robot_setup.m

% 1. Create the Robot Model
robot = rigidBodyTree('DataFormat','column'); % Use 'column' for standard vector inputs

% 2. Define Link Lengths (Adjust these to fit your truck/carousel dimensions)
L1 = 0.1; % Base height/offset
L2 = 0.5; % Shoulder link
L3 = 0.4; % Elbow link
L4 = 0.05; % Wrist offset
L6 = 0.2; % End effector/gripper length

% 3. Define the Robot Base Position (e.g., mounted on the truck wall)
% This defines where the robot sits relative to the carousel center (0,0)
BaseOffset_XYZ = [-1.5, 0, 1.0]; % Example: 1.5m to the side, centered height

% 4. Build the Robot using Standard Kinematic Structure (modified DH parameters)
% Links are added sequentially to the robot tree.

% Link 1: Base Rotation (Yaw)
body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
setFixedTransform(jnt1, trvec2tform(BaseOffset_XYZ), 'xyz');
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% Link 2: Shoulder Pitch
body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
setFixedTransform(jnt2, trvec2tform([0, 0, L1]), 'xyz'); % Move up by L1
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

% Link 3: Elbow Pitch
body3 = rigidBody('link3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
setFixedTransform(jnt3, trvec2tform([L2, 0, 0]), 'xyz'); % Translate out by L2
body3.Joint = jnt3;
addBody(robot, body3, 'link2');

% Link 4: Wrist Pitch
body4 = rigidBody('link4');
jnt4 = rigidBodyJoint('jnt4', 'revolute');
setFixedTransform(jnt4, trvec2tform([L3, 0, 0]), 'xyz'); % Translate out by L3
body4.Joint = jnt4;
addBody(robot, body4, 'link3');

% Link 5 & 6 (for orientation control)
body5 = rigidBody('link5');
jnt5 = rigidBodyJoint('jnt5', 'revolute');
setFixedTransform(jnt5, trvec2tform([L4, 0, 0]), 'xyz'); 
body5.Joint = jnt5;
addBody(robot, body5, 'link4');

body6 = rigidBody('link6');
jnt6 = rigidBodyJoint('jnt6', 'revolute');
setFixedTransform(jnt6, trvec2tform([L4, 0, 0]), 'xyz');
body6.Joint = jnt6;
addBody(robot, body6, 'link5');

% End Effector (Gripper)
endEffector = rigidBody('gripper');
setFixedTransform(endEffector.Joint, trvec2tform([L6, 0, 0]), 'xyz');
addBody(robot, endEffector, 'link6');

% Test visualization (optional, run in command window after running script)
% show(robot);