%% IK SOLVER SETUP SCRIPT: ik_solver_setup.m

% 1. DEFINE ROBOT GEOMETRY (Link Lengths)
L1 = 0.1; % Base height/offset
L2 = 0.5; % Shoulder link
L3 = 0.4; % Elbow link
L4 = 0.05; % Wrist offset
L6 = 0.2; % End effector/gripper length

% Define where the robot base is mounted relative to the carousel center (0,0)
RobotBase_XYZ = [-1.5, 0, 1.0]; 

% 2. BUILD THE ROBOT (rigidBodyTree)
robot = rigidBodyTree('DataFormat','column');

% [Insert the code for adding all 6 links and the gripper here, 
%  using the L1...L6 variables and the BaseOffset_XYZ variable]
% --- Define Base Link (Link 1 - Yaw) ---
body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1', 'revolute'); % Revolute joint (moving)
jnt1.HomePosition = 0; % Start position
% The base of the robot is offset by RobotBase_XYZ from the world origin.
setFixedTransform(jnt1, trvec2tform(RobotBase_XYZ)); 
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% --- Link 2 (Shoulder Pitch) ---
body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('jnt2', 'revolute'); % Revolute joint (moving)
jnt2.HomePosition = pi/2; % Start with shoulder raised 90 degrees
% Translate up by L1 from the base to the shoulder joint axis
setFixedTransform(jnt2, trvec2tform([0, 0, L1])); 
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

% --- Link 3 (Elbow Pitch) ---
body3 = rigidBody('link3');
jnt3 = rigidBodyJoint('jnt3', 'revolute'); % Revolute joint (moving)
% Translate along X by L2 (shoulder link length) to the elbow joint axis
setFixedTransform(jnt3, trvec2tform([L2, 0, 0])); 
body3.Joint = jnt3;
addBody(robot, body3, 'link2');

% --- Link 4 (Wrist Pitch) ---
body4 = rigidBody('link4');
jnt4 = rigidBodyJoint('jnt4', 'revolute'); % Revolute joint (moving)
% Translate along X by L3 (elbow link length) to the wrist joint axis
setFixedTransform(jnt4, trvec2tform([L3, 0, 0])); 
body4.Joint = jnt4;
addBody(robot, body4, 'link3');

% --- Link 5 (Wrist Roll) ---
body5 = rigidBody('link5');
jnt5 = rigidBodyJoint('jnt5', 'revolute'); % Revolute joint (moving)
setFixedTransform(jnt5, trvec2tform([L4, 0, 0])); 
body5.Joint = jnt5;
addBody(robot, body5, 'link4');

% --- Link 6 (Wrist Yaw) ---
body6 = rigidBody('link6');
jnt6 = rigidBodyJoint('jnt6', 'revolute'); % Revolute joint (moving)
setFixedTransform(jnt6, trvec2tform([L4, 0, 0])); 
body6.Joint = jnt6;
addBody(robot, body6, 'link5');

% --- End Effector (Gripper) ---
endEffector = rigidBody('gripper');
% This is a FIXED connection from Link 6, not a moving joint.
setFixedTransform(endEffector.Joint, trvec2tform([L6, 0, 0]));
addBody(robot, endEffector, 'link6');

% ... (All the addBody and setFixedTransform commands from Step 3)

% 3. SETUP THE INVERSE KINEMATICS (IK) SOLVER
ik = inverseKinematics('RigidBodyTree', robot);

% Set IK Solver Parameters
ik.SolverAlgorithm = 'LevenbergMarquardt';
weights = [0.1 0.1 0.1 1 1 1]; % High weight for position (XYZ)
end_effector_name = 'gripper';
initial_guess = homeConfiguration(robot); 

% Optional: Save the objects to a file for easier loading in Simulink
% save('robot_objects.mat', 'robot', 'ik', 'initial_guess', 'weights', 'end_effector_name');