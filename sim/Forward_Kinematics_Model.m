function [pos] = fk_gantry(joint_vars)
% FK_GANTRY Computes Forward Kinematics for a 3-axis gantry
%
% Inputs:
%   joint_vars: A 1x3 vector [px, py, pz] representing the
%               linear positions of the three gantry axes.
%
% Outputs:
%   pos: A 1x3 vector [x, y, z] representing the
%        Cartesian position of the end-effector in the world frame.

% For a simple gantry, the end-effector position is
% identical to the joint positions, assuming the world
% origin (0,0,0) is the same as the joint origin.

px = joint_vars(1);
py = joint_vars(2);
pz = joint_vars(3);

% World position (x, y, z)
x = px;
y = py;
z = pz;

pos = [x, y, z];
end
