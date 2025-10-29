function [joint_vars] = ik_gantry(pos)
% IK_GANTRY Computes Inverse Kinematics for a 3-axis gantry
%
% Inputs:
%   pos: A 1x3 vector [x, y, z] representing the
%        desired Cartesian position of the end-effector.
%
% Outputs:
%   joint_vars: A 1x3 vector [px, py, pz] representing the
%               required linear positions of the three gantry axes.

% For a simple gantry, the required joint positions are
% identical to the desired end-effector position.

x = pos(1);
y = pos(2);
z = pos(3);

% Required joint positions (px, py, pz)
px = x;
py = y;
pz = z;

joint_vars = [px, py, pz];
end
