% --- Main Simulation Script for Dual-Shelf Gantry System ---
%
% This script simulates a 3-axis gantry robot operating in an aisle
% between two storage shelves (Shelf A and Shelf B).
%
% It uses separate functions for:
% - Kinematics (fk_gantry, ik_gantry - UNCHANGED)
% - Environment Plotting (plot_environment - UPDATED)
% - Movement/Animation (move_gantry - UPDATED)
%
% To run, place all .m files in the same MATLAB directory
% and execute this script.

clc; 
clear; 
close all;

disp('Initializing Dual-Shelf Gantry Simulation...');

% --- 1. ENVIRONMENT DIMENSIONS CONTROL PANEL ---
% Define all your dimensions here.
env.aisle_width = 4;        % Distance between the two shelves (m)
env.shelf_depth = 1.5;      % Depth of the shelves (m)
env.shelf_length = 9;       % Length of the shelves along the X-axis (m)
env.shelf_height = 6;       % Total height of the shelves (m)

% Compartment setup (for a 9x4 = 36 compartment shelf)
env.compartments_x = 9;     % Number of compartments along X
env.compartments_z = 4;     % Number of compartments along Z

% --- 2. AUTOMATICALLY CALCULATED POSITIONS ---
% Shelf Y-positions (front faces)
env.shelf_A_y_pos = -env.aisle_width / 2; % Shelf A (Negative Y)
env.shelf_B_y_pos = env.aisle_width / 2;  % Shelf B (Positive Y)

% Compartment dimensions
comp_width = env.shelf_length / env.compartments_x;
comp_height = env.shelf_height / env.compartments_z;

% --- 3. TASK COORDINATES (THIS IS HOW YOU TWEAK THE TASK) ---
%
% --- Define the Target Parcel Location ---
% Pick a shelf:
target_shelf_y = env.shelf_A_y_pos; % Use env.shelf_A_y_pos or env.shelf_B_y_pos
% Pick a compartment (e.g., column 3, row 2)
target_comp_x_index = 3; % From 1 to env.compartments_x
target_comp_z_index = 2; % From 1 to env.compartments_z

% Calculate the 3D coordinate of that compartment's center
target_x = (target_comp_x_index - 0.5) * comp_width;
target_z = (target_comp_z_index - 0.5) * comp_height;
target_y_depth = target_shelf_y; % Position in middle of shelf depth

% This is the final [x,y,z] for the parcel
rack_pos = [target_x, target_y_depth, target_z];

% --- Define the Platform Location ---
% (In the middle of the aisle, halfway along its length, 1m high)
platform_pos = [env.shelf_length / 2, 0, 1];

% --- Define Safety and Home Positions ---
safe_height = env.shelf_height + 0.5; % Must be > shelf height
home_joints = [0, 0, safe_height]; % Home: (x=0, y=0, z=safe_height)

% --- 4. SETUP THE 3D PLOT ---
figure('Name', 'Gantry Kinematics Simulation', 'NumberTitle', 'off', 'Units', 'normalized', 'OuterPosition', [0.1 0.1 0.8 0.8]);
hold on;
grid on;
axis equal; % Ensures 1:1:1 aspect ratio
view(120, 25); % New view angle to see down the aisle

% Set plot limits based on environment dimensions
max_x = env.shelf_length;
max_y_limit = (env.aisle_width / 2) + 0.5; % For plotting
max_z_limit = env.shelf_height + 1; % For plotting
xlim([0 max_x]);
ylim([-max_y_limit max_y_limit]);
zlim([0 max_z_limit]);

xlabel('X-axis (Aisle Length)');
ylabel('Y-axis (Aisle Width)');
zlabel('Z-axis (Height)');
title('Dual-Shelf Gantry Simulation');

% --- 5. PLOT STATIC ENVIRONMENT ---
% Pass the entire 'env' struct to the drawing function
Environment_Plotting(env, rack_pos, platform_pos);

% --- 6. INITIALIZE GANTRY VISUALIZATION (New Model) ---
% Get initial EE position from home joint state using Forward Kinematics
current_pos = Forward_Kinematics_Model(home_joints);
px = current_pos(1);
py = current_pos(2);
pz = current_pos(3);

% Plot the end-effector (gripper)
h_gantry = plot3(px, py, pz, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'b');

% Plot the gantry structure (Aisle-Spanning Model)
% h_structure(1) = Z-arm (vertical hoist)
% h_structure(2) = Y-trolley (cart on the bridge)
% h_structure(3) = X-bridge (spans aisle, moves along X-rail)
h_structure(1) = plot3([px px], [py py], [pz max_z_limit], 'k-', 'LineWidth', 2.5, 'Color', [0.3 0.3 0.3]);
h_structure(2) = plot3([px px], [py-0.2 py+0.2], [max_z_limit max_z_limit], 'k-', 'LineWidth', 4, 'Color', 'r');
h_structure(3) = plot3([px px], [-max_y_limit max_y_limit], [max_z_limit max_z_limit], 'k--', 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);
legend(h_gantry, 'Gantry Gripper', 'Location', 'northwest');


% --- 7. SIMULATION PARAMETERS ---
time_step = 0.05;  % Simulation time step
move_time = 2.0;   % Time for a long move
short_move_time = 1.0; % Time for up/down/sideways moves

% --- 8. RUN THE NEW TASK SEQUENCE (Safer Logic) ---
disp('Starting task sequence...');
pause(1);

% 1. Move to Pre-Grasp position (correct X/Z, but center aisle Y=0)
disp('Moving to aisle position (X, Z)...');
pre_aisle_pos = [rack_pos(1), 0, rack_pos(3)];
current_pos = Movement_and_Animation(current_pos, pre_aisle_pos, move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 2. Move sideways (Y-axis) to grasp parcel
disp('Moving sideways to grasp...');
grasp_pos = rack_pos; % This is the [x, y, z] inside the shelf
current_pos = Movement_and_Animation(current_pos, grasp_pos, short_move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 3. Simulate Grasp
disp('Gripping parcel (Gripper Red)...');
set(h_gantry, 'MarkerFaceColor', 'r'); % Red = holding
pause(0.5);

% 4. Retract from shelf (Y-axis) back to center aisle
disp('Retracting from shelf...');
current_pos = Movement_and_Animation(current_pos, pre_aisle_pos, short_move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 5. Move to safe height
disp('Moving to safe height...');
safe_travel_pos = [rack_pos(1), 0, safe_height];
current_pos = Movement_and_Animation(current_pos, safe_travel_pos, short_move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 6. Move to pre-release position (above platform, center aisle)
disp('Moving to pre-release position (X)...');
pre_release_pos = [platform_pos(1), 0, safe_height];
current_pos = Movement_and_Animation(current_pos, pre_release_pos, move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 7. Move down to release (Z-axis)
disp('Moving down to release...');
release_pos = platform_pos;
current_pos = Movement_and_Animation(current_pos, release_pos, short_move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 8. Simulate Release
disp('Releasing parcel (Gripper Blue)...');
set(h_gantry, 'MarkerFaceColor', 'b'); % Blue = empty
pause(0.5);

% 9. Move up to safe height
disp('Moving back to safe height...');
current_pos = Movement_and_Animation(current_pos, pre_release_pos, short_move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

% 10. Return to Home
disp('Returning to home position...');
home_pos = Forward_Kinematics_Model(home_joints);
current_pos = Movement_and_Animation(current_pos, home_pos, move_time, time_step, h_gantry, h_structure, max_y_limit, max_z_limit);

disp('Task Complete.');

% --- End of Main Script ---

