function [final_pos] = move_gantry(start_pos, end_pos, T, dt, h_gantry, h_structure, max_y, max_z)
% MOVE_GANTRY Animates the gantry movement from start to end
%
% This version animates the new aisle-spanning gantry model.
%
% Inputs:
%   ... (same as before) ...
%   h_structure: Handle to the gantry structure (3 parts)
%   max_y:       The Y-limit of the aisle (for drawing the bridge)
%   max_z:       The Z-limit (height of the gantry rails)

% Create time vector
time_vec = 0:dt:T;
if time_vec(end) ~= T
    time_vec(end+1) = T; % Ensure final point is reached
end

for i = 1:length(time_vec)
    t = time_vec(i);
    
    % Simple smooth trajectory (Ease-in, Ease-out using cosine)
    s = 0.5 * (1 - cos(pi * t / T));
    
    % Calculate current desired EE position
    current_pos = start_pos + s * (end_pos - start_pos);
    
    % Get joint variables (trivial for gantry)
    joint_vars = Inverse_Kinematics_Model(current_pos);
    
    % Get actual EE position (trivial for gantry)
    ee_pos = Forward_Kinematics_Model(joint_vars);
    
    px = ee_pos(1);
    py = ee_pos(2);
    pz = ee_pos(3);
    
    % --- Update Plot Handles (New Visualization) ---
    
    % 1. End-effector
    set(h_gantry, 'XData', px, 'YData', py, 'ZData', pz);
    
    % 2. Structure lines
    % h_structure(1) = Z-arm (vertical hoist)
    set(h_structure(1), 'XData', [px px], 'YData', [py py], 'ZData', [pz max_z]);
    % h_structure(2) = Y-trolley (cart on the bridge)
    set(h_structure(2), 'XData', [px px], 'YData', [py-0.2 py+0.2], 'ZData', [max_z max_z]);
    % h_structure(3) = X-bridge (spans aisle, moves along X-rail)
    set(h_structure(3), 'XData', [px px], 'YData', [-max_y max_y], 'ZData', [max_z max_z]);
    
    drawnow; % Redraw the figure
    pause(dt/2); 
end

% --- Set final position ---
final_pos = end_pos;
joint_vars = Inverse_Kinematics_Model(final_pos);
ee_pos = Forward_Kinematics_Model(joint_vars);
px = ee_pos(1); py = ee_pos(2); pz = ee_pos(3);

set(h_gantry, 'XData', px, 'YData', py, 'ZData', pz);
set(h_structure(1), 'XData', [px px], 'YData', [py py], 'ZData', [pz max_z]);
set(h_structure(2), 'XData', [px px], 'YData', [py-0.2 py+0.2], 'ZData', [max_z max_z]);
set(h_structure(3), 'XData', [px px], 'YData', [-max_y max_y], 'ZData', [max_z max_z]);
drawnow;

end

