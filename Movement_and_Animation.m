function [final_pos] = Movement_and_Animation(start_pos, end_pos, T, dt, ...
    h_bin, h_structure, max_y, max_z, bin_dims, ...
    h_parcel, is_holding, masses, g, h_table_text)
% MOVE_GANTRY Animates the gantry movement from start to end
%
% This version animates the gantry structure, the 3D bin,
% moves the parcel, and updates the parameter table.
%
% Inputs:
%   start_pos:   [x,y,z] starting position
%   end_pos:     [x,y,z] ending position
%   T:           Time for move (seconds)
%   dt:          Time step (seconds)
%   h_bin:       Handle to the gantry bin (patch object)
%   h_structure: Handle to the gantry structure (3 parts)
%   max_y:       The Y-limit of the aisle (for drawing the bridge)
%   max_z:       The Z-limit (height of the gantry rails)
%   bin_dims:    [L, W, H] of the bin
%   h_parcel:    Handle to the parcel plot object
%   is_holding:  Boolean (true/false) if gantry is holding the parcel
%   masses:      Struct with .m_parcel, .m_hoist, etc.
%   g:           Gravitational constant (e.g., 9.81)
%   h_table_text: Handle to the annotation text box for parameter display

% --- 1. Calculate Total Masses for this move ---
if is_holding
    current_parcel_mass = masses.m_parcel;
    parcel_status = 'Holding Parcel';
    parcel_mass_str = sprintf('%.1f kg', masses.m_parcel);
else
    current_parcel_mass = 0;
    parcel_status = 'Empty';
    parcel_mass_str = '0.0 kg';
end

% Cumulative masses
m_Z_total = masses.m_hoist + current_parcel_mass;
m_Y_total = masses.m_trolley + m_Z_total;
m_X_total = masses.m_bridge + m_Y_total;


% --- 2. Create time vector ---
time_vec = 0:dt:T;
if time_vec(end) ~= T
    time_vec(end+1) = T; % Ensure final point is reached
end

delta_pos = end_pos - start_pos; % Total distance vector
accel_const = 0.5 * (pi/T)^2; % Constant for acceleration calculation

% --- Pre-allocate for bin dimensions ---
L = bin_dims(1);
W = bin_dims(2);
H = bin_dims(3);
% 8 vertices relative to the center [0,0,0]
verts_offset = [
    -L/2, -W/2, -H/2; % 1
    +L/2, -W/2, -H/2; % 2
    +L/2, +W/2, -H/2; % 3
    -L/2, +W/2, -H/2; % 4
    -L/2, -W/2, +H/2; % 5
    +L/2, -W/2, +H/2; % 6
    +L/2, +W/2, +H/2; % 7
    -L/2, +W/2, +H/2  % 8
];

for i = 1:length(time_vec)
    t = time_vec(i);
    
    % --- 3. Trajectory Calculation ---
    % Position (Ease-in, Ease-out)
    s = 0.5 * (1 - cos(pi * t / T));
    current_pos = start_pos + s * delta_pos;
    
    % Acceleration (2nd derivative of position)
    % a(t) = delta_pos * 0.5 * (pi/T)^2 * cos(pi*t/T)
    a_scalar = accel_const * cos(pi * t / T);
    a_vec = delta_pos * a_scalar;

    
    % --- 4. NEW: Force Calculation ---
    % F = m*a
    Fx = m_X_total * a_vec(1);
    Fy = m_Y_total * a_vec(2);
    % Fz = m*a_z + F_gravity (always fighting gravity)
    Fz = m_Z_total * a_vec(3) + m_Z_total * g;
    
    % --- 5. NEW: Update Parameter Table Display ---
    % Create a cell array of strings for the table
    table_str = { ...
        '-- DYNAMIC PARAMETERS --', ...
        ' ', ...
        sprintf('Status: %s', parcel_status), ...
        sprintf('Parcel Mass: %s', parcel_mass_str), ...
        '____________________', ...
        ' ', ...
        '-- Motor Forces (N) --', ...
        sprintf('Fx (Bridge): %8.1f', Fx), ...
        sprintf('Fy (Trolley): %8.1f', Fy), ...
        sprintf('Fz (Hoist):  %8.1f', Fz), ...
        '____________________', ...
        ' ', ...
        '-- Total Mass (kg) --', ...
        sprintf('X-Axis Total: %8.1f', m_X_total), ...
        sprintf('Y-Axis Total: %8.1f', m_Y_total), ...
        sprintf('Z-Axis Total: %8.1f', m_Z_total) ...
    };
    % Set the string of the text box
    set(h_table_text, 'String', table_str);

    
    % --- 6. Kinematics (trivial) ---
    joint_vars = Inverse_Kinematics_Model(current_pos);
    ee_pos = Forward_Kinematics_Model(joint_vars);
    px = ee_pos(1);
    py = ee_pos(2);
    pz = ee_pos(3);
    
    
    % --- 7. Update Plot Handles ---
    
    % --- 7a. NEW: Update Bin Prism Vertices ---
    % Calculate 8 vertices based on current ee_pos
    new_verts = repmat(ee_pos, 8, 1) + verts_offset;
    set(h_bin, 'Vertices', new_verts);
    
    % --- 7b. Gantry Structure ---
    set(h_structure(1), 'XData', [px px], 'YData', [py py], 'ZData', [pz max_z]);
    set(h_structure(2), 'XData', [px px], 'YData', [py-0.2 py+0.2], 'ZData', [max_z max_z]);
    set(h_structure(3), 'XData', [px px], 'YData', [-max_y max_y], 'ZData', [max_z max_z]);
    
    % --- 7c. Parcel ---
    if is_holding
        % "Parent" the parcel to the gripper/bin
        set(h_parcel, 'XData', px, 'YData', py, 'ZData', pz);
    end
    
    drawnow; % Redraw the figure
    pause(dt/2); 
end

% --- 8. Set final state ---
% Ensure gantry and parcel are at the exact end_pos
final_pos = end_pos;
joint_vars = Inverse_Kinematics_Model(final_pos);
ee_pos = Forward_Kinematics_Model(joint_vars);
px = ee_pos(1); py = ee_pos(2); pz = ee_pos(3);

% --- NEW: Update Bin Vertices ---
new_verts = repmat(ee_pos, 8, 1) + verts_offset;
set(h_bin, 'Vertices', new_verts);

% --- Update Structure ---
set(h_structure(1), 'XData', [px px], 'YData', [py py], 'ZData', [pz max_z]);
set(h_structure(2), 'XData', [px px], 'YData', [py-0.2 py+0.2], 'ZData', [max_z max_z]);
set(h_structure(3), 'XData', [px px], 'YData', [-max_y max_y], 'ZData', [max_z max_z]);

% --- Update Parcel ---
if is_holding
    set(h_parcel, 'XData', px, 'YData', py, 'ZData', pz);
end

% --- 9. NEW: Update table for static (end of move) state ---
% Fx and Fy are 0 because acceleration is 0
% Fz is just the static force of gravity
Fz_static = m_Z_total * g;
table_str = { ...
    '-- DYNAMIC PARAMETERS --', ...
    ' ', ...
    sprintf('Status: %s', parcel_status), ...
    sprintf('Parcel Mass: %s', parcel_mass_str), ...
    '____________________', ...
    ' ', ...
    '-- Motor Forces (N) --', ...
    sprintf('Fx (Bridge): %8.1f', 0.0), ...
    sprintf('Fy (Trolley): %8.1f', 0.0), ...
    sprintf('Fz (Hoist):  %8.1f', Fz_static), ...
    '____________________', ...
    ' ', ...
    '-- Total Mass (kg) --', ...
    sprintf('X-Axis Total: %8.1f', m_X_total), ...
    sprintf('Y-Axis Total: %8.1f', m_Y_total), ...
    sprintf('Z-Axis Total: %8.1f', m_Z_total) ...
};
set(h_table_text, 'String', table_str);
drawnow;

end