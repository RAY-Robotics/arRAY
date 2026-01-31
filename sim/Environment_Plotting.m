function [h_parcel] = Environment_Plotting(env, rack_pos, platform_pos)
% PLOT_ENVIRONMENT Draws the dual-shelf environment, parcel, and platform.
%
% Inputs:
%   env: A struct containing all environment dimensions
%        (e.g., env.aisle_width, env.shelf_length, etc.)
%   rack_pos: The [x,y,z] coordinate of the target parcel
%   platform_pos: The [x,y,z] coordinate of the drop-off platform
%
% Outputs:
%   h_parcel: A handle to the parcel plot object, so it can be moved

% --- Plot the parcel ---
% This is the specific target for the gripper
h_parcel = plot3(rack_pos(1), rack_pos(2), rack_pos(3), 's', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.8 0.5 0.2], 'MarkerEdgeColor', 'k');
text(rack_pos(1), rack_pos(2), rack_pos(3) + 0.2, 'Parcel', 'FontSize', 10, 'HorizontalAlignment', 'center');

% --- Draw the Two Shelves ---
% We can use a helper function to draw each shelf
draw_shelf('Shelf A', env.shelf_A_y_pos, env.shelf_length, env.shelf_height, env.shelf_depth, env.compartments_x, env.compartments_z);
draw_shelf('Shelf B', env.shelf_B_y_pos, env.shelf_length, env.shelf_height, env.shelf_depth, env.compartments_x, env.compartments_z);


% --- Plot the robotic platform ---
plat_size = 0.5; % half-width
plat_x = [platform_pos(1)-plat_size, platform_pos(1)+plat_size, platform_pos(1)+plat_size, platform_pos(1)-plat_size];
plat_y = [platform_pos(2)-plat_size, platform_pos(2)-plat_size, platform_pos(2)+plat_size, platform_pos(2)+plat_size];
plat_z = [platform_pos(3), platform_pos(3), platform_pos(3), platform_pos(3)];
patch(plat_x, plat_y, plat_z, 'm', 'FaceAlpha', 0.4, 'EdgeColor', 'k');
text(platform_pos(1), platform_pos(2), platform_pos(3) + 0.2, 'Platform Drop-off', 'FontSize', 10, 'HorizontalAlignment', 'center');
% Simple "platform" legs
plot3([plat_x; plat_x], [plat_y; plat_y], [plat_z; zeros(1,4)], 'k:');

end


% --- Helper function to draw one shelf ---
function draw_shelf(name, y_pos, len_x, h_z, depth, num_x, num_z)
    % Draw the main shelf frame (a box)
    % The shelf extends *away* from the aisle
    y_back = y_pos - sign(y_pos) * depth; % -depth for A, +depth for B
    
    % Draw back plane
    patch([0 len_x len_x 0], [y_back y_back y_back y_back], [0 0 h_z h_z], [0.7 0.7 0.7], 'FaceAlpha', 0.3);
    % Draw top plane
    patch([0 len_x len_x 0], [y_pos y_pos y_back y_back], [h_z h_z h_z h_z], [0.7 0.7 0.7], 'FaceAlpha', 0.3);
    % Draw side planes
    patch([0 0 0 0], [y_pos y_back y_back y_pos], [0 0 h_z h_z], [0.7 0.7 0.7], 'FaceAlpha', 0.3);
    patch([len_x len_x len_x len_x], [y_pos y_back y_back y_pos], [0 0 h_z h_z], [0.7 0.7 0.7], 'FaceAlpha', 0.3);

    % Draw the compartment grid on the front face
    
    % Vertical dividers
    x_divs = linspace(0, len_x, num_x + 1);
    for i = 1:length(x_divs)
        plot3([x_divs(i) x_divs(i)], [y_pos y_pos], [0 h_z], 'k-', 'Color', [0.4 0.4 0.4]);
    end
    
    % Horizontal dividers
    z_divs = linspace(0, h_z, num_z + 1);
    for i = 1:length(z_divs)
        plot3([0 len_x], [y_pos y_pos], [z_divs(i) z_divs(i)], 'k-', 'Color', [0.4 0.4 0.4]);
    end
    
    text(len_x/2, y_pos - sign(y_pos)*0.2, h_z + 0.2, name, 'HorizontalAlignment', 'center');
end