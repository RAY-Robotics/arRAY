function RAY_Carousel_Simulation_Final_v4()
    % RAY_Carousel_Simulation_Final_v4
    % - INCLUDES CAROUSEL PERFORMANCE ANALYSIS -
    
    clc; clear; close all;
    %% 1. SYSTEM CONFIGURATION
    
    % --- VIEW STYLE PARAMETER ---
    Shell_Style = 'Wireframe'; 
    % --- Dimensions ---
    System.Length = 3.0;       
    System.Width = 1.2;        
    
    % Vertical Layout
    System.Levels = 5;         
    System.Level_Height = 0.45;
    System.Base_Height = 0.2;  
    
    % Bin Dimensions
    Bin.Length = 0.30;         
    Bin.Depth  = 0.40;         
    Bin.Height = 0.30;         
    Bin.Mass   = 5.0;          
    
    % Track Geometry
    System.Track_Width = 0.8;  % m (Center-to-center)
    Curve_Radius = System.Track_Width / 2;               
    Pole_Distance = System.Length - System.Track_Width;  
    Track_Perimeter = (2 * Pole_Distance) + (2 * pi * Curve_Radius);
    
    % Drive Sprocket Radius (Used for Torque calculation)
    R_drive = Curve_Radius * 0.9; % Assumes drive is on the sprocket
    
    % Collision Spacing
    Safety_Gap = 0.15; 
    Bin_Pitch = Bin.Length + Safety_Gap;
    
    % Bin Count Calculation
    Max_Bins_Geo = floor(Track_Perimeter / Bin_Pitch);
    if 50 > Max_Bins_Geo
        System.N_Bins_Per_Level = Max_Bins_Geo;
        fprintf('NOTICE: Reduced to %d bins per level for safety gap.\n', Max_Bins_Geo);
    else
        System.N_Bins_Per_Level = 50;
    end
    
    Total_Bins = System.N_Bins_Per_Level * System.Levels;
    Total_Mass = Total_Bins * Bin.Mass; 
    
    %% 2. GRAPHICS SETUP
    
    f = figure('Name', 'RAY: Final High-Fidelity Simulation', 'Color', 'w', 'NumberTitle', 'off');
    set(f, 'Position', [50, 50, 1200, 800]);
    axis equal; grid on; hold on;
    view([-20, 25]); 
    
    % --- 2.1 VAN EXTERIOR (Black) ---
    Van.L = 3.8; Van.W = 2.2; Van.H = 2.8;
    Van_Origin = [-0.6, -1.1, 0]; % Center Y around 0
    draw_shell_custom(Van_Origin, Van.L, Van.W, Van.H, [0 0 0], 'Wireframe', 'Van');
    
    % --- 2.2 SYSTEM HOUSING (White) ---
    House.L = System.Length + 0.2; 
    House.W = System.Width + 0.2;
    House.H = (System.Levels * System.Level_Height) + System.Base_Height + 0.2;
    House_Origin = [-Curve_Radius-0.1, -Curve_Radius-0.1, 0];
    draw_shell_custom(House_Origin, House.L, House.W, House.H, [1 1 1], Shell_Style, 'Housing');
    
    % Set scenic limits
    xlim([Van_Origin(1)-1, Van_Origin(1)+Van.L+1]);
    ylim([Van_Origin(2)-1, Van_Origin(2)+Van.W+1]);
    zlim([0, Van.H+0.5]);
    xlabel('Length (X)'); ylabel('Width (Y)'); zlabel('Height (Z)');
    
    % --- 2.3 MECHANISM: RODS & SPROCKETS ---
    Shaft_Height = House.H - 0.2;
    draw_cylinder(0, 0, 0, 0.05, Shaft_Height, [0.4 0.4 0.4]); % Rear Shaft
    draw_cylinder(Pole_Distance, 0, 0, 0.05, Shaft_Height, [0.4 0.4 0.4]); % Front Shaft
    
    for k = 1:System.Levels
        h_lvl = System.Base_Height + (k-1)*System.Level_Height + Bin.Height + 0.05; % Slightly above bin
        % Rear Sprocket
        draw_cylinder(0, 0, h_lvl, Curve_Radius*0.9, 0.02, [0.6 0.6 0.6]);
        % Front Sprocket
        draw_cylinder(Pole_Distance, 0, h_lvl, Curve_Radius*0.9, 0.02, [0.6 0.6 0.6]);
        
        % Draw Track Line
        s_plot = linspace(0, Track_Perimeter, 200);
        [tx, ty, ~] = get_track_pos(s_plot, Pole_Distance, Curve_Radius);
        plot3(tx, ty, ones(size(tx))*(h_lvl), 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
    end
    
    % --- 2.4 INITIALIZE BINS & HANGERS ---
    Bin_Handles = gobjects(System.Levels, System.N_Bins_Per_Level);
    Hanger_Handles = gobjects(System.Levels, System.N_Bins_Per_Level);
    
    Bin_Positions = linspace(0, Track_Perimeter, System.N_Bins_Per_Level + 1);
    Bin_Positions(end) = [];
    
    for lvl = 1:System.Levels
        for i = 1:System.N_Bins_Per_Level
            c_base = [0.2, 0.4, 0.8]; 
            Bin_Handles(lvl, i) = patch('Vertices', [], 'Faces', [], ...
                'FaceColor', c_base, 'EdgeColor', 'k', 'FaceAlpha', 1.0);
            
            Hanger_Handles(lvl, i) = patch('Vertices', [], 'Faces', [], ...
                'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none'); 
        end
    end
    
    % Info Text
    h_text = text(Van_Origin(1), Van_Origin(2)-0.8, Van.H, 'Status: Idle', ...
        'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');
    h_perf = text(Van_Origin(1)+2, Van_Origin(2)-0.8, Van.H, 'Metrics', ...
        'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');
    % Drop Off Zone Marker
    text(-Curve_Radius-0.5, 0, 0.1, 'DROP OFF ZONE', 'Color', 'r', 'FontWeight', 'bold', 'Rotation', 90);
    
    %% 3. SIMULATION LOOP
    
    % --- Physics Parameters ---
    Max_Vel = 0.8; 
    Accel = 0.5; 
    Friction_Coeff = 0.08; 
    Gravity = 9.81;
    
    % --- Access Point ---
    s_access = (2 * Pole_Distance) + (1.5 * pi * Curve_Radius);
    s_access = mod(s_access, Track_Perimeter);
    
    % --- PERFORMANCE ANALYSIS SETUP ---
    N_PICKS = 10; % Number of picks to simulate
    Cycle_Times = zeros(1, N_PICKS);
    Rotation_Times = zeros(1, N_PICKS);
    Total_Time_Active = 0;
    
    % --- MOTOR SIZING DATA LOGGING ---
    Total_Time_History = [];
    Force_History = [];
    Power_History = [];
    Torque_History = [];
    Current_Sim_Time = 0; 
    
    for pick_k = 1:N_PICKS
        if ~ishandle(f); break; end
        
        % 3.1 TARGET SELECTION (Executed once per pick cycle)
        Target_Col_Idx = randi(System.N_Bins_Per_Level);
        Target_Lvl_Idx = randi(System.Levels);
        
        % Color Highlighting
        for lvl = 1:System.Levels
            for col = 1:System.N_Bins_Per_Level
                if lvl == Target_Lvl_Idx && col == Target_Col_Idx
                    set(Bin_Handles(lvl, col), 'FaceColor', [1, 0.5, 0]); % Orange (Target)
                else
                    set(Bin_Handles(lvl, col), 'FaceColor', [0.2, 0.4, 0.8]); % Blue (Normal)
                end
            end
        end
        
        % 3.2 ROTATION TIME AND DISTANCE CALCULATION
        Current_Pos = Bin_Positions(Target_Col_Idx);
        
        % Calculate shortest path distance
        diff = s_access - Current_Pos;
        diff = mod(diff + Track_Perimeter/2, Track_Perimeter) - Track_Perimeter/2;
        Move_Dist = diff; 
        total_dist_abs = abs(Move_Dist);
        
        % Calculate rotation time T_rot based on trapezoidal profile
        T_rot = calculate_T_rot(total_dist_abs, Max_Vel, Accel);
        
        T_cycle = T_rot;
        
        % Store Metrics
        Cycle_Times(pick_k) = T_cycle;
        Rotation_Times(pick_k) = T_rot;
        Total_Time_Active = Total_Time_Active + T_cycle;
        
        % 3.3 ANIMATION AND MOVEMENT
        t = 0; dt = 0.05; current_vel = 0; dist_covered = 0;
        direction = sign(Move_Dist);
        
        % Initialize required acceleration for the start of the dynamic loop
        req_acc = 0; 
        
        pause(0.5); 
        
        while dist_covered < total_dist_abs && ishandle(f)
            tic;
            
            % Physics: Calculate required acceleration
            if dist_covered < (current_vel^2)/(2*Accel); req_acc = -direction * Accel;
            elseif abs(current_vel) < Max_Vel; req_acc = direction * Accel;
            else; req_acc = 0; end
            
            current_vel = current_vel + (req_acc * dt);
            % Small tolerance to prevent overshoot and stop motion
            if (total_dist_abs - dist_covered) < 0.05; current_vel = direction * 0.1; end
            
            ds = current_vel * dt;
            dist_covered = dist_covered + abs(ds);
            
            % Update all bin positions
            Bin_Positions = mod(Bin_Positions + ds, Track_Perimeter);
            t = t + dt;
            
            % --- MOTOR SIZING LOGGING (Uses req_acc from this step) ---
            F_Total = (Friction_Coeff * Total_Mass * Gravity) + (Total_Mass * abs(req_acc));
            Power = F_Total * abs(current_vel);
            Torque = F_Total * R_drive;

            Current_Sim_Time = Current_Sim_Time + dt;
            Force_History = [Force_History, F_Total];
            Power_History = [Power_History, Power];
            Torque_History = [Torque_History, Torque];
            Total_Time_History = [Total_Time_History, Current_Sim_Time];
            
            % --- RENDER UPDATE ---
            for i = 1:System.N_Bins_Per_Level
                s = Bin_Positions(i);
                [bx, by, angle] = get_track_pos(s, Pole_Distance, Curve_Radius);
                
                verts_base = get_rotated_box_at_zero(bx, by, Bin.Length, Bin.Depth, Bin.Height, angle);
                h_verts_base = get_hanger_geometry(bx, by, Bin.Length, Bin.Depth, angle);
                
                for lvl = 1:System.Levels
                    z_bin_base = System.Base_Height + (lvl-1)*System.Level_Height;
                    
                    % Update Bin
                    verts_lvl = verts_base; 
                    verts_lvl(:,3) = verts_lvl(:,3) + z_bin_base + (Bin.Height/2);
                    faces_bin = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5];
                    set(Bin_Handles(lvl, i), 'Vertices', verts_lvl, 'Faces', faces_bin);
                    
                    % Update Hanger
                    z_track = z_bin_base + Bin.Height + 0.05;
                    h_verts_lvl = h_verts_base;
                    h_verts_lvl(:,3) = h_verts_lvl(:,3) + z_bin_base + Bin.Height;
                    set(Hanger_Handles(lvl, i), 'Vertices', h_verts_lvl, ...
                        'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6]);
                end
            end
            
            % Text Update (Uses the calculated F_Total/Power)
            set(h_text, 'String', sprintf('Target: Lvl %d, Bin %d\nAction: Rotating (Time: %.2f s)\nLoad: %d Bins', Target_Lvl_Idx, Target_Col_Idx, T_cycle, Total_Bins));
            set(h_perf, 'String', sprintf('--- PICK %d of %d ---\nMass: %.0f kg\nForce: %.0f N\nPower: %.0f W\nTime Elapsed: %.2f s', pick_k, N_PICKS, Total_Mass, F_Total, Power, t));
            
            drawnow;
            pause(dt - toc);
        end
        
        % Final Pick Completion
        set(h_text, 'String', sprintf('Target: Lvl %d, Bin %d\nAction: Pick Complete\nLoad: %d Bins', Target_Lvl_Idx, Target_Col_Idx, Total_Bins));
        drawnow;
        pause(0.5); 
    end
    
    %% 4. PERFORMANCE ANALYSIS REPORT
    
    fprintf('\n========================================\n');
    fprintf('   CAROUSEL SYSTEM PERFORMANCE REPORT\n');
    fprintf('========================================\n');
    
    Mean_Cycle_Time = mean(Cycle_Times);
    Throughput = 60 / Mean_Cycle_Time; % Picks per minute
    
    % Total time the system was studied (including idle time between picks)
    Total_Time_Studied = sum(Cycle_Times) + (N_PICKS * 0.5);
    System_Utilization = Total_Time_Active / Total_Time_Studied;
    
    fprintf('Total Picks Simulated: %d\n', N_PICKS);
    fprintf('Mean Rotation Distance: %.2f m\n', mean(abs(Rotation_Times))); % Use abs(Rotation_Times) as it holds distance
    fprintf('----------------------------------------\n');
    fprintf('A. Throughput Metrics\n');
    fprintf('   Average Cycle Time (Mean T_rot): %.2f seconds\n', Mean_Cycle_Time);
    fprintf('   Throughput (Picks/min): %.2f\n', Throughput);
    fprintf('----------------------------------------\n');
    fprintf('B. Efficiency Metrics\n');
    fprintf('   Total Active Time: %.2f seconds\n', Total_Time_Active);
    fprintf('   System Utilization (Active/Total): %.2f%%\n', System_Utilization * 100);
    fprintf('========================================\n');
    
    % --- MOTOR SIZING PLOTS ---
    if ~isempty(Total_Time_History) % Only plot if data was logged
        figure('Name', 'Motor Sizing Plots');

        % Plot 1: Required Torque
        subplot(2, 1, 1);
        plot(Total_Time_History, Torque_History, 'r-');
        xlabel('Time (s)');
        ylabel('Required Torque (Nm)');
        title(sprintf('Motor Torque Requirement (Peak: %.2f Nm)', max(Torque_History)));
        grid on;

        % Plot 2: Required Power
        subplot(2, 1, 2);
        plot(Total_Time_History, Power_History, 'b-');
        xlabel('Time (s)');
        ylabel('Required Power (W)');
        title(sprintf('Motor Power Requirement (Peak: %.2f W)', max(Power_History)));
        grid on;
    end
    
end

%% HELPER FUNCTIONS

% ... [The HELPER FUNCTIONS remain unchanged from the last corrected version] ...

function T_rot = calculate_T_rot(D, V_max, A)
    % Calculates rotation time based on trapezoidal velocity profile.
    % D: distance to move (m)
    % V_max: maximum linear velocity (m/s)
    % A: acceleration (m/s^2)
    
    % Distance covered during acceleration to V_max
    D_accel = (V_max^2) / (2 * A); 
    
    if D <= 2 * D_accel
        % Triangular Profile (Does not reach V_max)
        % D = 2 * 0.5 * A * t_accel^2 -> t_accel = sqrt(D/A)
        T_rot = 2 * sqrt(D / (2 * A));
    else
        % Trapezoidal Profile (Reaches V_max)
        % Time to accelerate/decelerate
        T_accel = V_max / A; 
        % Time at constant velocity
        D_const = D - 2 * D_accel;
        T_const = D_const / V_max;
        
        T_rot = 2 * T_accel + T_const;
    end
end
function draw_shell_custom(Origin, L, W, H, Color, Style, Tag)
    % Draws a box with specific style (Wireframe vs Opaque)
    x = Origin(1); y = Origin(2); z = Origin(3);
    
    if strcmp(Style, 'Wireframe')
        % Draw thick edges, transparent faces
        FaceAlpha = 0.05;
        EdgeAlpha = 0.8; % High visibility for edges (ALPHA VALUE)
        LineWidth = 2.0; % Thick lines (NOT an ALPHA VALUE)
    else
        % Opaque
        FaceAlpha = 1.0;
        EdgeAlpha = 1.0;
        LineWidth = 0.5;
    end
    
    % Define vertices (used for the Housing section)
    verts = [x, y, z; x+L, y, z; x+L, y+W, z; x, y+W, z; ...
             x, y, z+H; x+L, y, z+H; x+L, y+W, z+H; x, y+W, z+H];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    
    if strcmp(Tag, 'Van')
        % Draw individual walls except rear
        % Floor: z min
        patch([x, x+L, x+L, x], [y, y, y+W, y+W], [z, z, z, z], Color, 'FaceAlpha', FaceAlpha, 'EdgeAlpha', EdgeAlpha, 'LineWidth', LineWidth);
        % Roof: z max
        patch([x, x+L, x+L, x], [y, y, y+W, y+W], [z+H, z+H, z+H, z+H], Color, 'FaceAlpha', FaceAlpha, 'EdgeAlpha', EdgeAlpha, 'LineWidth', LineWidth);
        % Side 1 (y min)
        patch([x, x+L, x+L, x], [y, y, y, y], [z, z, z+H, z+H], Color, 'FaceAlpha', FaceAlpha, 'EdgeAlpha', EdgeAlpha, 'LineWidth', LineWidth);
        % Side 2 (y max)
        patch([x, x+L, x+L, x], [y+W, y+W, y+W, y+W], [z, z, z+H, z+H], Color, 'FaceAlpha', FaceAlpha, 'EdgeAlpha', EdgeAlpha, 'LineWidth', LineWidth);
        % Front (x max)
        patch([x+L, x+L, x+L, x+L], [y, y+W, y+W, y], [z, z, z+H, z+H], Color, 'FaceAlpha', FaceAlpha, 'EdgeAlpha', EdgeAlpha, 'LineWidth', LineWidth);
        
    else
        % Housing - Draw full box
         patch('Vertices', verts, 'Faces', faces, 'FaceColor', Color, ...
               'FaceAlpha', FaceAlpha, 'EdgeAlpha', EdgeAlpha, 'LineWidth', LineWidth);
    end
end
function draw_cylinder(cx, cy, base_z, r, h, color)
    [X,Y,Z] = cylinder(r, 20);
    X = X + cx;
    Y = Y + cy;
    Z = Z * h + base_z;
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1);
end
function [x, y, angle] = get_track_pos(s, L, R)
    Perimeter = 2*L + 2*pi*R; s = mod(s, Perimeter);
    % Map: Top Straight -> Right Curve -> Bottom Straight -> Left Curve
    if s < L; x = s; y = R; angle = 0;
    elseif s < (L + pi*R); dist = s - L; theta = dist / R; x = L + R*sin(theta); y = R*cos(theta); angle = -theta;
    elseif s < (2*L + pi*R); dist = s - (L + pi*R); x = L - dist; y = -R; angle = pi;
    else; dist = s - (2*L + pi*R); theta = dist / R; x = -R*sin(theta); y = -R*cos(theta); angle = pi - theta;
    end
end
function verts = get_rotated_box_at_zero(cx, cy, L, D, H, theta)
    dx = [-L/2, L/2, L/2, -L/2, -L/2, L/2, L/2, -L/2];
    dy = [-D/2, -D/2, D/2, D/2, -D/2, -D/2, D/2, D/2];
    dz = [-H/2, -H/2, -H/2, -H/2, H/2, H/2, H/2, H/2];
    x_rot = dx*cos(theta) - dy*sin(theta); y_rot = dx*sin(theta) + dy*cos(theta);
    verts = [x_rot' + cx, y_rot' + cy, dz'];
end
function verts = get_hanger_geometry(cx, cy, BinL, BinD, theta)
    HL = 0.05; HW = BinD * 0.8; HH = 0.05;
    
    dx = [-HL/2, HL/2, HL/2, -HL/2, -HL/2, HL/2, HL/2, -HL/2];
    dy = [-HW/2, -HW/2, HW/2, HW/2, -HW/2, -HW/2, HW/2, HW/2];
    dz = [0, 0, 0, 0, HH, HH, HH, HH];
    
    x_rot = dx*cos(theta) - dy*sin(theta); y_rot = dx*sin(theta) + dy*cos(theta);
    verts = [x_rot' + cx, y_rot' + cy, dz'];
end