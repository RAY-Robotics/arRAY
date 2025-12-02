function RAY_Carousel_Simulation_Final_v4()
    % RAY_Carousel_Simulation_Final_v4
    % - 5 Stacked Levels
    % - "Wireframe" mode for Van and Housing to see inside
    % - Added Housing structure (White)
    % - Added Central Rods, Sprockets, and Bin Hangers
    % - Single Box selection logic
    
    clc; clear; close all;

    %% 1. SYSTEM CONFIGURATION
    
    % --- VIEW STYLE PARAMETER ---
    % Options: 'Wireframe' (See-through edges) or 'Opaque' (Solid walls)
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
    
    % Camera View (Looking into rear)
    view([-20, 25]); 
    
    % --- 2.1 VAN EXTERIOR (Black) ---
    Van.L = 3.8; Van.W = 2.2; Van.H = 2.8;
    Van_Origin = [-0.6, -1.1, 0]; % Center Y around 0
    
    % Helper function to draw the shell based on style
    draw_shell_custom(Van_Origin, Van.L, Van.W, Van.H, [0 0 0], Shell_Style, 'Van');

    % --- 2.2 SYSTEM HOUSING (White) ---
    % Surrounds the track tightly
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
    % Draw Vertical Drive Shafts at the center of curves
    % Left Curve Center (Rear): roughly X=0, Y=0 (based on track math)
    % Actually, track math defines Left Curve Center at (-Radius, 0) relative to straight start?
    % Let's use the track helper to find centers.
    % Based on `get_track_pos`:
    % Left Curve Center is (0,0) in our logic for the rear curve? 
    % No, logic was: Start (0,R). Right Center (L,0). Left Center (0,0).
    % So Centers are at (0,0) and (Pole_Distance, 0).
    
    Shaft_Height = House.H - 0.2;
    draw_cylinder(0, 0, 0, 0.05, Shaft_Height, [0.4 0.4 0.4]); % Rear Shaft
    draw_cylinder(Pole_Distance, 0, 0, 0.05, Shaft_Height, [0.4 0.4 0.4]); % Front Shaft
    
    % Draw Sprockets at each level
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
            % Bin Body
            c_base = [0.2, 0.4, 0.8]; 
            Bin_Handles(lvl, i) = patch('Vertices', [], 'Faces', [], ...
                'FaceColor', c_base, 'EdgeColor', 'k', 'FaceAlpha', 1.0);
            
            % Hanger (Connecting Piece)
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
    
    Max_Vel = 0.8; Accel = 0.5; Friction_Coeff = 0.08; Gravity = 9.81;
    
    % Access Point: Middle of Rear Curve (Center 0,0, Angle pi)
    % In our get_track_pos logic: Left Curve is the last segment.
    % Center is (0,0). Angle goes from pi to 0. Apex is at angle pi/2? 
    % Let's hardcode the s_access based on geometry:
    % It is the point farthest negative X. 
    % That corresponds to Left Curve at theta = pi/2.
    s_access = (2 * Pole_Distance) + (1.5 * pi * Curve_Radius);
    s_access = mod(s_access, Track_Perimeter);
    
    while ishandle(f)
        % Target Selection
        Target_Col_Idx = randi(System.N_Bins_Per_Level);
        Target_Lvl_Idx = randi(System.Levels);
        
        % Color Highlighting
        for lvl = 1:System.Levels
            for col = 1:System.N_Bins_Per_Level
                if lvl == Target_Lvl_Idx && col == Target_Col_Idx
                    set(Bin_Handles(lvl, col), 'FaceColor', [1, 0.5, 0]); % Orange
                else
                    set(Bin_Handles(lvl, col), 'FaceColor', [0.2, 0.4, 0.8]); % Blue
                end
            end
        end
        
        % Move Logic
        Current_Pos = Bin_Positions(Target_Col_Idx);
        diff = s_access - Current_Pos;
        diff = mod(diff + Track_Perimeter/2, Track_Perimeter) - Track_Perimeter/2;
        Move_Dist = diff; 
        
        % Animation
        t = 0; dt = 0.05; current_vel = 0; dist_covered = 0;
        total_dist_abs = abs(Move_Dist); direction = sign(Move_Dist);
        
        pause(0.5); 
        
        while dist_covered < total_dist_abs && ishandle(f)
            tic;
            % Physics
            if dist_covered < (current_vel^2)/(2*Accel); req_acc = -direction * Accel;
            elseif abs(current_vel) < Max_Vel; req_acc = direction * Accel;
            else; req_acc = 0; end
            
            current_vel = current_vel + (req_acc * dt);
            if (total_dist_abs - dist_covered) < 0.05; current_vel = direction * 0.1; end
            
            ds = current_vel * dt;
            dist_covered = dist_covered + abs(ds);
            Bin_Positions = mod(Bin_Positions + ds, Track_Perimeter);
            
            % --- RENDER UPDATE ---
            for i = 1:System.N_Bins_Per_Level
                s = Bin_Positions(i);
                [bx, by, angle] = get_track_pos(s, Pole_Distance, Curve_Radius);
                
                % 1. Bin Geometry (Rotated)
                verts_base = get_rotated_box_at_zero(bx, by, Bin.Length, Bin.Depth, Bin.Height, angle);
                
                % 2. Hanger Geometry
                % Connects (bx, by) at track height to center of bin
                % Simple bar geometry
                h_verts_base = get_hanger_geometry(bx, by, Bin.Length, Bin.Depth, angle);

                for lvl = 1:System.Levels
                    z_bin_base = System.Base_Height + (lvl-1)*System.Level_Height;
                    
                    % Update Bin
                    verts_lvl = verts_base; 
                    verts_lvl(:,3) = verts_lvl(:,3) + z_bin_base + (Bin.Height/2);
                    % Open top faces
                    faces_bin = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5];
                    set(Bin_Handles(lvl, i), 'Vertices', verts_lvl, 'Faces', faces_bin);
                    
                    % Update Hanger
                    % Hanger sits on top of bin, connects to track level above
                    z_track = z_bin_base + Bin.Height + 0.05;
                    h_verts_lvl = h_verts_base;
                    h_verts_lvl(:,3) = h_verts_lvl(:,3) + z_bin_base + Bin.Height;
                    
                    % Simple faces for hanger bar
                    set(Hanger_Handles(lvl, i), 'Vertices', h_verts_lvl, ...
                        'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6]);
                end
            end
            
            % Text Update
            F_Total = (Friction_Coeff * Total_Mass * Gravity) + (Total_Mass * abs(req_acc));
            Power = F_Total * abs(current_vel);
            set(h_text, 'String', sprintf('Target: Lvl %d, Bin %d\nAction: Rotating\nLoad: %d Bins', Target_Lvl_Idx, Target_Col_Idx, Total_Bins));
            set(h_perf, 'String', sprintf('Mass: %.0f kg\nForce: %.0f N\nPower: %.0f W', Total_Mass, F_Total, Power));
            
            drawnow;
            pause(dt - toc);
        end
        pause(1.0);
    end
end

%% HELPER FUNCTIONS

function draw_shell_custom(Origin, L, W, H, Color, Style, Tag)
    % Draws a box with specific style (Wireframe vs Opaque)
    x = Origin(1); y = Origin(2); z = Origin(3);
    
    if strcmp(Style, 'Wireframe')
        % Draw thick edges, transparent faces
        FaceAlpha = 0.05;
        EdgeAlpha = 0.8; % High visibility for edges
        LineWidth = 2.0; % Thick lines
    else
        % Opaque
        FaceAlpha = 1.0;
        EdgeAlpha = 1.0;
        LineWidth = 0.5;
    end
    
    % Define vertices
    verts = [x, y, z; x+L, y, z; x+L, y+W, z; x, y+W, z; ...
             x, y, z+H; x+L, y, z+H; x+L, y+W, z+H; x, y+W, z+H];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    
    % If Van, remove rear face (Face 6: 2 3 7 6 is Right? No.)
    % Faces: 1=Bottom, 2=Top, 3=Front, 4=Back(Right?), 5=Left, 6=Right
    % Let's explicitly define walls to be sure which one to remove.
    
    if strcmp(Tag, 'Van')
        % Draw individual walls except rear
        % Rear of system is -X direction. 
        % Van Origin is -X. Front of van is +X.
        % So Rear Wall is the face at x (min).
        
        % Floor, Roof, Side1, Side2, Front(Cab)
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
    % Creates a small bracket on top of the bin
    % Size: Small bar, width of bin depth, thin in L
    HL = 0.05; HW = BinD * 0.8; HH = 0.05;
    
    dx = [-HL/2, HL/2, HL/2, -HL/2, -HL/2, HL/2, HL/2, -HL/2];
    dy = [-HW/2, -HW/2, HW/2, HW/2, -HW/2, -HW/2, HW/2, HW/2];
    dz = [0, 0, 0, 0, HH, HH, HH, HH];
    
    x_rot = dx*cos(theta) - dy*sin(theta); y_rot = dx*sin(theta) + dy*cos(theta);
    verts = [x_rot' + cx, y_rot' + cy, dz'];
end