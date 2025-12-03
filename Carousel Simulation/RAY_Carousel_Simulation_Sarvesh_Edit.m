function RAY_Delivery_Robot_Simulation_v25()
    % RAY_Delivery_Robot_Simulation_v25
    % - INCLUDES: Motor Sizing and Performance Analysis.
    
    clc; clear; close all;
    
    %% 1. SYSTEM CONFIGURATION
    Shell_Style = 'Wireframe'; 
    
    % --- Dimensions ---
    System.Length = 2.4;        
    System.Levels = 4;          
    System.Level_Height = 0.40;
    System.Base_Height = 0.20;   
    
    % Bin Dimensions
    Bin.Length = 0.30; Bin.Depth  = 0.40; Bin.Height = 0.30; 
    Bin.Mass   = 10.0; 
    Avg_Pkgs_Per_Bin = 7; 
    
    % Track Geometry
    System.Track_Width = 0.9;   
    Curve_Radius = System.Track_Width / 2;                
    Pole_Distance = System.Length - System.Track_Width;   
    Track_Perimeter = (2 * Pole_Distance) + (2 * pi * Curve_Radius);
    System.N_Bins_Per_Level = 12;
    
    % Motor Sizing Constant
    R_drive = Curve_Radius * 0.9; % Effective radius of the drive sprocket
    
    % Logic State: Tracks if a bin is IN the carousel (1) or OUT (0)
    System.Bin_Active = true(System.Levels, System.N_Bins_Per_Level);
    
    % Physics & Metrics
    Total_Bins = System.N_Bins_Per_Level * System.Levels;
    Total_Mass = Total_Bins * Bin.Mass;
    Total_Expected_Pkgs = Total_Bins * Avg_Pkgs_Per_Bin;
    
    %% 2. GRAPHICS SETUP
    f = figure('Name', 'RAY: Last-Mile Fleet Sim (v25)', 'Color', 'w', 'NumberTitle', 'off');
    set(f, 'Position', [50, 50, 1200, 800]);
    axis equal; grid on; hold on;
    view([-130, 25]); 
    
    % --- 2.1 SYSTEM BOUNDS ---
    Track_Min_X = -Curve_Radius; 
    
    % Van & Housing
    Housing_Padding = 0.30; 
    House.L = (Track_Perimeter/2) * 0.8; 
    House.W = System.Track_Width + (2 * Housing_Padding);
    House.H = (System.Levels * System.Level_Height) + System.Base_Height + 0.1;
    
    Van.L = 3.8; Van.W = 2.4; Van.H = 2.1;
    Van_Origin = [0, -Van.W/2, 0]; 
    
    Wall_Padding = 0.05; 
    House_Origin_X = (Van.L - Wall_Padding) - System.Length - 0.2;
    House_Origin_Y = (Van_Origin(2) + Wall_Padding);
    
    Track_World_Offset_X = House_Origin_X + Housing_Padding + 0.4;
    Track_World_Offset_Y = House_Origin_Y + Housing_Padding + 0.4;
    
    draw_shell_custom([House_Origin_X, House_Origin_Y, 0], System.Length+0.6, House.W, House.H, [0.9 0.9 0.9], Shell_Style, 'Housing');
    draw_shell_custom(Van_Origin, Van.L, Van.W, Van.H, [0.1 0.1 0.1], Shell_Style, 'Van'); 
    
    % Adjust View Limits
    xlim([-2.0, Van.L+0.5]); ylim([-Van.W/2 - 0.5, Van.W/2 + 0.5]); zlim([0, Van.H+0.5]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % --- 2.3 MECHANISM: SPROCKETS ---
    Shaft_Height = House.H - 0.1;
    for shaft_x = [0, Pole_Distance]
        wx = shaft_x + Track_World_Offset_X; wy = 0 + Track_World_Offset_Y;
        draw_solid_cylinder(wx, wy, 0, 0.05, Shaft_Height, [0.4 0.4 0.4]);
        for k = 1:System.Levels
            h_lvl = System.Base_Height + (k-1)*System.Level_Height + Bin.Height + 0.05; 
            draw_solid_cylinder(wx, wy, h_lvl, Curve_Radius*0.9, 0.03, [0.7 0.7 0.7]);
        end
    end
    
    % --- 2.4 ROBOT & FLEET SETUP ---
    Robot_Base_X = Track_Min_X - 0.50 + Track_World_Offset_X; 
    Robot_Base_Y = 0 + Track_World_Offset_Y;
    
    [h_Robot.SubBase, h_Robot.Rotator, h_Robot.RotatorTop] = init_robot_base_graphics(Robot_Base_X, Robot_Base_Y);
    h_Robot.Mast = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k');
    h_Robot.Carriage = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.9 0.1 0.1], 'EdgeColor', 'k'); 
    h_Robot.Bracket = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k'); 
    h_Robot.Rails   = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none'); 
    
    Robot_State.Z = System.Base_Height;
    Robot_State.Angle = pi/2; 
    Robot_State.Extension = 0; 
    update_robot_geometry(h_Robot, Robot_Base_X, Robot_Base_Y, Robot_State.Z, Robot_State.Angle, Robot_State.Extension, Bin);
    
    % --- DELIVERY FLEET SETUP ---
    Fleet_Size = 3;
    Bot_Width = 0.5; Bot_Length = 0.6; Bot_Height = 0.25;
    
    % --- FIX: CENTERING ---
    % Load Zone X must align with Robot_Base_X so the arm (at pi/2) hits the center.
    Load_Zone_X = Robot_Base_X; 
    Load_Zone_Y = Robot_Base_Y + 1.2; 
    Exit_X = -1.5; 
    
    Pod_Spacing = 0.8;
    Pod_Handles = gobjects(1, Fleet_Size);
    Pod_Locs = zeros(Fleet_Size, 2);
    
    for i = 1:Fleet_Size
        px = Load_Zone_X + (i-1)*Pod_Spacing + 0.8; 
        py = Load_Zone_Y; 
        Pod_Locs(i,:) = [px, py];
        Pod_Handles(i) = patch([px-0.25, px+0.25, px+0.25, px-0.25], ...
                               [py-0.25, py-0.25, py+0.25, py+0.25], ...
                               [0.01, 0.01, 0.01, 0.01], [0 1 0], 'FaceAlpha', 0.5);
    end
    
    % Initialize Bots
    Mobile_Bots = repmat(struct('h_Body', [], 'h_Wheels', [], 'X', 0, 'Y', 0, 'HasBin', false, 'BinHandle', [], 'TargetLvl', 0, 'TargetCol', 0), 1, Fleet_Size);
    
    for i = 1:Fleet_Size
        Mobile_Bots(i).X = Pod_Locs(i,1);
        Mobile_Bots(i).Y = Pod_Locs(i,2);
        Mobile_Bots(i).HasBin = false; 
        Mobile_Bots(i).BinHandle = [];
        [Mobile_Bots(i).h_Body, Mobile_Bots(i).h_Wheels] = init_delivery_robot(Mobile_Bots(i).X, Mobile_Bots(i).Y, Bot_Length, Bot_Width, Bot_Height);
    end
    
    % --- 2.5 INITIALIZE BINS ---
    Bin_Handles = gobjects(System.Levels, System.N_Bins_Per_Level);
    Hanger_Handles = gobjects(System.Levels, System.N_Bins_Per_Level);
    Chain_Handles = gobjects(System.Levels, 1);
    Bin_Positions = linspace(0, Track_Perimeter, System.N_Bins_Per_Level + 1); Bin_Positions(end) = [];
    
    for lvl = 1:System.Levels
        Chain_Handles(lvl) = plot3(nan, nan, nan, 'Color', [0.2 0.2 0.2], 'LineWidth', 3);
        for i = 1:System.N_Bins_Per_Level
            Bin_Handles(lvl, i) = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.2, 0.4, 0.8], 'EdgeColor', 'k', 'FaceAlpha', 1.0);
            Hanger_Handles(lvl, i) = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none'); 
        end
    end
    
    % --- STATUS DISPLAY ---
    h_text = text(0, 0, Van.H+0.6, 'Status: System Init', 'FontSize', 12, 'BackgroundColor', 'w', 'FontWeight', 'bold');
    h_perf = text(2.0, 0, Van.H+0.4, sprintf('Weight: %.0fkg\nPkgs: %d\nPower: 0W', Total_Mass, Total_Expected_Pkgs), 'FontSize', 10, 'BackgroundColor', 'w');
    
    %% 3. SIMULATION LOOP (Batched)
    
    % Physics Constants
    Max_Vel = 0.8; Accel = 0.5; Friction_Coeff = 0.08; Gravity = 9.81;
    Robot_VZ_Max = 0.6; 
    s_access = (2 * Pole_Distance) + (1.5 * pi * Curve_Radius); 
    s_access = mod(s_access, Track_Perimeter);
    
    % --- PERFORMANCE/MOTOR SIZING LOGGING SETUP ---
    N_PICKS = Fleet_Size * 2; % One export and one import per robot
    Cycle_Times = zeros(1, N_PICKS);
    Total_Time_Active = 0;
    
    Total_Time_History = [];
    Force_History = [];
    Power_History = [];
    Torque_History = [];
    Current_Sim_Time = 0;
    
    
    % Initial Carousel Draw
    base_coords = update_carousel_graphics(System, Bin_Positions, Pole_Distance, Curve_Radius, Track_World_Offset_X, Track_World_Offset_Y, Bin, Bin_Handles, Hanger_Handles, Chain_Handles);
    
    pick_counter = 0; % Track total carousel movements
    
    while ishandle(f)
        
        %% --- PHASE 1: EXPORT (Carousel -> Robot -> Drive Off) ---
        for rob_idx = 1:Fleet_Size
            if ~ishandle(f); break; end
            pick_counter = pick_counter + 1;
            
            % A. Call Robot
            set(Mobile_Bots(rob_idx).h_Body, 'Visible', 'on');
            for w=1:6; set(Mobile_Bots(rob_idx).h_Wheels(w), 'Visible', 'on'); end
            
            set(h_text, 'String', sprintf('Status: Robot %d approaching Loading Zone...', rob_idx));
            animate_delivery_drive(Mobile_Bots(rob_idx), Load_Zone_X, Load_Zone_Y, Bot_Length, Bot_Width, Bot_Height, Bin, 30);
            Mobile_Bots(rob_idx).X = Load_Zone_X; Mobile_Bots(rob_idx).Y = Load_Zone_Y;
            
            % B. Carousel Prep (Spin & Elevate)
            valid_bins = find(System.Bin_Active);
            % Handle case where all bins have been exported
            if isempty(valid_bins)
                fprintf('WARNING: All bins have been exported. Skipping export phase.\n');
                break; 
            end
            
            idx_choice = valid_bins(randi(length(valid_bins)));
            [Target_Lvl, Target_Col] = ind2sub(size(System.Bin_Active), idx_choice);
            Target_Bin_Handle = Bin_Handles(Target_Lvl, Target_Col);
            
            set(Target_Bin_Handle, 'FaceColor', [1, 0.5, 0]);
            
            Mobile_Bots(rob_idx).TargetLvl = Target_Lvl;
            Mobile_Bots(rob_idx).TargetCol = Target_Col;
            Current_Pos = Bin_Positions(Target_Col);
            diff = s_access - Current_Pos;
            Move_Dist = mod(diff + Track_Perimeter/2, Track_Perimeter) - Track_Perimeter/2;
            
            set(h_text, 'String', 'Status: Carousel Spinning / Elevator Aligning...');
            
            % Calculate T_rot *before* animation starts for logging
            T_rot = calculate_T_rot(abs(Move_Dist), Max_Vel, Accel);
            Cycle_Times(pick_counter) = T_rot;
            Total_Time_Active = Total_Time_Active + T_rot;

            t = 0; dt = 0.04; current_vel = 0; dist_covered = 0;
            total_dist_abs = abs(Move_Dist); direction = sign(Move_Dist);
            carousel_done = false; robot_z_done = false;
            Target_Z = System.Base_Height + (Target_Lvl-1)*System.Level_Height;
            
            while (~carousel_done || ~robot_z_done) && ishandle(f)
                tic;
                
                % CAROUSEL PHYSICS
                if ~carousel_done
                    if dist_covered < (current_vel^2)/(2*Accel); req_acc = -direction * Accel;
                    elseif abs(current_vel) < Max_Vel; req_acc = direction * Accel;
                    else; req_acc = 0; end
                    current_vel = current_vel + (req_acc * dt);
                    if (total_dist_abs - dist_covered) < 0.02; current_vel = 0; req_acc = 0; carousel_done = true; end
                    ds = current_vel * dt; dist_covered = dist_covered + abs(ds);
                    Bin_Positions = mod(Bin_Positions + ds, Track_Perimeter);
                else
                    req_acc = 0; % Ensure zero acceleration when stopped
                end
                
                % ROBOT Z MOVEMENT
                z_err = Target_Z - Robot_State.Z;
                if abs(z_err) > 0.01; Robot_State.Z = Robot_State.Z + (sign(z_err) * Robot_VZ_Max * dt);
                else; Robot_State.Z = Target_Z; robot_z_done = true; end
                
                
                % --- MOTOR SIZING LOGGING ---
                F_Total = (Friction_Coeff * Total_Mass * Gravity) + (Total_Mass * abs(req_acc));
                Power = F_Total * abs(current_vel);
                Torque = F_Total * R_drive;
                
                Current_Sim_Time = Current_Sim_Time + dt;
                Force_History = [Force_History, F_Total];
                Power_History = [Power_History, Power];
                Torque_History = [Torque_History, Torque];
                Total_Time_History = [Total_Time_History, Current_Sim_Time];
                % ----------------------------
                
                set(h_perf, 'String', sprintf('Weight: %.0fkg\nPkgs: %d\nPower: %.0f W', Total_Mass, Total_Expected_Pkgs, Power));
                update_robot_geometry(h_Robot, Robot_Base_X, Robot_Base_Y, Robot_State.Z, Robot_State.Angle, Robot_State.Extension, Bin);
                base_coords = update_carousel_graphics(System, Bin_Positions, Pole_Distance, Curve_Radius, Track_World_Offset_X, Track_World_Offset_Y, Bin, Bin_Handles, Hanger_Handles, Chain_Handles);
                drawnow; pause(dt - toc);
            end
            set(h_perf, 'String', sprintf('Weight: %.0fkg\nPkgs: %d\nPower: 0 W', Total_Mass, Total_Expected_Pkgs));
            
            % C. Grabber Retrieve Sequence
            tx = base_coords(Target_Col, 1); ty = base_coords(Target_Col, 2);
            Dist_To_Bin = sqrt((tx - Robot_Base_X)^2 + (ty - Robot_Base_Y)^2);
            Req_Ext = Dist_To_Bin - 0.30;
            
            animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Angle', 0, 15); Robot_State.Angle = 0;
            animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', Req_Ext, 15); Robot_State.Extension = Req_Ext;
            
            % GRAB BIN: Mark Inactive
            System.Bin_Active(Target_Lvl, Target_Col) = false;
            
            Offset = base_coords(Target_Col, 3) - Robot_State.Angle;
            animate_robot_move_with_bin(h_Robot, Target_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', 0.05, 15, Offset); Robot_State.Extension = 0.05;
            
            % D. Deliver to Robot
            animate_robot_move_with_bin(h_Robot, Target_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Angle', pi/2, 25, Offset); Robot_State.Angle = pi/2;
            
            % Calculate Bot Surface Z
            Bot_Surface_Z = 0.24; 
            
            animate_robot_move_with_bin(h_Robot, Target_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Z', Bot_Surface_Z, 20, Offset); Robot_State.Z = Bot_Surface_Z;
            
            Dist_To_Robot = sqrt((Load_Zone_X - Robot_Base_X)^2 + (Load_Zone_Y - Robot_Base_Y)^2);
            Req_Ext_Drop = Dist_To_Robot - 0.30; 
            
            animate_robot_move_with_bin(h_Robot, Target_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', Req_Ext_Drop, 20, Offset); 
            Robot_State.Extension = Req_Ext_Drop;
            
            % Drop Bin on Robot
            set(h_text, 'String', sprintf('Status: Loading Robot %d...', rob_idx));
            Mobile_Bots(rob_idx).HasBin = true;
            Mobile_Bots(rob_idx).BinHandle = Target_Bin_Handle;
            
            animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', 0, 15); Robot_State.Extension = 0;
            
            % F. Robot Drives Away & Disappears
            set(h_text, 'String', sprintf('Status: Robot %d departing...', rob_idx));
            animate_delivery_drive(Mobile_Bots(rob_idx), Exit_X, Load_Zone_Y, Bot_Length, Bot_Width, Bot_Height, Bin, 40);
            Mobile_Bots(rob_idx).X = Exit_X;
            
            % Hide Robot
            set(Mobile_Bots(rob_idx).h_Body, 'Visible', 'off');
            for w=1:6; set(Mobile_Bots(rob_idx).h_Wheels(w), 'Visible', 'off'); end
            if ishandle(Mobile_Bots(rob_idx).BinHandle); set(Mobile_Bots(rob_idx).BinHandle, 'Visible', 'off'); end
            
            pause(0.2);
            Current_Sim_Time = Current_Sim_Time + 0.2; % Account for the pause
        end
        
        %% --- DELAY ---
        set(h_text, 'String', 'Status: Fleet Deployed. Waiting...');
        pause(1.0); 
        Current_Sim_Time = Current_Sim_Time + 1.0; % Account for the pause
        
        %% --- PHASE 2: IMPORT (Return -> Grabber -> Carousel) ---
        set(h_text, 'String', 'Status: Initiating RETURN CYCLE.');
        
        % FIX: Loop Backwards so outermost robot moves first
        for i = 1:Fleet_Size
             if ~ishandle(f); break; end
             
             rob_idx = Fleet_Size - i + 1; 
             Target_Pod_Loc = Pod_Locs(Fleet_Size - i + 1, :);
             
             if ~Mobile_Bots(rob_idx).TargetLvl
                 % Robot never exported a bin, skip return cycle
                 continue;
             end
             
             % A. Robot Reappears
             set(h_text, 'String', sprintf('Status: Robot %d Returning...', rob_idx));
             
             set(Mobile_Bots(rob_idx).h_Body, 'Visible', 'on');
             for w=1:6; set(Mobile_Bots(rob_idx).h_Wheels(w), 'Visible', 'on'); end
             if ishandle(Mobile_Bots(rob_idx).BinHandle); set(Mobile_Bots(rob_idx).BinHandle, 'Visible', 'on'); end
             
             animate_delivery_drive(Mobile_Bots(rob_idx), Load_Zone_X, Load_Zone_Y, Bot_Length, Bot_Width, Bot_Height, Bin, 40);
             Mobile_Bots(rob_idx).X = Load_Zone_X;
             
             % B. Grabber Picks Bin
             Dist_To_Robot = sqrt((Load_Zone_X - Robot_Base_X)^2 + (Load_Zone_Y - Robot_Base_Y)^2);
             Req_Ext_Pickup = Dist_To_Robot - 0.30;
             Bot_Surface_Z = 0.24; 
             
             % --- FIX: ROTATE FIRST, THEN MOVE Z ---
             animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Angle', pi/2, 15); Robot_State.Angle = pi/2;
             animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Z', Bot_Surface_Z, 15); Robot_State.Z = Bot_Surface_Z;
             
             animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', Req_Ext_Pickup, 15); Robot_State.Extension = Req_Ext_Pickup;
             
             Current_Bin_Handle = Mobile_Bots(rob_idx).BinHandle;
             Mobile_Bots(rob_idx).HasBin = false;
             Mobile_Bots(rob_idx).BinHandle = [];
             
             Bin_Offset = -pi/2; 
             
             set(h_text, 'String', 'Status: Retrieving Bin from Robot...');
             animate_robot_move_with_bin(h_Robot, Current_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', 0.05, 15, Bin_Offset); Robot_State.Extension = 0.05;
             
             % C. Return to Carousel
             Refill_Col = Mobile_Bots(rob_idx).TargetCol;
             Refill_Lvl = Mobile_Bots(rob_idx).TargetLvl;
             Refill_Z = System.Base_Height + (Refill_Lvl-1)*System.Level_Height;
             
             set(h_text, 'String', 'Status: Aligning Carousel & Lifting Bin...');
             
             % Calculate Move
             Current_Pos = Bin_Positions(Refill_Col);
             diff = s_access - Current_Pos;
             Move_Dist = mod(diff + Track_Perimeter/2, Track_Perimeter) - Track_Perimeter/2;
             
             % Calculate T_rot *before* animation starts for logging
             T_rot = calculate_T_rot(abs(Move_Dist), Max_Vel, Accel);
             Cycle_Times(pick_counter) = Cycle_Times(pick_counter) + T_rot; % Accumulate time in the same pick slot
             Total_Time_Active = Total_Time_Active + T_rot;
             
             t = 0; current_vel = 0; dist_covered = 0;
             total_dist_abs = abs(Move_Dist); direction = sign(Move_Dist);
             carousel_done = false; robot_z_done = false;
             
             while (~carousel_done || ~robot_z_done) && ishandle(f)
                 tic;
                 
                 % CAROUSEL PHYSICS
                 if ~carousel_done
                     if dist_covered < (current_vel^2)/(2*Accel); req_acc = -direction * Accel;
                     elseif abs(current_vel) < Max_Vel; req_acc = direction * Accel;
                     else; req_acc = 0; end
                     current_vel = current_vel + (req_acc * dt);
                     if (total_dist_abs - dist_covered) < 0.02; current_vel = 0; req_acc = 0; carousel_done = true; end
                     ds = current_vel * dt; dist_covered = dist_covered + abs(ds);
                     Bin_Positions = mod(Bin_Positions + ds, Track_Perimeter);
                 else
                     req_acc = 0;
                 end
                 
                 % Elevate Arm
                 z_err = Refill_Z - Robot_State.Z;
                 if abs(z_err) > 0.01; Robot_State.Z = Robot_State.Z + (sign(z_err) * Robot_VZ_Max * dt);
                 else; Robot_State.Z = Refill_Z; robot_z_done = true; end
                 
                 % --- MOTOR SIZING LOGGING ---
                 F_Total = (Friction_Coeff * Total_Mass * Gravity) + (Total_Mass * abs(req_acc));
                 Power = F_Total * abs(current_vel);
                 Torque = F_Total * R_drive;
                
                 Current_Sim_Time = Current_Sim_Time + dt;
                 Force_History = [Force_History, F_Total];
                 Power_History = [Power_History, Power];
                 Torque_History = [Torque_History, Torque];
                 Total_Time_History = [Total_Time_History, Current_Sim_Time];
                 % ----------------------------
                 
                 set(h_perf, 'String', sprintf('Weight: %.0fkg\nPkgs: %d\nPower: %.0f W', Total_Mass, Total_Expected_Pkgs, Power));
                 update_robot_geometry(h_Robot, Robot_Base_X, Robot_Base_Y, Robot_State.Z, Robot_State.Angle, Robot_State.Extension, Bin);
                 base_coords = update_carousel_graphics(System, Bin_Positions, Pole_Distance, Curve_Radius, Track_World_Offset_X, Track_World_Offset_Y, Bin, Bin_Handles, Hanger_Handles, Chain_Handles);
                 
                 [cx, cy] = get_robot_carriage_center(Robot_Base_X, Robot_Base_Y, Robot_State.Angle, Robot_State.Extension);
                 bin_z = Robot_State.Z + Bin.Height/2;
                 Current_Bin_Angle = Robot_State.Angle + Bin_Offset;
                 verts = get_rotated_box_at_zero(cx, cy, Bin.Length, Bin.Depth, Bin.Height, Current_Bin_Angle);
                 verts(:,3) = verts(:,3) + bin_z;
                 set(Current_Bin_Handle, 'Vertices', verts);
                 drawnow; pause(dt - toc);
             end
             set(h_perf, 'String', sprintf('Weight: %.0fkg\nPkgs: %d\nPower: 0 W', Total_Mass, Total_Expected_Pkgs));
             set(h_text, 'String', 'Status: Placing Bin into Slot...');
             
             animate_robot_move_with_bin(h_Robot, Current_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Angle', 0, 20, Bin_Offset); Robot_State.Angle = 0;
             
             [bx, by, ~] = get_track_pos(Bin_Positions(Refill_Col), Pole_Distance, Curve_Radius);
             bx = bx + Track_World_Offset_X; by = by + Track_World_Offset_Y;
             Dist_To_Slot = sqrt((bx - Robot_Base_X)^2 + (by - Robot_Base_Y)^2);
             
             animate_robot_move_with_bin(h_Robot, Current_Bin_Handle, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', Dist_To_Slot-0.3, 15, Bin_Offset); Robot_State.Extension = Dist_To_Slot-0.3;
             
             System.Bin_Active(Refill_Lvl, Refill_Col) = true;
             set(Current_Bin_Handle, 'FaceColor', [0.2, 0.4, 0.8]);
             
             update_carousel_graphics(System, Bin_Positions, Pole_Distance, Curve_Radius, Track_World_Offset_X, Track_World_Offset_Y, Bin, Bin_Handles, Hanger_Handles, Chain_Handles);
             animate_robot_move(h_Robot, Robot_Base_X, Robot_Base_Y, Bin, Robot_State, 'Extension', 0, 15); Robot_State.Extension = 0;
             
             set(h_text, 'String', sprintf('Status: Robot %d Parking...', rob_idx));
             animate_delivery_drive(Mobile_Bots(rob_idx), Target_Pod_Loc(1), Target_Pod_Loc(2), Bot_Length, Bot_Width, Bot_Height, Bin, 30);
             Mobile_Bots(rob_idx).X = Target_Pod_Loc(1);
        end
        
        set(h_text, 'String', 'Status: Cycle Complete. Resetting...');
        pause(2.0);
        Current_Sim_Time = Current_Sim_Time + 2.0; % Account for the pause
        
        % BREAK after one full loop (Export + Import cycles)
        break; 
    end
    
    %% 4. PERFORMANCE ANALYSIS REPORT & MOTOR SIZING PLOTS
    
    % Only include non-zero cycle times (only movements, excluding failed exports)
    Valid_Cycle_Times = Cycle_Times(1:pick_counter);
    Total_Time_Active = sum(Valid_Cycle_Times);
    
    fprintf('\n========================================\n');
    fprintf('   CAROUSEL SYSTEM PERFORMANCE REPORT\n');
    fprintf('========================================\n');
    
    if isempty(Valid_Cycle_Times)
         fprintf('No valid carousel movements recorded for analysis.\n');
    else
        Mean_Cycle_Time = mean(Valid_Cycle_Times);
        Throughput = 60 / Mean_Cycle_Time; % Movements per minute
        
        % Total time the system was studied (including idle time)
        Total_Time_Studied = Current_Sim_Time;
        System_Utilization = Total_Time_Active / Total_Time_Studied;
        
        fprintf('Total Carousel Movements Simulated: %d\n', pick_counter);
        fprintf('----------------------------------------\n');
        fprintf('A. Throughput Metrics\n');
        fprintf('   Average Movement Time (Mean T_rot): %.2f seconds\n', Mean_Cycle_Time);
        fprintf('   Throughput (Movements/min): %.2f\n', Throughput);
        fprintf('----------------------------------------\n');
        fprintf('B. Efficiency Metrics\n');
        fprintf('   Total Active Time: %.2f seconds\n', Total_Time_Active);
        fprintf('   System Utilization (Active/Total): %.2f%%\n', System_Utilization * 100);
        fprintf('========================================\n');
    end

    % --- MOTOR SIZING PLOTS ---
    if ~isempty(Total_Time_History) 
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
%% --- HELPER FUNCTIONS ---

function T_rot = calculate_T_rot(D, V_max, A)
    % Calculates rotation time based on trapezoidal velocity profile.
    % D: distance to move (m)
    D_accel = (V_max^2) / (2 * A); 
    if D <= 2 * D_accel
        T_rot = 2 * sqrt(D / (2 * A));
    else
        T_accel = V_max / A; 
        D_const = D - 2 * D_accel;
        T_const = D_const / V_max;
        T_rot = 2 * T_accel + T_const;
    end
end
function animate_delivery_drive(BotStruct, tx, ty, L, W, H, Bin, Steps)
    sx = BotStruct.X; sy = BotStruct.Y;
    Wheel_R = 0.12; Body_Z_Min = 0.05; Body_Z_Max = Wheel_R * 2; 
    Body_Thick = Body_Z_Max - Body_Z_Min; Visual_Body_Width = W * 0.7; 
    Wheel_Thick = 0.05;
    
    for i = 1:Steps
        t = i/Steps;
        cx = sx + (tx - sx)*t; cy = sy + (ty - sy)*t;
        
        b_verts = get_box_at_pos(cx, cy, Body_Z_Min + Body_Thick/2, L, Visual_Body_Width, Body_Thick);
        set(BotStruct.h_Body, 'Vertices', b_verts);
        
        w_off_x = [-L/3, 0, L/3, -L/3, 0, L/3];
        w_dist = (Visual_Body_Width / 2) + (Wheel_Thick / 2); 
        w_off_y = [w_dist, w_dist, w_dist, -w_dist, -w_dist, -w_dist];
        
        for w = 1:6
            wx = cx + w_off_x(w); wy = cy + w_off_y(w);
            [X,Y,Z] = cylinder(Wheel_R, 15); 
            Z = Z * Wheel_Thick; 
            Xr = X; Yr = Z; Zr = Y + Wheel_R; 
            if w > 3; Yr = Yr - Wheel_Thick; end 
            set(BotStruct.h_Wheels(w), 'XData', Xr + wx, 'YData', Yr + wy, 'ZData', Zr);
        end
        
        if BotStruct.HasBin && ~isempty(BotStruct.BinHandle) 
            if all(ishandle(BotStruct.BinHandle))
                 bin_z_base = Body_Z_Max;
                 bin_verts = get_box_at_pos(cx, cy, bin_z_base + Bin.Height/2, Bin.Length, Bin.Depth, Bin.Height);
                 set(BotStruct.BinHandle, 'Vertices', bin_verts);
            end
        end
        drawnow; pause(0.01);
    end
end
function animate_robot_move(h_Robot, bx, by, Bin, StartState, Mode, TargetVal, Steps)
    for s = 1:Steps
        t = s/Steps; t_smooth = t*t*(3 - 2*t);
        CurrentState = StartState;
        if strcmp(Mode, 'Angle'); CurrentState.Angle = StartState.Angle + (TargetVal - StartState.Angle)*t_smooth;
        elseif strcmp(Mode, 'Z'); CurrentState.Z = StartState.Z + (TargetVal - StartState.Z)*t_smooth;
        elseif strcmp(Mode, 'Extension'); CurrentState.Extension = StartState.Extension + (TargetVal - StartState.Extension)*t_smooth;
        end
        update_robot_geometry(h_Robot, bx, by, CurrentState.Z, CurrentState.Angle, CurrentState.Extension, Bin);
        drawnow; pause(0.02);
    end
end
function animate_robot_move_with_bin(h_Robot, h_Bin, bx, by, Bin, StartState, Mode, TargetVal, Steps, AngleOffset)
    for s = 1:Steps
        t = s/Steps; t_smooth = t*t*(3 - 2*t);
        CurrentState = StartState;
        if strcmp(Mode, 'Angle'); CurrentState.Angle = StartState.Angle + (TargetVal - StartState.Angle)*t_smooth;
        elseif strcmp(Mode, 'Z'); CurrentState.Z = StartState.Z + (TargetVal - StartState.Z)*t_smooth; 
        elseif strcmp(Mode, 'Extension'); CurrentState.Extension = StartState.Extension + (TargetVal - StartState.Extension)*t_smooth;
        end
        update_robot_geometry(h_Robot, bx, by, CurrentState.Z, CurrentState.Angle, CurrentState.Extension, Bin);
        [cx, cy] = get_robot_carriage_center(bx, by, CurrentState.Angle, CurrentState.Extension);
        bin_z = CurrentState.Z + Bin.Height/2;
        Current_Bin_Angle = CurrentState.Angle + AngleOffset;
        verts = get_rotated_box_at_zero(cx, cy, Bin.Length, Bin.Depth, Bin.Height, Current_Bin_Angle);
        verts(:,3) = verts(:,3) + bin_z;
        set(h_Bin, 'Vertices', verts);
        drawnow; pause(0.02);
    end
end
function [h_body, h_wheels] = init_delivery_robot(x, y, L, W, H)
    Wheel_R = 0.12; 
    Body_Z_Min = 0.05; 
    Body_Z_Max = Wheel_R * 2; 
    Body_Thick = Body_Z_Max - Body_Z_Min;
    Visual_Body_Width = W * 0.7; 
    Wheel_Thick = 0.05;
    verts = get_box_at_pos(x, y, Body_Z_Min + Body_Thick/2, L, Visual_Body_Width, Body_Thick);
    h_body = patch('Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8], ...
                   'FaceColor', [0.9 0.9 0.1], 'EdgeColor', 'k');
                   
    h_wheels = gobjects(1,6);
    w_off_x = [-L/3, 0, L/3, -L/3, 0, L/3];
    w_dist = (Visual_Body_Width / 2) + (Wheel_Thick / 2);
    w_off_y = [w_dist, w_dist, w_dist, -w_dist, -w_dist, -w_dist];
    
    for i = 1:6
        wx = x + w_off_x(i); wy = y + w_off_y(i);
        [X,Y,Z] = cylinder(Wheel_R, 15); 
        Z = Z * Wheel_Thick; 
        Xr = X; Yr = Z; Zr = Y + Wheel_R; 
        if i > 3; Yr = Yr - Wheel_Thick; end
        h_wheels(i) = surf(Xr+wx, Yr+wy, Zr, 'FaceColor', 'k', 'EdgeColor', 'none');
    end
end
function verts = get_box_at_pos(cx, cy, cz, L, W, H)
    dx = [-L/2, L/2, L/2, -L/2, -L/2, L/2, L/2, -L/2];
    dy = [-W/2, -W/2, W/2, W/2, -W/2, -W/2, W/2, W/2];
    dz = [-H/2, -H/2, -H/2, -H/2, H/2, H/2, H/2, H/2];
    verts = [dx' + cx, dy' + cy, dz' + cz];
end
function [h_sub, h_rot_surf, h_rot_top] = init_robot_base_graphics(bx, by)
    Base_W = 0.6; Base_H = 0.04; 
    verts = get_rotated_box_at_zero(bx, by, Base_W, Base_W, Base_H, 0);
    verts(:,3) = verts(:,3) + Base_H/2;
    h_sub = patch('Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8], ...
                  'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');
    h_rot_surf = surf([],[],[], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
    h_rot_top = patch('XData', [], 'YData', [], 'ZData', [], 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
end
function [rx, ry] = get_robot_carriage_center(bx, by, angle, extension)
    Offset_Base = 0.30; Total_Dist = Offset_Base + extension;
    rx = bx + Total_Dist*cos(angle); ry = by + Total_Dist*sin(angle);
end
function update_robot_geometry(h_Robot, bx, by, z_carr, angle, extension, Bin)
    Rot_H = 0.10; Rot_R = 0.25; Base_Offset_Z = 0.04;
    [X,Y,Z] = cylinder(Rot_R, 30);
    X_rot = X*cos(angle) - Y*sin(angle) + bx; Y_rot = X*sin(angle) + Y*cos(angle) + by;
    Z_rot = Z*Rot_H + Base_Offset_Z;
    set(h_Robot.Rotator, 'XData', X_rot, 'YData', Y_rot, 'ZData', Z_rot);
    set(h_Robot.RotatorTop, 'XData', X_rot(2,:), 'YData', Y_rot(2,:), 'ZData', Z_rot(2,:));
    
    Mast_H = 1.8; Mast_W = 0.15; Mast_D = 0.08;
    mx = -0.15; my = 0; 
    rot_mx = mx*cos(angle) - my*sin(angle); rot_my = mx*sin(angle) + my*cos(angle);
    verts = get_rotated_box_at_zero(bx + rot_mx, by + rot_my, Mast_D, Mast_W, Mast_H, angle);
    verts(:,3) = verts(:,3) + Mast_H/2 + Base_Offset_Z + Rot_H;
    set(h_Robot.Mast, 'Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8]);
    
    Brack_L = 0.25; Brack_W = Mast_W; Brack_H = 0.04;
    brx = 0.10; rot_brx = brx*cos(angle); rot_bry = brx*sin(angle);
    verts = get_rotated_box_at_zero(bx + rot_brx, by + rot_bry, Brack_L, Brack_W, Brack_H, angle);
    verts(:,3) = verts(:,3) + z_carr - Brack_H/2;
    set(h_Robot.Bracket, 'Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8]);
    
    Rail_L = 0.6; Rail_W = 0.02; Rail_H = 0.02; 
    rail_center = mx + (Rail_L/2); 
    rot_rx = rail_center*cos(angle); rot_ry = rail_center*sin(angle);
    verts = get_rotated_box_at_zero(bx + rot_rx, by + rot_ry, Rail_L, Rail_W, Rail_H, angle);
    verts(:,3) = verts(:,3) + z_carr - Brack_H - Rail_H;
    set(h_Robot.Rails, 'Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8]);
    
     Carr_L = Bin.Depth + 0.02; Carr_W = Bin.Length + 0.02; Carr_H = 0.04;
     cx = 0.30 + extension; cy = 0; 
     rot_cx = cx*cos(angle) - cy*sin(angle); rot_cy = cx*sin(angle) + cy*cos(angle);
     verts = get_rotated_box_at_zero(bx + rot_cx, by + rot_cy, Carr_L, Carr_W, Carr_H, angle);
     verts(:,3) = verts(:,3) + z_carr - Carr_H/2; 
     set(h_Robot.Carriage, 'Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8]);
end
function base_coords = update_carousel_graphics(System, Bin_Positions, Pole_Dist, Curve_Rad, OffX, OffY, Bin, h_Bins, h_Hang, h_Chain)
    base_coords = zeros(System.N_Bins_Per_Level, 3);
    for i = 1:System.N_Bins_Per_Level
        [bx, by, angle] = get_track_pos(Bin_Positions(i), Pole_Dist, Curve_Rad);
        base_coords(i, :) = [bx + OffX, by + OffY, angle];
    end
    for lvl = 1:System.Levels
        z_base = System.Base_Height + (lvl-1)*System.Level_Height;
        chain_x = zeros(1, System.N_Bins_Per_Level + 1); chain_y = zeros(1, System.N_Bins_Per_Level + 1);
        chain_z = ones(1, System.N_Bins_Per_Level + 1) * (z_base + Bin.Height + 0.05);
        for i = 1:System.N_Bins_Per_Level
            bx = base_coords(i,1); by = base_coords(i,2); angle = base_coords(i,3);
            % CHECK IF BIN IS ACTIVE (In Carousel)
            if System.Bin_Active(lvl, i)
                verts = get_rotated_box_at_zero(bx, by, Bin.Length, Bin.Depth, Bin.Height, angle);
                verts(:,3) = verts(:,3) + z_base + Bin.Height/2;
                set(h_Bins(lvl, i), 'Vertices', verts, 'Faces', [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5], 'FaceAlpha', 1.0);
            else
                % Hide bin if it's currently on the robot/out of the carousel
                set(h_Bins(lvl, i), 'Vertices', [], 'Faces', []);
            end
            
            h_verts = get_hanger_geometry(bx, by, Bin.Length, Bin.Depth, angle);
            h_verts(:,3) = h_verts(:,3) + z_base + Bin.Height;
            set(h_Hang(lvl, i), 'Vertices', h_verts, 'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6]);
            chain_x(i) = bx; chain_y(i) = by;
        end
        chain_x(end)=chain_x(1); chain_y(end)=chain_y(1);
        set(h_Chain(lvl), 'XData', chain_x, 'YData', chain_y, 'ZData', chain_z);
    end
end
function draw_shell_custom(Origin, L, W, H, Color, Style, Tag)
    x = Origin(1); y = Origin(2); z = Origin(3);
    if strcmp(Style, 'Wireframe'); FaceAlpha = 0.05; EdgeAlpha = 0.5; LineWidth = 1.0;
        if strcmp(Tag, 'Van'), LineWidth = 4.0; EdgeAlpha = 0.3; end 
    else; FaceAlpha = 1.0; EdgeAlpha = 1.0; LineWidth = 0.5; end
    patch([x,x+L,x+L,x],[y,y,y+W,y+W],[z,z,z,z],Color,'FaceAlpha',FaceAlpha,'EdgeAlpha',EdgeAlpha,'LineWidth',LineWidth);
    patch([x,x+L,x+L,x],[y,y,y+W,y+W],[z+H,z+H,z+H,z+H],Color,'FaceAlpha',FaceAlpha,'EdgeAlpha',EdgeAlpha,'LineWidth',LineWidth);
    if strcmp(Tag, 'Housing')
        patch([x,x,x,x],[y,y+W,y+W,y],[z,z,z+H,z+H],Color,'FaceAlpha',FaceAlpha,'EdgeAlpha',EdgeAlpha,'LineWidth',LineWidth);
        patch([x+L,x+L,x+L,x+L],[y,y+W,y+W,y],[z,z,z+H,z+H],Color,'FaceAlpha',FaceAlpha,'EdgeAlpha',EdgeAlpha,'LineWidth',LineWidth);
    elseif strcmp(Tag, 'Van')
        patch([x+L,x+L,x+L,x+L],[y,y+W,y+W,y],[z,z,z+H,z+H],Color,'FaceAlpha',0.1,'EdgeAlpha',EdgeAlpha,'LineWidth',LineWidth);
    end
end
function draw_solid_cylinder(cx, cy, base_z, r, h, color)
    [X,Y,Z] = cylinder(r, 20); X=X+cx; Y=Y+cy; Z=Z*h+base_z;
    surf(X,Y,Z,'FaceColor',color,'EdgeColor','none');
    patch(X(2,:),Y(2,:),Z(2,:),color,'EdgeColor','none');
    patch(X(1,:),Y(1,:),Z(1,:),color,'EdgeColor','none');
end
function [x, y, angle] = get_track_pos(s, L, R)
    Perimeter = 2*L + 2*pi*R; s = mod(s, Perimeter);
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