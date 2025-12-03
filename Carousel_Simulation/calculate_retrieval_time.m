function [T_cycle, T_rot, T_grab] = calculate_retrieval_time(current_bin_index, target_bin_index, constants)
% CALCULATE_RETRIEVAL_TIME Determines the total time required for a pick cycle.
%
% Inputs:
%   current_bin_index: The starting bin index (where the carousel currently is).
%   target_bin_index: The bin index to be retrieved (the requested parcel).
%   constants: A structure containing Carousel, System, and Operational dynamics.

    % --- Unpack Constants ---
    N_columns = constants.System.N_Columns;
    V_max = constants.Carousel.Max_Linear_Vel;
    T_accel = constants.Carousel.Accel_Time;
    T_decel = T_accel; % Assume deceleration time equals acceleration time
    D_bin = constants.Carousel.Bin_Spacing;
    
    % --- 1. Calculate Shortest Travel Distance (Delta L) ---
    
    % Distance in terms of number of bins
    N_diff = abs(target_bin_index - current_bin_index);
    
    % The carousel can move in two directions. Find the shortest path (min bins).
    Delta_N = min(N_diff, N_columns - N_diff);
    
    % Convert bin count to linear distance (Delta L)
    Delta_L = Delta_N * D_bin;
    
    % --- 2. Calculate Distance traveled during Acceleration/Deceleration ---
    
    % Distance traveled during one acceleration phase (D = 0.5 * V_max * T_accel)
    D_accel = 0.5 * V_max * T_accel; 
    
    % Total distance traveled during both accel and decel phases
    D_accel_decel = 2 * D_accel; 
    
    % --- 3. Calculate Rotation Time (T_rot) based on travel distance ---
    
    if Delta_L <= D_accel_decel
        % Case 1: Short Travel (Doesn't reach V_max)
        % Time is calculated using the constant acceleration equation: D = 0.5 * a * t^2
        % where a = V_max / T_accel. 
        % T_rot = sqrt(2 * Delta_L / (V_max / T_accel))
        
        T_rot = sqrt((2 * Delta_L * T_accel) / V_max);
        
    else
        % Case 2: Long Travel (Reaches V_max)
        % Time = Accel Time + Constant Velocity Time + Decel Time
        
        % Distance traveled at constant velocity (D_const)
        D_const = Delta_L - D_accel_decel;
        
        % Time spent at constant velocity (T_const = D_const / V_max)
        T_const = D_const / V_max;
        
        T_rot = T_accel + T_const + T_decel; 
    end
    
    % --- 4. Calculate Grabber Time (T_grab) ---
    
    % In the initial DES model, we use a simple estimate for the grabber cycle time.
    % This time covers the motion, grasping, and retraction once the carousel stops.
    T_grab_motion = 2.0; % Time for arm to move to bin once carousel stops (seconds)
    T_grab_actuation = 0.5; % Time to grasp the parcel
    T_grab_retract = 1.0; % Time to retract to safe position

    T_grab = T_grab_motion + T_grab_actuation + T_grab_retract;
    
    % --- 5. Calculate Total Cycle Time (T_cycle) ---
    % Since the grabber must wait for the carousel to stop, the events are sequential.
    T_cycle = T_rot + T_grab;
end