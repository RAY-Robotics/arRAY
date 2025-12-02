classdef CarouselController < matlab.DiscreteEventSystem
    
    % --- 1. DEFINE PUBLIC PROPERTIES (Inputs from Simulink) ---
    properties (Nontunable)
        % These match the inputs you'll provide from Constant blocks
        ConstantsStruct;    % The System and Carousel constants (R, V_max, etc.)
        Robot;              % The rigidBodyTree object (robot)
        IK;                 % The inverseKinematics object (ik)
        Weights;            % The IK solver weights
    end
    
    % --- 2. DEFINE PRIVATE PROPERTIES (Internal State and Constants) ---
    properties (Access = private)
        CurrentRobotConfig; % Stores the joint configuration after the last pick (the state)
    end
    
    % --- 3. CORE METHODS (System Definition) ---
    methods (Access = protected)
        
        function setupImpl(obj)
            % Initialize the internal state variables once at startup
            
            % Get home configuration from the rigidBodyTree object
            initialConfig = homeConfiguration(obj.Robot); 
            obj.CurrentRobotConfig = initialConfig;
        end
        
        function num = getNumInputsImpl(~)
            % Only the Entity needs to come into the block
            num = 1; 
        end
        
        function num = getNumOutputsImpl(~)
            % Output 1: T_cycle (Total Time)
            % Output 2: Target_XYZ (for visualization, optional)
            num = 2; 
        end
        
        function entityTypes = getEntityTypesImpl(obj)
            
            % --- 1. Define the Attributes using a MATLAB Struct Array ---
            % This uses explicit field names recognized by the System Object.
            attributeStruct = struct(...
                'Name', 'Target_Bin_ID', ...
                'DataType', 'double', ... % Use double for compatibility
                'InitialValue', 1);
            
            % --- 2. Define the Entity Type ---
            % Pass the struct array directly as the 'Attributes' argument.
            entityTypes = obj.entityType('PickOrder', ...
                                         'Priority', 1, ...
                                         'Attributes', attributeStruct);
        end
        
        % Define the output types (T_cycle is a double, Target_XYZ is a vector)
        function out = getOutputDataTypeImpl(~)
             out = {'double', 'double'};
        end
        
        % --- 4. THE ENTRY LOGIC (Executed when an entity arrives) ---
        function [entity, T_cycle, target_xyz] = entryImpl(obj, entity, ~)
            
            % Access properties
            constants = obj.ConstantsStruct;
            
            % --- A. Calculate Carousel Rotation Time (T_rot) ---
            target_bin_index = entity.Target_Bin_ID;
            access_port_index = 1; 
            current_bin_index = access_port_index; 
            
            % Assume you have access to calculate_retrieval_time.m
            [~, T_rot, T_grab_fixed] = calculate_retrieval_time(...
                current_bin_index, target_bin_index, constants);

            % --- B. Determine Grabber Target and IK ---
            
            % Assume you have access to get_bin_position.m
            target_xyz = get_bin_position(access_port_index, constants);
            
            % --- C. Dynamic T_grab Calculation (Simplified) ---
            
            % For simplicity, we skip the IK time calculation for now 
            % and use the fixed time until we implement the IK velocity constraints.
            T_grab_dynamic = T_grab_fixed; 
            
            % --- D. Final Cycle Time & State Update ---
            T_cycle = T_rot + T_grab_dynamic;
            
            % ******* Placeholder for Robot Configuration Update *******
            % In a fully implemented model, you'd solve IK here and update the state:
            % [angles_target, ~] = step(obj.IK, T_target, obj.Weights, obj.CurrentRobotConfig);
            % obj.CurrentRobotConfig = angles_target; 
            
            % --- E. Emit Entity ---
            % Emit the entity with the calculated service time (T_cycle)
            entity = obj.emit(entity, T_cycle);
        end
    end
end