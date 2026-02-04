function VanSortingSimulation()
    % ======================================================
    % MAIN UI SETUP
    % ======================================================
    
    close all
    % Configuration
    config.rows = 6;
    config.cols = 2;
    
    % Create Figure
    f = figure('Color', 'w', 'Name', 'Persistent Van Inventory', ...
               'NumberTitle', 'off', 'Position', [100, 100, 950, 700]);
    
    % --- TOP CONTROL PANEL ---
    uicontrol('Parent', f, 'Style', 'frame', 'Units', 'normalized', ...
              'Position', [0 0.9 1 0.1], 'BackgroundColor', [0.9 0.9 0.9]);
    % Label: Target Row
    uicontrol('Parent', f, 'Style', 'text', 'String', 'Row (1-6):', ...
              'Units', 'normalized', 'Position', [0.05 0.93 0.1 0.04], ...
              'BackgroundColor', [0.9 0.9 0.9], 'FontSize', 10, 'HorizontalAlignment', 'right');
    h_row = uicontrol('Parent', f, 'Style', 'edit', 'String', '4', ...
                      'Units', 'normalized', 'Position', [0.16 0.935 0.05 0.05], 'FontSize', 11);
                  
    % Label: Target Col
    uicontrol('Parent', f, 'Style', 'text', 'String', 'Col (1-2):', ...
              'Units', 'normalized', 'Position', [0.22 0.93 0.1 0.04], ...
              'BackgroundColor', [0.9 0.9 0.9], 'FontSize', 10, 'HorizontalAlignment', 'right');
    h_col = uicontrol('Parent', f, 'Style', 'edit', 'String', '1', ...
                      'Units', 'normalized', 'Position', [0.33 0.935 0.05 0.05], 'FontSize', 11);
    
    % Button: RETRIEVE
    btn_run = uicontrol('Parent', f, 'Style', 'pushbutton', 'String', 'RETRIEVE SHELF', ...
                    'Units', 'normalized', 'Position', [0.45 0.925 0.2 0.06], ...
                    'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.6 0.8 1.0], ...
                    'Callback', @(src, ev) onRetrieve(f));
    % Button: RESET
    uicontrol('Parent', f, 'Style', 'pushbutton', 'String', 'RESET GRID', ...
              'Units', 'normalized', 'Position', [0.66 0.925 0.15 0.06], ...
              'FontSize', 9, 'BackgroundColor', [1.0 0.6 0.6], ...
              'Callback', @(src, ev) initSystem(f));

    % --- STATUS TEXT DISPLAY (New) ---
    h_status = uicontrol('Parent', f, 'Style', 'text', 'String', 'Ready', ...
                         'Units', 'normalized', 'Position', [0.82 0.91 0.17 0.08], ...
                         'BackgroundColor', [0.9 0.9 0.9], 'ForegroundColor', [0.2 0.2 0.2], ...
                         'FontSize', 10, 'FontWeight', 'normal', 'HorizontalAlignment', 'center');

    % --- 3D VIEW ---
    ax = axes('Parent', f, 'Position', [0.05 0.05 0.9 0.85]);
    
    % Store handle to axes and controls in figure data for easy access
    data.ax = ax;
    data.h_row = h_row;
    data.h_col = h_col;
    data.h_status = h_status; % Store status handle
    data.btn_run = btn_run;
    data.config = config;
    guidata(f, data);
    % Initialize the World
    initSystem(f);
end
% ======================================================
% INITIALIZATION (RESET)
% ======================================================
function initSystem(f)
    data = guidata(f);
    ax = data.ax;
    rows = data.config.rows;
    cols = data.config.cols;
    
    cla(ax);
    set(data.h_status, 'String', 'Ready'); % Reset status text

    % Setup 3D Environment
    view(ax, -30, 45); 
    axis(ax, [0 cols+1 -1 rows+1 0 5]); 
    grid(ax, 'on'); hold(ax, 'on');
    daspect(ax, [1 1 1]);
    xlabel('Col'); ylabel('Row'); zlabel('Height');
    
    % Draw Van
    plot3(ax, [0, 0, 3, 3, 0], [-1, 7, 7, -1, -1], [0, 0, 0, 0, 0], 'k--', 'LineWidth', 1);
    
    % Draw Retrieval Zone (1,1)
    patch(ax, 'Vertices', [0.5 0.5 0; 1.5 0.5 0; 1.5 -0.5 0; 0.5 -0.5 0], ...
          'Faces', [1 2 3 4], 'FaceColor', [1 1 0], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    text(ax, 1, 0, 0, 'PICKUP', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'k');
    % --- LOGICAL STATE INIT ---
    % gridMap stores the ID of the shelf at (row, col). 0 = Empty.
    gridMap = zeros(rows, cols);
    shelfHandles = containers.Map('KeyType', 'double', 'ValueType', 'any');
    textHandles = containers.Map('KeyType', 'double', 'ValueType', 'any');
    
    counter = 1;
    for r = 1:rows
        for c = 1:cols
            % Let's make (1,2) the empty slot initially
            if r == 1 && c == 2
                gridMap(r,c) = 0;
            else
                id = counter;
                gridMap(r,c) = id;
                
                % Draw the physical shelf
                [h_patch, h_text] = drawShelf(ax, c, r, id);
                shelfHandles(id) = h_patch;
                textHandles(id) = h_text;
                
                counter = counter + 1;
            end
        end
    end
    
    % SAVE STATE TO FIGURE
    data.gridMap = gridMap;
    data.shelfHandles = shelfHandles; % Map: ID -> Patch Handle
    data.textHandles = textHandles;   % Map: ID -> Text Handle
    guidata(f, data);
end
% ======================================================
% RETRIEVAL LOGIC
% ======================================================
function onRetrieve(f)
    data = guidata(f);
    ax = data.ax;
    btn = data.btn_run;
    h_stat = data.h_status;
    
    % 1. Get User Input
    target_r = str2double(get(data.h_row, 'String'));
    target_c = str2double(get(data.h_col, 'String'));
    
    % Validation
    max_r = data.config.rows; max_c = data.config.cols;
    if isnan(target_r) || target_r < 1 || target_r > max_r || ...
       isnan(target_c) || target_c < 1 || target_c > max_c
        title(ax, 'Invalid Coordinates');
        set(h_stat, 'String', 'Error: Invalid Coords');
        return;
    end
    
    % 2. Identify WHAT is at that location
    targetID = data.gridMap(target_r, target_c);
    
    if targetID == 0
        title(ax, 'Error: That slot is empty!');
        set(h_stat, 'String', 'Error: Slot Empty');
        return;
    end
    
    % Lock UI
    set(btn, 'Enable', 'off', 'String', 'Moving...');
    set(h_stat, 'String', 'Calculating...');
    
    % Highlight Target
    t_handle = data.shelfHandles(targetID);
    set(t_handle, 'FaceColor', [0.9 0.2 0.2]); % Turn Red
    title(ax, ['Retrieving Shelf #' num2str(targetID) ' from (' num2str(target_r) ',' num2str(target_c) ')']);
    drawnow;
    
    % 3. Find Empty Slot
    [empty_r, empty_c] = find(data.gridMap == 0);
    empty_pos = [empty_r, empty_c];
    
    % 4. Solve Path (BFS) - Destination is always (1,1)
    % Note: Solver calculates moves based on current gridMap
    start_pos = [target_r, target_c];
    moves = bfs_solve(max_r, max_c, start_pos, empty_pos);
    
    % --- UPDATE STATUS TEXT IN FIGURE ---
    % This replaces the msgbox with a text update in the figure
    statusMsg = ['Optimized in: ' num2str(size(moves, 1)) ' moves'];
    set(h_stat, 'String', statusMsg, 'ForegroundColor', [0 0.5 0], 'FontWeight', 'bold');

    % 5. Animate Moves
    % moves format: [start_r, start_c, end_r, end_c]
    % This means "Shelf at Start moves to End" (Empty moves End to Start)
    
    frames_per_move = 10;
    
    for k = 1:size(moves, 1)
        curr_r = moves(k, 1); curr_c = moves(k, 2);
        dest_r = moves(k, 3); dest_c = moves(k, 4);
        
        % Identify which shelf is moving
        movingID = data.gridMap(curr_r, curr_c);
        movingShelf = data.shelfHandles(movingID);
        movingText = data.textHandles(movingID);
        
        % Animate Slide
        p_start = [curr_c, curr_r];
        p_end   = [dest_c, dest_r];
        
        for t = linspace(0, 1, frames_per_move)
            pos = p_start + (p_end - p_start) * t;
            updateShelfVisuals(movingShelf, movingText, pos(1), pos(2));
            drawnow;
        end
        
        % Update Logical Grid
        data.gridMap(dest_r, dest_c) = movingID;
        data.gridMap(curr_r, curr_c) = 0;
    end
    
    % 6. Mock Retrieval Animation (Lift Box)
    if target_r == 1 && target_c == 1
        % Already there
    else
        % It is now at 1,1
    end
    
    title(ax, 'Retrieving Payload...');
    pkg = drawPackage(ax, 1, 1, 3.0);
    arm = plot3(ax, [0.6, 0.6, 1.4, 1.4], [1, 1, 1, 1], [3, 2.8, 2.8, 3], 'k-', 'LineWidth', 3);
    
    % Lift and Drop
    for t = linspace(0, 1, 50)
        y = 1 + (-1) * t;
        updatePackage(pkg, 1, y, 3.0);
        set(arm, 'YData', [y, y, y, y]);
        drawnow;
    end
    for t = linspace(0, 1, 50)
        z = 3.0 + (-3.0) * t;
        updatePackage(pkg, 1, 0, z);
        set(arm, 'ZData', [z, z-0.2, z-0.2, z]);
        drawnow;
    end

    pause(1)
    delete(pkg); delete(arm);
    
    % Reset Color
    set(t_handle, 'FaceColor', [0.3 0.5 0.9]);
    title(ax, ['Shelf #' num2str(targetID) ' Retrieved. Ready for next command.']);

    % 7. SAVE STATE and Unlock
    guidata(f, data);
    set(btn, 'Enable', 'on', 'String', 'RETRIEVE SHELF');
end
% ======================================================
% SOLVER (BFS)
% ======================================================
function path = bfs_solve(rows, cols, t_start, e_start)
    % If target is already at (1,1), no moves needed
    if t_start(1) == 1 && t_start(2) == 1
        path = [];
        return;
    end
    state2ind = @(tr, tc, er, ec) sub2ind([rows, cols, rows, cols], tr, tc, er, ec);
    start_idx = state2ind(t_start(1), t_start(2), e_start(1), e_start(2));
    
    queue = [t_start(1), t_start(2), e_start(1), e_start(2)];
    parentMap = containers.Map('KeyType', 'double', 'ValueType', 'any');
    parentMap(start_idx) = [];
    
    visited = false(rows, cols, rows, cols);
    visited(t_start(1), t_start(2), e_start(1), e_start(2)) = true;
    
    q_ptr = 1;
    final_state = [];
    
    while q_ptr <= size(queue, 1)
        curr = queue(q_ptr, :); q_ptr = q_ptr + 1;
        tr = curr(1); tc = curr(2); er = curr(3); ec = curr(4);
        
        if tr == 1 && tc == 1, final_state = curr; break; end
        
        dirs = [-1, 0; 1, 0; 0, -1; 0, 1];
        for d = 1:4
            nr = er + dirs(d, 1); nc = ec + dirs(d, 2);
            if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                new_tr = tr; new_tc = tc;
                if nr == tr && nc == tc, new_tr = er; new_tc = ec; end % Swap
                
                if ~visited(new_tr, new_tc, nr, nc)
                    visited(new_tr, new_tc, nr, nc) = true;
                    queue(end+1, :) = [new_tr, new_tc, nr, nc];
                    
                    c_idx = state2ind(tr, tc, er, ec);
                    n_idx = state2ind(new_tr, new_tc, nr, nc);
                    % Record Move: Shelf moved from (nr,nc) to (er,ec)
                    parentMap(n_idx) = {c_idx, [nr, nc, er, ec]};
                end
            end
        end
    end
    
    path = [];
    if ~isempty(final_state)
        curr_idx = state2ind(final_state(1), final_state(2), final_state(3), final_state(4));
        while parentMap.isKey(curr_idx) && ~isempty(parentMap(curr_idx))
            val = parentMap(curr_idx);
            path = [val{2}; path];
            curr_idx = val{1};
        end
    end
end
% ======================================================
% GRAPHICS HELPERS
% ======================================================
function [h_patch, h_text] = drawShelf(ax, x, y, id)
    w = 0.8; d = 0.8; h = 3.5;
    % Draw Box
    x_low = x - w/2; x_high = x + w/2;
    y_low = y - d/2; y_high = y + d/2;
    vert = [x_low y_low 0; x_high y_low 0; x_high y_high 0; x_low y_high 0; ...
            x_low y_low h; x_high y_low h; x_high y_high h; x_low y_high h];
    fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    h_patch = patch(ax, 'Vertices', vert, 'Faces', fac, 'FaceColor', [0.3 0.5 0.9], ...
                    'FaceAlpha', 0.8, 'EdgeColor', 'k');
    
    % Draw Text Label
    h_text = text(ax, x, y, h + 0.5, ['#' num2str(id)], ...
              'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
end
function updateShelfVisuals(h_patch, h_text, x, y)
    w = 0.8; d = 0.8; h = 3.5;
    x_low = x - w/2; x_high = x + w/2;
    y_low = y - d/2; y_high = y + d/2;
    vert = [x_low y_low 0; x_high y_low 0; x_high y_high 0; x_low y_high 0; ...
            x_low y_low h; x_high y_low h; x_high y_high h; x_low y_high h];
    set(h_patch, 'Vertices', vert);
    set(h_text, 'Position', [x, y, h+0.5]);
end
function h = drawPackage(ax, x, y, z)
    w = 0.6; d = 0.6; h_val = 0.4;
    x_low = x - w/2; x_high = x + w/2;
    y_low = y - d/2; y_high = y + d/2;
    z_low = z; z_high = z + h_val;
    vert = [x_low y_low z_low; x_high y_low z_low; x_high y_high z_low; x_low y_high z_low; ...
            x_low y_low z_high; x_high y_low z_high; x_high y_high z_high; x_low y_high z_high];
    fac = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    h = patch(ax, 'Vertices', vert, 'Faces', fac, 'FaceColor', [1 1 0], 'FaceAlpha', 1, 'EdgeColor', 'k');
end
function updatePackage(h, x, y, z)
    w = 0.6; d = 0.6; h_val = 0.4;
    x_low = x - w/2; x_high = x + w/2;
    y_low = y - d/2; y_high = y + d/2;
    z_low = z; z_high = z + h_val;
    vert = [x_low y_low z_low; x_high y_low z_low; x_high y_high z_low; x_low y_high z_low; ...
            x_low y_low z_high; x_high y_low z_high; x_high y_high z_high; x_low y_high z_high];
    set(h, 'Vertices', vert);
end