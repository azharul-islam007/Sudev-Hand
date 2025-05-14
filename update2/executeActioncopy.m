%% executeAction.m - Enhanced handover and link selection with hysteresis
function [vehicle, handoverStats] = executeAction(vehicle, action, gNBs)
    % Execute the action decided by the RL agent with improved decision logic
    % to prevent excessive handovers and ping-pong effect
    
    % Initialize handover statistics
    handoverStats = struct('type', 0, 'isPingPong', false, 'interfaceSwitch', false);
    
    % Initialize history arrays if they don't exist
    % Using cell arrays instead of struct arrays for better compatibility
    if ~isfield(vehicle, 'handover_times')
        vehicle.handover_times = [];   % When handovers happened
        vehicle.handover_types = {};   % Type of handover
        vehicle.handover_from = [];    % Source cell
        vehicle.handover_to = [];      % Target cell
    end
    
    if ~isfield(vehicle, 'previous_cell_times')
        vehicle.previous_cell_times = [];  % When cells were added to history
        vehicle.previous_cell_ids = [];    % Which cells are in history
    end
    
    % IMPROVED: Track last handover time and add hysteresis timers
    if ~isfield(vehicle, 'last_handover_time')
        vehicle.last_handover_time = 0;
    end
    
    % Minimum time between handovers (hysteresis timer)
    min_handover_interval = 5; % 5 seconds between handovers
    current_time = now() * 86400; % Convert to seconds
    
    % Check if handover is allowed based on timer
    handover_allowed = (current_time - vehicle.last_handover_time) >= min_handover_interval;
    
    % Set maximum history length
    max_history = 5;
    
    % Process action with improved decision logic
    switch action
        case 1 % MCG Handover
            if handover_allowed
                % Find best macro cell based on RSRP and add margin for current cell
                best_macro = findBestCellWithHysteresis(vehicle, gNBs, "macro");
                
                % Only handover if the new cell is different and better by margin
                if best_macro ~= 0 && best_macro ~= vehicle.serving_mcg
                    % Check for ping-pong: returning to a recent cell
                    isPingPong = false;
                    
                    % Check previous cells
                    for i = 1:length(vehicle.previous_cell_ids)
                        if vehicle.previous_cell_ids(i) == best_macro
                            % Check how recent this cell was
                            if (current_time - vehicle.previous_cell_times(i)) < 30
                                isPingPong = true;
                                break;
                            end
                        end
                    end
                    
                    handoverStats.isPingPong = isPingPong;
                    
                    % Execute the handover if it passes our criteria
                    prev_cell = vehicle.serving_mcg;
                    vehicle.serving_mcg = best_macro;
                    
                    % Update handover time
                    vehicle.last_handover_time = current_time;
                    
                    % Record handover in history (using separate arrays)
                    vehicle.handover_times = [vehicle.handover_times; current_time];
                    vehicle.handover_types{end+1} = "MCG";
                    vehicle.handover_from = [vehicle.handover_from; prev_cell];
                    vehicle.handover_to = [vehicle.handover_to; best_macro];
                    
                    % Update previous cells list (using separate arrays)
                    vehicle.previous_cell_times = [vehicle.previous_cell_times; current_time];
                    vehicle.previous_cell_ids = [vehicle.previous_cell_ids; prev_cell];
                    
                    % Trim history if needed
                    if length(vehicle.previous_cell_times) > max_history
                        vehicle.previous_cell_times = vehicle.previous_cell_times(end-max_history+1:end);
                        vehicle.previous_cell_ids = vehicle.previous_cell_ids(end-max_history+1:end);
                    end
                    
                    handoverStats.type = 1;  % MCG handover
                end
            end
            
        case 2 % SCG Add
            if handover_allowed && vehicle.serving_scg == 0
                % IMPROVED: Only add SCG if SINR conditions are appropriate
                % First check if MCG SINR is below threshold suggesting SCG would help
                uu_sinr_threshold = 15; % SINR threshold for SCG addition
                need_scg = false;
                if isfield(vehicle, 'sinr_uu')
                    need_scg = isscalar(vehicle.sinr_uu) && vehicle.sinr_uu < uu_sinr_threshold;
                end
                
                if need_scg
                    % Find best small cell with hysteresis
                    best_small = findBestCellWithHysteresis(vehicle, gNBs, "small");
                    
                    % Only add if good small cell found and SINR is adequate
                    if best_small > 0
                        % Check small cell SINR (simplified)
                        [~, small_cell_sinr] = estimateCellPerformance(vehicle, gNBs(best_small));
                        
                        if small_cell_sinr > 5 % Minimum SINR for SCG addition
                            vehicle.serving_scg = best_small;
                            
                            % Record in history
                            vehicle.handover_times = [vehicle.handover_times; current_time];
                            vehicle.handover_types{end+1} = "SCG_Add";
                            vehicle.handover_from = [vehicle.handover_from; 0];
                            vehicle.handover_to = [vehicle.handover_to; best_small];
                            
                            % Update handover time
                            vehicle.last_handover_time = current_time;
                            
                            handoverStats.type = 2;  % SCG add
                        end
                    end
                end
            end
            
        case 3 % SCG Remove
            if vehicle.serving_scg > 0
                % IMPROVED: Only remove SCG if MCG conditions are good
                % Check if MCG SINR is above threshold suggesting SCG is not needed
                uu_sinr_threshold = 20; % Higher SINR threshold for removal than addition (hysteresis)
                safe_to_remove = false;
                if isfield(vehicle, 'sinr_uu')
                    safe_to_remove = isscalar(vehicle.sinr_uu) && vehicle.sinr_uu > uu_sinr_threshold;
                end
                
                if safe_to_remove
                    prev_scg = vehicle.serving_scg;
                    vehicle.serving_scg = 0;
                    
                    % Record in history
                    vehicle.handover_times = [vehicle.handover_times; current_time];
                    vehicle.handover_types{end+1} = "SCG_Remove";
                    vehicle.handover_from = [vehicle.handover_from; prev_scg];
                    vehicle.handover_to = [vehicle.handover_to; 0];
                    
                    % Update handover time
                    vehicle.last_handover_time = current_time;
                    
                    handoverStats.type = 3;  % SCG remove
                end
            end
            
        case 4 % Switch Safety to Uu
            % IMPROVED: Only switch safety to Uu if PC5 performance is poor
            if ~strcmpi(vehicle.safety_interface, 'Uu')
                switch_needed = false;
                
                % Check PC5 performance to determine if switch is needed
                if ~isfield(vehicle, 'pc5_metrics') || isempty(vehicle.pc5_metrics)
                    % No PC5 links = switch needed
                    switch_needed = true;
                else
                    % Calculate average PC5 SINR
                    pc5_sinrs = [];
                    for i = 1:length(vehicle.pc5_metrics)
                        if isfield(vehicle.pc5_metrics(i), 'sinr')
                            pc5_sinrs = [pc5_sinrs, vehicle.pc5_metrics(i).sinr];
                        end
                    end
                    
                    if isempty(pc5_sinrs) || mean(pc5_sinrs) < 0 % SINR threshold for PC5
                        switch_needed = true;
                    end
                end
                
                % Only switch if needed and Uu link is good
                sinr_check = false;
                if isfield(vehicle, 'sinr_uu')
                    sinr_check = isscalar(vehicle.sinr_uu) && vehicle.sinr_uu > 5;
                end
                if switch_needed && sinr_check
                    vehicle.safety_interface = 'Uu';
                    handoverStats.interfaceSwitch = true;
                    handoverStats.type = 4;  % Interface switch
                end
            end
            
        case 5 % Switch Safety to PC5
            % IMPROVED: Only switch safety to PC5 if PC5 performance is good
            if ~strcmpi(vehicle.safety_interface, 'PC5')
                switch_possible = false;
                
                % Check if PC5 links exist and have good performance
                if isfield(vehicle, 'pc5_metrics') && ~isempty(vehicle.pc5_metrics)
                    % Calculate average PC5 SINR
                    pc5_sinrs = [];
                    for i = 1:length(vehicle.pc5_metrics)
                        if isfield(vehicle.pc5_metrics(i), 'sinr')
                            pc5_sinrs = [pc5_sinrs, vehicle.pc5_metrics(i).sinr];
                        end
                    end
                    
                    if ~isempty(pc5_sinrs) && mean(pc5_sinrs) > 5 % SINR threshold for PC5
                        switch_possible = true;
                    end
                end
                
                if switch_possible
                    vehicle.safety_interface = 'PC5';
                    handoverStats.interfaceSwitch = true;
                    handoverStats.type = 5;  % Interface switch
                end
            end
            
        case 6 % Switch Non-Safety to Uu
            % IMPROVED: Switch non-safety based on throughput requirements and availability
            if ~strcmpi(vehicle.nonsafety_interface, 'Uu')
                % Check if we have a good cellular connection
                sinr_check = false;
                if isfield(vehicle, 'sinr_uu')
                    sinr_check = isscalar(vehicle.sinr_uu) && vehicle.sinr_uu > 0;
                end
                if vehicle.serving_mcg > 0 && sinr_check
                    vehicle.nonsafety_interface = 'Uu';
                    handoverStats.interfaceSwitch = true;
                    handoverStats.type = 6;  % Interface switch
                end
            end
            
        case 7 % Switch Non-Safety to PC5
            % IMPROVED: Only switch if PC5 has capacity and cellular is congested
            if ~strcmpi(vehicle.nonsafety_interface, 'PC5')
                % Check if PC5 links exist and cellular is congested
                if isfield(vehicle, 'pc5_metrics') && ~isempty(vehicle.pc5_metrics)
                    % Check cellular congestion
                    cell_congested = false;
                    if vehicle.serving_mcg > 0 && vehicle.serving_mcg <= length(gNBs)
                        cell_congested = gNBs(vehicle.serving_mcg).load > 70; % 70% load threshold
                    end
                    
                    % Only switch to PC5 if cell is congested or PC5 has good SINR
                    pc5_sinrs = [];
                    for i = 1:length(vehicle.pc5_metrics)
                        if isfield(vehicle.pc5_metrics(i), 'sinr')
                            pc5_sinrs = [pc5_sinrs, vehicle.pc5_metrics(i).sinr];
                        end
                    end
                    
                    good_pc5 = ~isempty(pc5_sinrs) && mean(pc5_sinrs) > 10; % Higher SINR threshold for non-safety
                    
                    if cell_congested || good_pc5
                        vehicle.nonsafety_interface = 'PC5';
                        handoverStats.interfaceSwitch = true;
                        handoverStats.type = 7;  % Interface switch
                    end
                end
            end
            
        case 8 % No Change
            % Do nothing
            
        otherwise
            warning('Unknown action: %d', action);
    end
end

function best_cell = findBestCellWithHysteresis(vehicle, gNBs, cell_type)
    % Find the best cell of the specified type using hysteresis to prevent frequent handovers
    
    best_rsrp = -inf;
    best_cell = 0;
    
    % Current serving cell
    current_cell = 0;
    if strcmpi(cell_type, "macro") && isfield(vehicle, 'serving_mcg')
        current_cell = vehicle.serving_mcg;
    elseif strcmpi(cell_type, "small") && isfield(vehicle, 'serving_scg')
        current_cell = vehicle.serving_scg;
    end
    
    % Hysteresis margin (dB) - new cell must be better by this margin
    hysteresis_margin = 3; % 3 dB
    
    % Current cell RSRP
    current_rsrp = -inf;
    if current_cell > 0 && current_cell <= length(gNBs)
        [current_rsrp, ~] = estimateCellPerformance(vehicle, gNBs(current_cell));
    end
    
    % Evaluate all cells of the specified type
    for i = 1:length(gNBs)
        if strcmpi(gNBs(i).type, cell_type)
            % Estimate RSRP and SINR
            [cell_rsrp, cell_sinr] = estimateCellPerformance(vehicle, gNBs(i));
            
            % Only consider cells with minimum SINR
            if cell_sinr > -5 % Minimum SINR threshold
                % Apply hysteresis for current cell
                effective_rsrp = cell_rsrp;
                if i == current_cell
                    effective_rsrp = effective_rsrp + hysteresis_margin;
                end
                
                % Update best cell if this one is better
                if effective_rsrp > best_rsrp
                    best_rsrp = effective_rsrp;
                    best_cell = i;
                end
            end
        end
    end
    
    % Apply handover margin to prevent handovers for marginal gain
    if best_cell ~= current_cell && current_cell > 0
        % Only handover if gain is significant
        if best_rsrp < (current_rsrp + hysteresis_margin)
            best_cell = current_cell; % Stay with current cell
        end
    end
end

function [rsrp, sinr] = estimateCellPerformance(vehicle, cell)
    % Estimate RSRP and SINR for a specific cell
    
    % Calculate distance
    distance = norm(vehicle.position - cell.position);
    
    % Simple path loss model
    carrier_freq_ghz = 3.5; % 3.5 GHz
    
    % 3GPP Urban Micro path loss model
    if distance < 10
        distance = 10; % Minimum distance
    end
    
    % Determine LoS probability based on distance
    p_los = min(18/distance, 1) * (1 - exp(-distance/36)) + exp(-distance/36);
    is_los = rand() < p_los;
    
    % Calculate path loss
    if is_los
        path_loss = 32.4 + 21 * log10(distance) + 20 * log10(carrier_freq_ghz);
    else
        path_loss = 32.4 + 40 * log10(distance) + 20 * log10(carrier_freq_ghz);
    end
    
    % Add shadow fading
    if is_los
        shadow_std = 4;  % For LoS
    else
        shadow_std = 6;  % For NLoS
    end
    shadow_fading = shadow_std * randn();
    
    % Calculate RSRP
    tx_power = cell.power;
    rsrp = tx_power - path_loss - shadow_fading;
    
    % Estimate SINR based on RSRP and typical noise+interference
    % Simplified model - in real system would consider actual interference
    noise_floor = -100; % Typical noise floor in dBm
    interference = -110 + 10 * log10(cell.load / 100 + 0.1); % Interference increases with load
    
    sinr = rsrp - 10 * log10(10^(noise_floor/10) + 10^(interference/10));
end