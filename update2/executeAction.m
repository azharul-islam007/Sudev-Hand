%% executeAction.m - Enhanced handover and link selection with hysteresis
function [vehicle, handoverStats] = executeAction(vehicle, action, gNBs)
    % Initialize handover statistics
    handoverStats = struct('type', 0, 'isPingPong', false, 'interfaceSwitch', false);
    
    % Initialize history arrays if they don't exist
    if ~isfield(vehicle, 'handover_times')
        vehicle.handover_times = [];
        vehicle.handover_types = {};
        vehicle.handover_from = [];
        vehicle.handover_to = [];
    end
    
    if ~isfield(vehicle, 'previous_cell_times')
        vehicle.previous_cell_times = [];
        vehicle.previous_cell_ids = [];
    end
    
    % Track previous SINR for trend analysis
    if isfield(vehicle, 'sinr_uu')
        if isscalar(vehicle.sinr_uu)
            % Store current SINR value for next iteration
            vehicle.previous_sinr = vehicle.sinr_uu;
        end
    end
    
    % Track last handover time
    if ~isfield(vehicle, 'last_handover_time')
        vehicle.last_handover_time = 0;
    end
    
    % Minimum time between handovers (hysteresis timer)
    min_handover_interval = 5; % 5 seconds between handovers
    current_time = now() * 86400; % Convert to seconds
    
    % Check if handover is allowed based on timer
    time_diff = current_time - vehicle.last_handover_time;
    handover_allowed = time_diff >= min_handover_interval;
    
    % Set maximum history length
    max_history = 5;
    
    % Check if vehicle is approaching intersection (context awareness)
    at_intersection = false;
    if isfield(vehicle, 'position')
        if isnumeric(vehicle.position)
            dist_to_center = norm(vehicle.position);
            if dist_to_center < 50 % Within 50m of intersection
                at_intersection = true;
            end
        end
    end
    
    % Ensure action is scalar
    actionValue = 8; % Default: no change
    if isscalar(action)
        actionValue = action;
    end
    
    % Process action based on type
    switch actionValue
        case 1 % MCG Handover
            if handover_allowed
                % Find best macro cell based on RSRP
                best_macro = findBestCellWithHysteresis(vehicle, gNBs, "macro");
                
                % Only proceed if best_macro is valid
                if isscalar(best_macro)
                    % Only handover if the new cell is different
                    if best_macro > 0 
                        % Check current serving cell
                        current_mcg = 0;
                        if isfield(vehicle, 'serving_mcg')
                            if isscalar(vehicle.serving_mcg)
                                current_mcg = vehicle.serving_mcg;
                            end
                        end
                        
                        % Only handover if different cell
                        if best_macro ~= current_mcg
                            % Check for ping-pong: returning to a recent cell
                            isPingPong = false;
                            
                            for i = 1:length(vehicle.previous_cell_ids)
                                if isscalar(vehicle.previous_cell_ids(i))
                                    if vehicle.previous_cell_ids(i) == best_macro
                                        % Check time threshold
                                        if isscalar(vehicle.previous_cell_times(i))
                                            if (current_time - vehicle.previous_cell_times(i)) < 15
                                                isPingPong = true;
                                                break;
                                            end
                                        end
                                    end
                                end
                            end
                            
                            handoverStats.isPingPong = isPingPong;
                            
                            % Execute the handover
                            prev_cell = current_mcg;
                            vehicle.serving_mcg = best_macro;
                            
                            % Update handover time
                            vehicle.last_handover_time = current_time;
                            
                            % Record handover in history
                            vehicle.handover_times = [vehicle.handover_times; current_time];
                            vehicle.handover_types{end+1} = "MCG";
                            vehicle.handover_from = [vehicle.handover_from; prev_cell];
                            vehicle.handover_to = [vehicle.handover_to; best_macro];
                            
                            % Update previous cells list
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
                end
            end
            
        case 2 % SCG Add
            % Check if handover is allowed and no SCG exists
            if handover_allowed
                scg_check = false;
                if isfield(vehicle, 'serving_scg')
                    if isscalar(vehicle.serving_scg)
                        scg_check = (vehicle.serving_scg == 0);
                    end
                end
                
                if scg_check
                    % Check if MCG SINR is below threshold suggesting SCG would help
                    uu_sinr_threshold = 15; % SINR threshold for SCG addition
                    need_scg = false;
                    
                    if isfield(vehicle, 'sinr_uu')
                        if isscalar(vehicle.sinr_uu)
                            need_scg = vehicle.sinr_uu < uu_sinr_threshold;
                            
                            % If at intersection, be more aggressive with SCG addition
                            if at_intersection
                                need_scg = vehicle.sinr_uu < uu_sinr_threshold + 3; % More likely to add SCG at intersections
                            end
                        end
                    end
                    
                    if need_scg
                        % Find best small cell
                        best_small = findBestCellWithHysteresis(vehicle, gNBs, "small");
                        
                        % Check if best small cell is valid
                        if isscalar(best_small)
                            if best_small > 0
                                % Check small cell SINR
                                [~, small_cell_sinr] = estimateCellPerformance(vehicle, gNBs(best_small));
                                
                                % Only add if SINR is adequate
                                if isscalar(small_cell_sinr)
                                    threshold = 5; % Base threshold for SCG addition
                                    
                                    % Lower threshold at intersections for more SCG use
                                    if at_intersection
                                        threshold = 3;
                                    end
                                    
                                    if small_cell_sinr > threshold
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
                    end
                end
            end
            
        case 3 % SCG Remove
            % Check if SCG exists
            scg_exists = false;
            if isfield(vehicle, 'serving_scg')
                if isscalar(vehicle.serving_scg)
                    scg_exists = (vehicle.serving_scg > 0);
                end
            end
            
            if scg_exists
                % Check if MCG SINR is above threshold suggesting SCG is not needed
                uu_sinr_threshold = 20; % Higher threshold for removal
                safe_to_remove = false;
                
                if isfield(vehicle, 'sinr_uu')
                    if isscalar(vehicle.sinr_uu)
                        safe_to_remove = vehicle.sinr_uu > uu_sinr_threshold;
                        
                        % Be more conservative about removing SCG at intersections
                        if at_intersection
                            safe_to_remove = vehicle.sinr_uu > uu_sinr_threshold + 5;
                        end
                    end
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
            if isfield(vehicle, 'safety_interface')
                if ischar(vehicle.safety_interface)
                    if ~strcmpi(vehicle.safety_interface, 'Uu')
                        % Check PC5 performance to determine if switch is needed
                        switch_needed = checkPC5Performance(vehicle, true); % true = check for poor performance
                        
                        % Check if Uu link is good
                        sinr_good = false;
                        if isfield(vehicle, 'sinr_uu')
                            if isscalar(vehicle.sinr_uu)
                                sinr_good = vehicle.sinr_uu > 5;
                                
                                % Be more conservative about switching to Uu at intersections
                                if at_intersection
                                    sinr_good = vehicle.sinr_uu > 8; % Higher threshold at intersections
                                end
                            end
                        end
                        
                        % Switch only if needed and Uu is good
                        if switch_needed
                            if sinr_good
                                vehicle.safety_interface = 'Uu';
                                handoverStats.interfaceSwitch = true;
                                handoverStats.type = 4;  % Interface switch
                            end
                        end
                    end
                end
            end
            
        case 5 % Switch Safety to PC5
            if isfield(vehicle, 'safety_interface')
                if ischar(vehicle.safety_interface)
                    if ~strcmpi(vehicle.safety_interface, 'PC5')
                        % Check if PC5 links exist and have good performance
                        switch_possible = checkPC5Performance(vehicle, false); % false = check for good performance
                        
                        if switch_possible
                            vehicle.safety_interface = 'PC5';
                            handoverStats.interfaceSwitch = true;
                            handoverStats.type = 5;  % Interface switch
                        end
                    end
                end
            end
            
        case 6 % Switch Non-Safety to Uu
            if isfield(vehicle, 'nonsafety_interface')
                if ischar(vehicle.nonsafety_interface)
                    if ~strcmpi(vehicle.nonsafety_interface, 'Uu')
                        % Check if we have a good cellular connection
                        sinr_good = false;
                        if isfield(vehicle, 'sinr_uu')
                            if isscalar(vehicle.sinr_uu)
                                sinr_good = vehicle.sinr_uu > 0;
                            end
                        end
                        
                        mcg_exists = false;
                        if isfield(vehicle, 'serving_mcg')
                            if isscalar(vehicle.serving_mcg)
                                mcg_exists = (vehicle.serving_mcg > 0);
                            end
                        end
                        
                        % Prioritize safety data over non-safety
                        safety_on_uu = false;
                        if isfield(vehicle, 'safety_interface')
                            if ischar(vehicle.safety_interface)
                                safety_on_uu = strcmpi(vehicle.safety_interface, 'Uu');
                            end
                        end
                        
                        % Only put non-safety on Uu if safety isn't using it OR SINR is high
                        good_enough_for_both = false;
                        if isfield(vehicle, 'sinr_uu')
                            if isscalar(vehicle.sinr_uu)
                                good_enough_for_both = vehicle.sinr_uu > 15; % High SINR can support both
                            end
                        end
                        
                        if mcg_exists && sinr_good && (good_enough_for_both || ~safety_on_uu)
                            vehicle.nonsafety_interface = 'Uu';
                            handoverStats.interfaceSwitch = true;
                            handoverStats.type = 6;  % Interface switch
                        end
                    end
                end
            end
            
        case 7 % Switch Non-Safety to PC5
            if isfield(vehicle, 'nonsafety_interface')
                if ischar(vehicle.nonsafety_interface)
                    if ~strcmpi(vehicle.nonsafety_interface, 'PC5')
                        % Check if PC5 links exist
                        pc5_available = isfield(vehicle, 'pc5_metrics');
                        if pc5_available
                            pc5_available = ~isempty(vehicle.pc5_metrics);
                        end
                        
                        if pc5_available
                            % Check cellular congestion
                            cell_congested = false;
                            if isfield(vehicle, 'serving_mcg')
                                if isscalar(vehicle.serving_mcg)
                                    if vehicle.serving_mcg > 0
                                        if vehicle.serving_mcg <= length(gNBs)
                                            cell_congested = gNBs(vehicle.serving_mcg).load > 70;
                                        end
                                    end
                                end
                            end
                            
                            % Check PC5 SINR
                            good_pc5 = checkPC5SINRThreshold(vehicle, 10); % Higher threshold for non-safety
                            
                            % Also check if safety is on PC5 with good performance
                            safety_on_pc5_with_good_perf = false;
                            if isfield(vehicle, 'safety_interface')
                                if ischar(vehicle.safety_interface)
                                    if strcmpi(vehicle.safety_interface, 'PC5')
                                        good_perf = checkPC5SINRThreshold(vehicle, 15);
                                        safety_on_pc5_with_good_perf = good_perf;
                                    end
                                end
                            end
                            
                            if cell_congested || good_pc5 || safety_on_pc5_with_good_perf
                                vehicle.nonsafety_interface = 'PC5';
                                handoverStats.interfaceSwitch = true;
                                handoverStats.type = 7;  % Interface switch
                            end
                        end
                    end
                end
            end
            
        case 8 % No Change
            % Do nothing
            
        otherwise
            % Do nothing for unknown actions
    end
end

function result = checkSINRCondition(vehicle, field, op, threshold)
    % Safely check SINR condition with scalar verification
    result = false;
    
    if isfield(vehicle, field)
        value = vehicle.(field);
        
        % Ensure it's scalar
        if isscalar(value)
            % Compare with threshold using specified operator
            if strcmp(op, '<')
                result = value < threshold;
            elseif strcmp(op, '>')
                result = value > threshold;
            elseif strcmp(op, '<=')
                result = value <= threshold;
            elseif strcmp(op, '>=')
                result = value >= threshold;
            end
        end
    end
end

function switch_needed = checkPC5Performance(vehicle, check_poor)
    % Check if PC5 performance is poor (check_poor=true) or good (check_poor=false)
    switch_needed = false;
    
    % Check for PC5 metrics
    has_pc5 = isfield(vehicle, 'pc5_metrics');
    if has_pc5
        has_pc5 = ~isempty(vehicle.pc5_metrics);
    end
    
    if ~has_pc5
        % No PC5 links
        if check_poor
            switch_needed = true; % Need switch if checking for poor performance
        else
            switch_needed = false; % Don't switch if checking for good performance
        end
        return;
    end
    
    % Get all PC5 SINRs
    pc5_sinrs = [];
    for i = 1:length(vehicle.pc5_metrics)
        if isfield(vehicle.pc5_metrics(i), 'sinr')
            sinr_val = vehicle.pc5_metrics(i).sinr;
            if isscalar(sinr_val)
                pc5_sinrs = [pc5_sinrs, sinr_val];
            end
        end
    end
    
    if isempty(pc5_sinrs)
        % No valid SINR values
        if check_poor
            switch_needed = true; % Need switch if checking for poor performance
        else
            switch_needed = false; % Don't switch if checking for good performance
        end
        return;
    end
    
    % Calculate average SINR
    avg_sinr = mean(pc5_sinrs);
    
    % Determine result based on what we're checking for
    if check_poor
        % Checking for poor performance - more tolerant of poor PC5
        threshold = -2; % DECREASED from 0 to -2 for poor performance
        if isscalar(avg_sinr)
            switch_needed = avg_sinr < threshold;
        end
    else
        % Checking for good performance - more conservative with switching to PC5
        threshold = 7; % INCREASED from 5 to 7 for good performance
        if isscalar(avg_sinr)
            switch_needed = avg_sinr > threshold;
        end
    end
    
    % Check if at intersection - we want more reliable PC5 at intersections
    at_intersection = false;
    if isfield(vehicle, 'position')
        if isnumeric(vehicle.position)
            dist_to_center = norm(vehicle.position);
            if dist_to_center < 50 % Within 50m of intersection
                at_intersection = true;
            end
        end
    end
    
    % Adjust thresholds for intersections
    if at_intersection && check_poor
        % Be more conservative switching away from PC5 at intersections
        if isscalar(avg_sinr)
            switch_needed = avg_sinr < -5; % Even more tolerant at intersections
        end
    end
end

function good_pc5 = checkPC5SINRThreshold(vehicle, threshold)
    % Check if PC5 SINR is above a threshold
    good_pc5 = false;
    
    % Check for PC5 metrics
    has_pc5 = isfield(vehicle, 'pc5_metrics');
    if has_pc5
        has_pc5 = ~isempty(vehicle.pc5_metrics);
    end
    
    if ~has_pc5
        return;
    end
    
    % Get all PC5 SINRs
    pc5_sinrs = [];
    for i = 1:length(vehicle.pc5_metrics)
        if isfield(vehicle.pc5_metrics(i), 'sinr')
            sinr_val = vehicle.pc5_metrics(i).sinr;
            if isscalar(sinr_val)
                pc5_sinrs = [pc5_sinrs, sinr_val];
            end
        end
    end
    
    if isempty(pc5_sinrs)
        return;
    end
    
    % Calculate average SINR
    avg_sinr = mean(pc5_sinrs);
    if isscalar(avg_sinr)
        good_pc5 = avg_sinr > threshold;
    end
end

function best_cell = findBestCellWithHysteresis(vehicle, gNBs, cell_type)
    % Find the best cell of the specified type using hysteresis to prevent frequent handovers
    
    best_rsrp = -inf;
    best_cell = 0;
    
    % Adaptive hysteresis based on vehicle speed
    hysteresis_margin = 3; % Base value
    
    if isfield(vehicle, 'velocity')
        if isnumeric(vehicle.velocity)
            speed = norm(vehicle.velocity);
            % Increase hysteresis for faster vehicles
            if speed > 10 % m/s (approx. 36 km/h)
                hysteresis_margin = 4; % Higher margin for faster vehicles
            elseif speed > 5 % m/s (approx. 18 km/h)
                hysteresis_margin = 3.5; % Medium margin for medium speed
            end
        end
    end
    
    % Current serving cell
    current_cell = 0;
    
    if strcmpi(cell_type, "macro")
        if isfield(vehicle, 'serving_mcg')
            if isscalar(vehicle.serving_mcg)
                current_cell = vehicle.serving_mcg;
            end
        end
    elseif strcmpi(cell_type, "small")
        if isfield(vehicle, 'serving_scg')
            if isscalar(vehicle.serving_scg)
                current_cell = vehicle.serving_scg;
            end
        end
    end
    
    % Current cell RSRP
    current_rsrp = -inf;
    
    if isscalar(current_cell)
        if current_cell > 0
            if current_cell <= length(gNBs)
                [current_rsrp, current_sinr] = estimateCellPerformance(vehicle, gNBs(current_cell));
                
                % For current serving cell, check if SINR is degrading
                if isfield(vehicle, 'previous_sinr')
                    if isscalar(vehicle.previous_sinr) && isscalar(current_sinr)
                        sinr_delta = current_sinr - vehicle.previous_sinr;
                        
                        % If SINR is dropping rapidly, reduce hysteresis to allow earlier handover
                        if sinr_delta < -2 % SINR dropping by more than 2dB
                            hysteresis_margin = max(1, hysteresis_margin - 1);
                        end
                    end
                end
            end
        end
    end
    
    % At intersection - be more aggressive with cell selection
    at_intersection = false;
    if isfield(vehicle, 'position')
        if isnumeric(vehicle.position)
            dist_to_center = norm(vehicle.position);
            if dist_to_center < 50 % Within 50m of intersection
                at_intersection = true;
                % Reduce hysteresis at intersections for more responsive handovers
                hysteresis_margin = max(1.5, hysteresis_margin - 1);
            end
        end
    end
    
    % Evaluate all cells of the specified type
    for i = 1:length(gNBs)
        if strcmpi(gNBs(i).type, cell_type)
            % Estimate RSRP and SINR
            [cell_rsrp, cell_sinr] = estimateCellPerformance(vehicle, gNBs(i));
            
            % Only consider cells with minimum SINR
            min_sinr_threshold = -5; % Base threshold
            
            % More stringent SINR requirement at intersections
            if at_intersection
                min_sinr_threshold = 0;
            end
            
            if isscalar(cell_sinr)
                if cell_sinr > min_sinr_threshold
                    % Apply hysteresis for current cell
                    effective_rsrp = cell_rsrp;
                    
                    if isscalar(current_cell)
                        if i == current_cell
                            effective_rsrp = effective_rsrp + hysteresis_margin;
                        end
                    end
                    
                    % Update best cell if this one is better
                    if isscalar(effective_rsrp)
                        if isscalar(best_rsrp)
                            if effective_rsrp > best_rsrp
                                best_rsrp = effective_rsrp;
                                best_cell = i;
                            end
                        end
                    end
                end
            end
        end
    end
    
    % Apply handover margin to prevent handovers for marginal gain
    if isscalar(best_cell) && isscalar(current_cell)
        if best_cell ~= current_cell && current_cell > 0
            % Only handover if gain is significant
            if isscalar(best_rsrp) && isscalar(current_rsrp)
                if best_rsrp < (current_rsrp + hysteresis_margin)
                    best_cell = current_cell; % Stay with current cell
                end
            end
        end
    end
end

function [rsrp, sinr] = estimateCellPerformance(vehicle, cell)
    % Estimate RSRP and SINR for a specific cell
    
    % Calculate distance
    distance = 100; % Default if calculation fails
    
    try
        if isfield(vehicle, 'position') && isfield(cell, 'position')
            pos1 = vehicle.position;
            pos2 = cell.position;
            
            if isnumeric(pos1) && isnumeric(pos2) && (length(pos1) == length(pos2))
                distance = norm(pos1 - pos2);
            end
        end
    catch
        % Use default distance if calculation fails
    end
    
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
    noise_floor = -100; % Typical noise floor in dBm
    interference = -110 + 10 * log10(cell.load / 100 + 0.1); % Interference increases with load
    
    sinr = rsrp - 10 * log10(10^(noise_floor/10) + 10^(interference/10));
end