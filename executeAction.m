%% executeAction.m - Execute handover or link selection action
function [vehicle, handoverStats] = executeAction(vehicle, action, gNBs)
    % Execute the action decided by the RL agent
    
    % Initialize handover statistics
    handoverStats = struct('type', 0, 'isPingPong', false, 'interfaceSwitch', false);
    
    % Process action
    switch action
        case 1 % MCG Handover
            % Find best macro cell based on RSRP
            best_macro = findBestCell(vehicle, gNBs, "macro");
            
            % Check if this is a different cell
            if best_macro ~= vehicle.serving_mcg
                % Track if this is a ping-pong handover
                if ~isempty(vehicle.handover_history) && ...
                   vehicle.handover_history(end).cell == best_macro && ...
                   vehicle.handover_history(end).type == "MCG"
                    handoverStats.isPingPong = true;
                end
                
                % Execute handover
                prev_cell = vehicle.serving_mcg;
                vehicle.serving_mcg = best_macro;
                
                % Record handover in history
                vehicle.handover_history = [vehicle.handover_history; ...
                    struct('time', now(), 'type', "MCG", 'from', prev_cell, 'to', best_macro)];
                
                handoverStats.type = 1;  % MCG handover
            end
            
        case 2 % SCG Add
            % Only if no SCG is currently active
            if vehicle.serving_scg == 0
                % Find best small cell
                best_small = findBestCell(vehicle, gNBs, "small");
                
                % Add SCG if found
                if best_small > 0
                    vehicle.serving_scg = best_small;
                    
                    % Record in history
                    vehicle.handover_history = [vehicle.handover_history; ...
                        struct('time', now(), 'type', "SCG_Add", 'from', 0, 'to', best_small)];
                    
                    handoverStats.type = 2;  % SCG add
                end
            end
            
        case 3 % SCG Remove
            % Only if SCG is active
            if vehicle.serving_scg > 0
                prev_scg = vehicle.serving_scg;
                vehicle.serving_scg = 0;
                
                % Record in history
                vehicle.handover_history = [vehicle.handover_history; ...
                    struct('time', now(), 'type', "SCG_Remove", 'from', prev_scg, 'to', 0)];
                
                handoverStats.type = 3;  % SCG remove
            end
            
        case 4 % Switch Safety to Uu
            if ~strcmp(vehicle.safety_interface, 'Uu')
                vehicle.safety_interface = 'Uu';
                handoverStats.interfaceSwitch = true;
                handoverStats.type = 4;  % Interface switch
            end
            
        case 5 % Switch Safety to PC5
            if ~strcmp(vehicle.safety_interface, 'PC5')
                vehicle.safety_interface = 'PC5';
                handoverStats.interfaceSwitch = true;
                handoverStats.type = 5;  % Interface switch
            end
            
        case 6 % Switch Non-Safety to Uu
            if ~strcmp(vehicle.nonsafety_interface, 'Uu')
                vehicle.nonsafety_interface = 'Uu';
                handoverStats.interfaceSwitch = true;
                handoverStats.type = 6;  % Interface switch
            end
            
        case 7 % Switch Non-Safety to PC5
            if ~strcmp(vehicle.nonsafety_interface, 'PC5')
                vehicle.nonsafety_interface = 'PC5';
                handoverStats.interfaceSwitch = true;
                handoverStats.type = 7;  % Interface switch
            end
            
        case 8 % No Change
            % Do nothing
            
        otherwise
            warning('Unknown action: %d', action);
    end
end

function best_cell = findBestCell(vehicle, gNBs, cell_type)
    % Find the best cell of the specified type based on RSRP
    
    best_rsrp = -inf;
    best_cell = 0;
    
    for i = 1:length(gNBs)
        if strcmp(gNBs(i).type, cell_type)
            % Calculate distance
            dist = norm(vehicle.position - gNBs(i).position);
            
            % Simplified RSRP calculation
            % In a real implementation, this would use the actual RSRP measurements
            if strcmp(cell_type, "macro")
                % Higher power for macro
                rsrp = gNBs(i).power - 20*log10(dist/10) - 20;
            else
                % Lower power for small cells
                rsrp = gNBs(i).power - 20*log10(dist/10) - 15;
            end
            
            if rsrp > best_rsrp
                best_rsrp = rsrp;
                best_cell = i;
            end
        end
    end
end