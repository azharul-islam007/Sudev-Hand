%% testPingPong.m - Test function to verify ping-pong detection
function testPingPong()
    % Create a test vehicle
    vehicle = struct();
    vehicle.serving_mcg = 1;
    vehicle.handover_times = [];
    vehicle.handover_types = {};
    vehicle.handover_from = [];
    vehicle.handover_to = [];
    vehicle.previous_cell_times = [];
    vehicle.previous_cell_ids = [];
    vehicle.last_handover_time = 0;
    
    % Create test gNBs
    gNBs(1) = struct('type', "macro", 'position', [0,0], 'power', 46, 'height', 25, 'load', 0);
    gNBs(2) = struct('type', "macro", 'position', [100,100], 'power', 46, 'height', 25, 'load', 0);
    
    % Test scenario:
    % 1. Handover from cell 1 to cell 2
    fprintf('Handover: 1->2\n');
    [vehicle, stats1] = doTestHandover(vehicle, 1, 2, now()*86400);
    fprintf('isPingPong: %d\n', stats1.isPingPong);
    
    % Wait a short time
    pause(1);
    
    % 2. Handover back from cell 2 to cell 1 (should be ping-pong)
    fprintf('Handover: 2->1\n');
    [vehicle, stats2] = doTestHandover(vehicle, 2, 1, now()*86400);
    fprintf('isPingPong: %d\n', stats2.isPingPong);
end

function [vehicle, handoverStats] = doTestHandover(vehicle, from_cell, to_cell, current_time)
    % Initialize handover statistics
    handoverStats = struct('type', 0, 'isPingPong', false, 'interfaceSwitch', false);
    
    % Check for ping-pong: returning to a recent cell
    isPingPong = false;
    
    % Check previous cells
    for i = 1:length(vehicle.previous_cell_ids)
        if vehicle.previous_cell_ids(i) == to_cell
            % Check how recent this cell was
            if (current_time - vehicle.previous_cell_times(i)) < 30
                isPingPong = true;
                break;
            end
        end
    end
    
    handoverStats.isPingPong = isPingPong;
    
    % Execute the handover
    prev_cell = vehicle.serving_mcg;
    vehicle.serving_mcg = to_cell;
    
    % Update handover time
    vehicle.last_handover_time = current_time;
    
    % Record handover in history
    vehicle.handover_times = [vehicle.handover_times; current_time];
    vehicle.handover_types{end+1} = "MCG";
    vehicle.handover_from = [vehicle.handover_from; prev_cell];
    vehicle.handover_to = [vehicle.handover_to; to_cell];
    
    % Update previous cells list
    vehicle.previous_cell_times = [vehicle.previous_cell_times; current_time];
    vehicle.previous_cell_ids = [vehicle.previous_cell_ids; prev_cell];
    
    handoverStats.type = 1;  % MCG handover
end