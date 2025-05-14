%% buildStateVector.m - Build the state vector for RL
function state = buildStateVector(vehicle, environment, gNBs, t)
    % Build the state vector for RL decision making
    
    % Initialize state vector
    state = zeros(10, 1);
    
    % Check if this is the initialization phase
    initialization_phase = (t == 0);
    
    % 1. RSRP from serving macro cell (normalized)
    if initialization_phase || ~isfield(vehicle, 'rsrp_uu') || isempty(vehicle.rsrp_uu)
        state(1) = 0.5; % Default middle value during initialization
    else
        state(1) = (vehicle.rsrp_uu - (-120)) / ((-70) - (-120));  % Normalize to [0,1]
    end
    
    % 2. SINR from serving macro cell (normalized)
    if initialization_phase || ~isfield(vehicle, 'sinr_uu') || isempty(vehicle.sinr_uu)
        state(2) = 0.5; % Default middle value
    else
        state(2) = (vehicle.sinr_uu - (-10)) / (30 - (-10));  % Normalize to [0,1]
    end
    
    % 3. Average PC5 SINR (normalized)
    if initialization_phase || ~isfield(vehicle, 'pc5_metrics') || isempty(vehicle.pc5_metrics)
        state(3) = 0; % No PC5 links initially
    else
        pc5_sinrs = [];
        for i = 1:length(vehicle.pc5_metrics)
            if isfield(vehicle.pc5_metrics(i), 'sinr')
                pc5_sinrs = [pc5_sinrs, vehicle.pc5_metrics(i).sinr];
            end
        end
        
        if ~isempty(pc5_sinrs)
            avg_pc5_sinr = mean(pc5_sinrs);
            state(3) = (avg_pc5_sinr - (-5)) / (25 - (-5));  % Normalize to [0,1]
        else
            state(3) = 0;  % No PC5 links
        end
    end
    
    % 4. PC5 cluster size (normalized)
    if initialization_phase || ~isfield(vehicle, 'pc5_metrics') || isempty(vehicle.pc5_metrics)
        state(4) = 0; % No cluster initially
    else
        cluster_size = length(vehicle.pc5_metrics);
        state(4) = min(1, cluster_size / 10);  % Normalize: 10+ vehicles is max
    end
    
    % 5. Serving cell load (normalized)
    if initialization_phase || ~isfield(vehicle, 'serving_mcg') || vehicle.serving_mcg <= 0
        state(5) = 0;
    else
        serving_mcg_idx = vehicle.serving_mcg;
        if serving_mcg_idx > 0 && serving_mcg_idx <= length(gNBs)
            state(5) = gNBs(serving_mcg_idx).load / 100;  % Assuming load is percentage
        else
            state(5) = 0;
        end
    end
    
    % 6. Vehicle speed (normalized)
    if initialization_phase || ~isfield(vehicle, 'velocity') || isempty(vehicle.velocity)
        state(6) = 0.5; % Medium speed initially
    else
        speed = norm(vehicle.velocity);
        state(6) = speed / 14;  % Normalize to max speed
    end
    
    % 7. Safety message queue size (normalized)
    if initialization_phase || ~isfield(vehicle, 'safety_queue')
        state(7) = 0; % Empty queue initially
    else
        state(7) = min(1, vehicle.safety_queue / 10);  % Normalize: 10+ is max
    end
    
    % 8. Non-safety message queue size (normalized)
    if initialization_phase || ~isfield(vehicle, 'nonsafety_queue')
        state(8) = 0; % Empty queue initially
    else
        state(8) = min(1, vehicle.nonsafety_queue / 20);  % Normalize: 20+ is max
    end
    
    % 9. Traffic signal state (0: red, 1: green)
    if initialization_phase || ~isfield(vehicle, 'direction')
        state(9) = 0.5; % Unknown initially
    else
        if (strcmp(vehicle.direction, 'NS') || strcmp(vehicle.direction, 'SN'))
            state(9) = environment.signals.ns_state == "green";
        else
            state(9) = environment.signals.ew_state == "green";
        end
    end
    
    % 10. Vehicle stopped flag
    if initialization_phase || ~isfield(vehicle, 'stopped')
        state(10) = 0; % Not stopped initially
    else
        state(10) = vehicle.stopped;
    end
    
    % Ensure all values are within [0,1]
    state = min(1, max(0, state));
end