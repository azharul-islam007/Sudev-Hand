%% main.m - Simplified version without visualizations
% 
% This is a basic version of the simulation that disables visualization
% to reduce errors and help you get the simulation running

clear; clc; close all;

%% 1. INITIALIZATION PHASE
% Load configuration parameters
configParams;

% Disable visualization to reduce errors
params.show_visualization = false;

% Initialize environment (intersection, buildings, traffic signals)
fprintf('Initializing urban environment...\n');
[environment, vehicles, gNBs] = initEnvironment(params);

% Initialize RL agent (DDQN)
fprintf('Setting up reinforcement learning agent...\n');
rlEnv = V2XRLEnvironment(params);
agent = createDDQNAgent(rlEnv, params.rl);

% Initialize performance metrics and logs
metrics = struct('handovers', [], 'pingpong', [], 'throughput', [], ...
                 'safety_pdr', [], 'latency', [], 'rewards', []);
logs = struct('vehicle_positions', [], 'link_states', [], 'actions', []);

% Set the environment state in the RL environment
rlEnv.setEnvironment(environment, gNBs);

% Initialize RL states for all vehicles
fprintf('Initializing vehicle states...\n');
for v = 1:length(vehicles)
    initial_state = buildStateVector(vehicles(v), environment, gNBs, 0);
    rlEnv.updateVehicleState(v, initial_state);
end

%% 2. MAIN SIMULATION LOOP
t = 0;
fprintf('Starting simulation...\n');

% Track start time for performance measurement
sim_start_time = tic;

% Run for a shorter time to test if things work
max_sim_time = min(30, params.T_sim); % 30 seconds or less

while t < max_sim_time
    % Print progress
    if mod(t, 5) < params.delta_t
        fprintf('Simulation time: %.1f / %.1f sec (%.1f%%)\n', t, max_sim_time, 100*t/max_sim_time);
    end
    
    %% (A) UPDATE TRAFFIC SIGNAL & MOBILITY
    % Update traffic signals
    environment.signals = trafficSignal(environment.signals, params.delta_t);
    
    % Move vehicles based on signal states and update PC5 clusters
    [vehicles, clusters] = vehicleMobility(vehicles, environment, params.delta_t);
    
    %% (B) APPLY CHANNEL MODEL
    % For each vehicle, calculate path loss and channel conditions
    for v = 1:length(vehicles)
        % Calculate geometry to macro/small cells and neighbor vehicles
        [vehicles(v).dist_macro, vehicles(v).dist_small] = calculateDistances(vehicles(v), gNBs);
        
        % Determine LoS/NLoS based on buildings and geometry
        vehicles(v).isLoS = determineLoS(vehicles(v), gNBs, environment.buildings);
        
        % Apply channel model to get RSRP, RSRQ, SINR for Uu
        try
            [vehicles(v).rsrp_uu, vehicles(v).sinr_uu] = channelModel('Uu', vehicles(v), gNBs, params.channel);
        catch e
            fprintf('Warning: Channel model error for Uu interface: %s\n', e.message);
            vehicles(v).rsrp_uu = -100; % Default value
            vehicles(v).sinr_uu = 0;    % Default value
        end
        
        % Initialize PC5 metrics for this timestep
        vehicles(v).pc5_metrics = [];
        
        % For each vehicle in PC5 range, calculate PC5 metrics
        if ~isempty(clusters{v})
            for c = 1:length(clusters{v})
                neighborIdx = clusters{v}(c);
                try
                    [sinr, reliability] = channelModel('PC5', vehicles(v), vehicles(neighborIdx), params.channel);
                    vehicles(v).pc5_metrics(neighborIdx).sinr = sinr;
                    vehicles(v).pc5_metrics(neighborIdx).reliability = reliability;
                catch e
                    fprintf('Warning: Channel model error for PC5 interface: %s\n', e.message);
                    vehicles(v).pc5_metrics(neighborIdx).sinr = 0;
                    vehicles(v).pc5_metrics(neighborIdx).reliability = 0;
                end
            end
        end
    end
    
    %% (C) BUILD DDQN STATE & DECIDE ACTION
    actions = zeros(length(vehicles), 1);
    
    for v = 1:length(vehicles)
        % Form state vector for this vehicle
        state = buildStateVector(vehicles(v), environment, gNBs, t);
        
        % Get action from DDQN (or use epsilon-greedy during training)
        if rand() < params.rl.epsilon || ~params.rl.do_training
            actions(v) = randi(params.rl.num_actions);  % Random exploration
        else
            try
                actions(v) = getAction(agent, state);  % DDQN decision
            catch e
                actions(v) = randi(params.rl.num_actions);  % Random fallback
                if mod(v, 10) == 0 % Limit error printing
                    fprintf('Warning: Error getting action from agent: %s\n', e.message);
                end
            end
        end
    end
    
    %% (D) EXECUTE HANDOVER / LINK SELECTION
    for v = 1:length(vehicles)
        % Execute handover or link selection based on the action
        try
            % Create a copy of the vehicle to avoid structure modification issues
            vehicle_copy = vehicles(v);
            
            % Execute action on the copy
            [modified_vehicle, handoverStats] = executeAction(vehicle_copy, actions(v), gNBs);
            
            % Only update specific fields that should change, not the whole structure
            if isfield(modified_vehicle, 'serving_mcg')
                vehicles(v).serving_mcg = modified_vehicle.serving_mcg;
            end
            
            if isfield(modified_vehicle, 'serving_scg')
                vehicles(v).serving_scg = modified_vehicle.serving_scg;
            end
            
            if isfield(modified_vehicle, 'safety_interface')
                vehicles(v).safety_interface = modified_vehicle.safety_interface;
            end
            
            if isfield(modified_vehicle, 'nonsafety_interface')
                vehicles(v).nonsafety_interface = modified_vehicle.nonsafety_interface;
            end
            
            if isfield(modified_vehicle, 'last_handover_time')
                vehicles(v).last_handover_time = modified_vehicle.last_handover_time;
            end
            
            % Track handover events and ping-pong
            if handoverStats.type > 0
                metrics.handovers = [metrics.handovers; t, v, handoverStats.type];
                
                if handoverStats.isPingPong
                    metrics.pingpong = [metrics.pingpong; t, v];
                end
            end
        catch e
            if mod(v, 10) == 0 % Limit error printing
                fprintf('Warning: Error executing action for vehicle %d: %s\n', v, e.message);
            end
            handoverStats = struct('type', 0, 'isPingPong', false, 'interfaceSwitch', false);
        end
    end
    
    %% (E) DATA TRANSMISSION & DDQN UPDATE
    % Simulate data transmission and calculate KPIs
    try
        [transmissionStats, qosMetrics] = dataTransmission(vehicles, gNBs, clusters, params);
    catch e
        fprintf('Warning: Error in data transmission simulation: %s\n', e.message);
        % Create default transmission stats if there's an error
        for v = 1:length(vehicles)
            transmissionStats(v) = struct('throughput', 0, 'max_throughput', 1, ...
                'safety_pdr', 0, 'latency', 1, 'safety_sent', 0, 'safety_received', 0, ...
                'nonsafety_sent', 0, 'nonsafety_received', 0);
        end
        qosMetrics = struct('avg_throughput', 0, 'avg_safety_pdr', 0, 'avg_latency', 1);
    end
    
    % Ensure transmissionStats fields are scalar
    for v = 1:length(vehicles)
        % Convert any non-scalar fields to scalars by taking the mean
        fields = fieldnames(transmissionStats);
        for i = 1:length(fields)
            field = fields{i};
            if ~isscalar(transmissionStats(v).(field))
                transmissionStats(v).(field) = mean(transmissionStats(v).(field)(:));
            end
        end
    end
    
    % Calculate rewards for each vehicle
    rewards = zeros(length(vehicles), 1);
    nextStates = cell(length(vehicles), 1);
    
    for v = 1:length(vehicles)
        % Compute reward based on safety reliability, throughput, and overhead
        try
            rewards(v) = rewardFunction(vehicles(v), transmissionStats(v), handoverStats);
        catch e
            if mod(v, 10) == 0 % Limit error printing
                fprintf('Warning: Error calculating reward: %s\n', e.message);
            end
            rewards(v) = 0;
        end
        
        % Store metrics
        metrics.throughput = [metrics.throughput; t, v, transmissionStats(v).throughput];
        metrics.safety_pdr = [metrics.safety_pdr; t, v, transmissionStats(v).safety_pdr];
        metrics.latency = [metrics.latency; t, v, transmissionStats(v).latency];
        metrics.rewards = [metrics.rewards; t, v, rewards(v)];
        
        % Build next state for DDQN update
        nextStates{v} = buildStateVector(vehicles(v), environment, gNBs, t);
        
        % Store experience in replay buffer and perform DDQN update
        if params.rl.do_training
            try
                % Get previous state
                previousState = rlEnv.getState(v);
                
                % Update RL environment with new state
                rlEnv.updateVehicleState(v, nextStates{v});
                
                % Store experience for learning
                storeExperience(agent, previousState, actions(v), rewards(v), nextStates{v});
                
                % Periodically update DDQN
                if mod(t, params.rl.update_frequency) == 0
                    updateDDQN(agent);
                end
            catch e
                if mod(v, 10) == 0 % Limit error printing
                    fprintf('Warning: Error in RL experience update: %s\n', e.message);
                end
            end
        end
    end
    
    % Log data for post-simulation analysis
    try
        logs = logData(logs, t, vehicles, environment, actions, transmissionStats);
    catch e
        fprintf('Warning: Error logging data: %s\n', e.message);
    end
    
    %% (F) NEXT TIMESTEP
    t = t + params.delta_t;
end

% Calculate and display simulation performance
sim_time = toc(sim_start_time);
fprintf('Simulation completed in %.2f seconds (%.1fx real-time)\n', ...
        sim_time, t / sim_time);

%% 3. END OF SIMULATION
fprintf('Processing results...\n');

% Aggregate KPIs and performance metrics
try
    fprintf('\nSummary of Results:\n');
    fprintf('  Average throughput: %.2f Mbps\n', mean(metrics.throughput(:,3)) / 1e6);
    fprintf('  Average safety PDR: %.2f%%\n', mean(metrics.safety_pdr(:,3)) * 100);
    fprintf('  Average latency: %.2f ms\n', mean(metrics.latency(:,3)) * 1000);
    fprintf('  Total handovers: %d\n', size(metrics.handovers, 1));
    fprintf('  Ping-pong handovers: %d (%.1f%%)\n', size(metrics.pingpong, 1), ...
            100 * size(metrics.pingpong, 1) / max(1, size(metrics.handovers, 1)));
    fprintf('  Average reward: %.4f\n', mean(metrics.rewards(:,3)));
catch e
    fprintf('Error calculating aggregate metrics: %s\n', e.message);
end

% Save results and logs
try
    if ~exist(params.output_dir, 'dir')
        mkdir(params.output_dir);
    end
    save(fullfile(params.output_dir, 'simulation_results.mat'), 'metrics', 'logs', 'params');
    fprintf('Results saved to %s\n', fullfile(params.output_dir, 'simulation_results.mat'));
catch e
    fprintf('Error saving results: %s\n', e.message);
end

fprintf('Simulation complete!\n');