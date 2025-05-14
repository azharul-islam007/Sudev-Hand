%% main.m - Optimized V2X Dual Connectivity Simulation
% 
% This script implements a comprehensive 5G V2X simulation with dual connectivity
% (Uu and PC5) in an urban environment. It uses reinforcement learning (DDQN)
% to optimize handover and interface selection decisions.

clear; clc; close all;

%% 1. INITIALIZATION PHASE
% Load configuration parameters
configParams;

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
progressBar = waitbar(0, 'Simulation in progress...');

% Track start time for performance measurement
sim_start_time = tic;

% Debug counters for ping-pong detection
debug_handover_count = 0;
debug_pingpong_count = 0;

while t < params.T_sim
    % Update progress bar (every 1 second of sim time)
    if mod(t, 1) < params.delta_t
        % Update progress bar text and position
        waitbar(t/params.T_sim, progressBar);
        
        % Update the waitbar title separately
        set(progressBar, 'Name', sprintf('Simulation: %.1f / %.1f sec', t, params.T_sim));
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
        
        % Apply channel model to get RSRP, SINR for Uu
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
        do_training = false;
        if isfield(params.rl, 'do_training')
            do_training = params.rl.do_training;
        end
        
        if rand() < params.rl.epsilon || ~do_training
            actions(v) = randi(params.rl.num_actions);  % Random exploration
        else
            try
                actions(v) = getAction(agent, state);  % DDQN decision
            catch e
                fprintf('Warning: Error getting action from agent: %s\n', e.message);
                actions(v) = randi(params.rl.num_actions);
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
            
            if isfield(modified_vehicle, 'handover_times')
                vehicles(v).handover_times = modified_vehicle.handover_times;
            end
            
            if isfield(modified_vehicle, 'handover_types')
                vehicles(v).handover_types = modified_vehicle.handover_types;
            end
            
            if isfield(modified_vehicle, 'handover_from')
                vehicles(v).handover_from = modified_vehicle.handover_from;
            end
            
            if isfield(modified_vehicle, 'handover_to')
                vehicles(v).handover_to = modified_vehicle.handover_to;
            end
            
            if isfield(modified_vehicle, 'previous_cell_times')
                vehicles(v).previous_cell_times = modified_vehicle.previous_cell_times;
            end
            
            if isfield(modified_vehicle, 'previous_cell_ids')
                vehicles(v).previous_cell_ids = modified_vehicle.previous_cell_ids;
            end
            
            % Track handover events and ping-pong with extra debugging
            handover_type = 0;
            is_ping_pong = false;
            
            % Check if handoverStats has the right fields
            if isfield(handoverStats, 'type')
                % Check if type is scalar
                if isscalar(handoverStats.type)
                    handover_type = handoverStats.type;
                end
            end
            
            if isfield(handoverStats, 'isPingPong')
                % Check if isPingPong is scalar
                if isscalar(handoverStats.isPingPong)
                    is_ping_pong = handoverStats.isPingPong;
                end
            end
            
            % Record handover if it occurred
            if handover_type > 0
                metrics.handovers = [metrics.handovers; t, v, handover_type];
                debug_handover_count = debug_handover_count + 1;
                
                % Debug output for handovers
                fprintf('DEBUG: Handover at t=%.1f, vehicle=%d, type=%d\n', t, v, handover_type);
                
                % Check for ping-pong
                if is_ping_pong
                    metrics.pingpong = [metrics.pingpong; t, v];
                    debug_pingpong_count = debug_pingpong_count + 1;
                    
                    % Debug output for ping-pong detection
                    fprintf('DEBUG: Ping-pong detected at t=%.1f, vehicle=%d\n', t, v);
                end
            end
        catch e
            % If there's still an error, just log it and continue with next vehicle
            fprintf('Warning: Error executing action for vehicle %d: %s\n', v, e.message);
            disp(getReport(e));
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
            fprintf('Warning: Error calculating reward: %s\n', e.message);
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
        do_training = false;
        if isfield(params.rl, 'do_training')
            do_training = params.rl.do_training;
        end
        
        if do_training
            try
                % Get previous state
                previousState = rlEnv.getState(v);
                
                % Update RL environment with new state
                rlEnv.updateVehicleState(v, nextStates{v});
                
                % Store experience for learning
                storeExperience(agent, previousState, actions(v), rewards(v), nextStates{v});
                
                % Periodically update DDQN
                update_freq = 10; % Default update frequency
                if isfield(params.rl, 'update_frequency')
                    update_freq = params.rl.update_frequency;
                end
                
                if mod(t, update_freq) == 0
                    updateDDQN(agent);
                end
                
                % Display some experiences occasionally
                if rand < 0.001  % Less frequent logging (0.1% chance)
                    fprintf('Experience: Vehicle=%d, Action=%d, Reward=%.2f, Safety PDR=%.2f, Latency=%.1fms\n', ...
                            v, actions(v), rewards(v), transmissionStats(v).safety_pdr, ...
                            transmissionStats(v).latency * 1000);
                end
            catch e
                fprintf('Warning: Error in RL experience update: %s\n', e.message);
            end
        end
    end
    
    % Log data for post-simulation analysis
    try
        logs = logData(logs, t, vehicles, environment, actions, transmissionStats);
    catch e
        fprintf('Warning: Error logging data: %s\n', e.message);
    end
    
    % Periodically update visualization
    show_viz = false;
    if isfield(params, 'show_visualization')
        show_viz = params.show_visualization;
    end
    
    if show_viz && mod(t, 2) < params.delta_t
        try
            plotEnvironment(environment, vehicles, gNBs);
            drawnow;
        catch e
            fprintf('Warning: Error updating visualization: %s\n', e.message);
        end
    end
    
    %% (F) NEXT TIMESTEP
    t = t + params.delta_t;
end

% Print debug counters
fprintf('DEBUG: Total handovers: %d, Total ping-pongs: %d\n', debug_handover_count, debug_pingpong_count);

% Calculate and display simulation performance
sim_time = toc(sim_start_time);
fprintf('Simulation completed in %.2f seconds (%.1fx real-time)\n', ...
        sim_time, params.T_sim / sim_time);

close(progressBar);

%% 3. END OF SIMULATION
fprintf('Processing results...\n');

% Aggregate KPIs and performance metrics
try
    aggregateMetrics = calculateAggregateMetrics(metrics);
    
    % Print summary
    fprintf('\nSummary of Results:\n');
    fprintf('  Average throughput: %.2f Mbps\n', aggregateMetrics.avg_throughput / 1e6);
    fprintf('  Average safety PDR: %.2f%%\n', aggregateMetrics.avg_safety_pdr * 100);
    fprintf('  Average latency: %.2f ms\n', aggregateMetrics.avg_latency * 1000);
    fprintf('  Total handovers: %d\n', aggregateMetrics.total_handovers);
    fprintf('  Ping-pong handovers: %d (%.1f%%)\n', aggregateMetrics.ping_pong_count, ...
            100 * aggregateMetrics.ping_pong_ratio);
    fprintf('  Average reward: %.4f\n', aggregateMetrics.avg_reward);
    
    % Add analysis of interface selection
    if isfield(aggregateMetrics, 'interface_stats')
        fprintf('  Interface usage at end of simulation:\n');
        fprintf('    Safety on Uu: %.1f%%, Safety on PC5: %.1f%%\n', ...
                aggregateMetrics.interface_stats.safety_uu_pct * 100, ...
                aggregateMetrics.interface_stats.safety_pc5_pct * 100);
        fprintf('    Non-safety on Uu: %.1f%%, Non-safety on PC5: %.1f%%\n', ...
                aggregateMetrics.interface_stats.nonsafety_uu_pct * 100, ...
                aggregateMetrics.interface_stats.nonsafety_pc5_pct * 100);
    end
catch e
    fprintf('Error calculating aggregate metrics: %s\n', e.message);
    
    % Create a default aggregateMetrics structure to prevent further errors
    aggregateMetrics = struct();
    aggregateMetrics.avg_throughput = 0;
    aggregateMetrics.avg_safety_pdr = 0;
    aggregateMetrics.avg_latency = 0;
    aggregateMetrics.total_handovers = size(metrics.handovers, 1);
    aggregateMetrics.ping_pong_count = size(metrics.pingpong, 1);
    aggregateMetrics.ping_pong_ratio = aggregateMetrics.ping_pong_count / max(1, aggregateMetrics.total_handovers);
    aggregateMetrics.avg_reward = 0;
end

% Compare with baseline (RSRP-only or no PC5 integration)
run_baseline = false;
has_baseline_file = false;

if isfield(params, 'run_baseline')
    run_baseline = params.run_baseline;
end

if isfield(params, 'baseline_file')
    baseline_file = params.baseline_file;
    has_baseline_file = exist(baseline_file, 'file') > 0;
end

if run_baseline && has_baseline_file
    try
        baselineMetrics = loadBaselineResults(params.baseline_file);
        comparisonResults = compareWithBaseline(aggregateMetrics, baselineMetrics);
        visualizeComparison(comparisonResults);
    catch e
        fprintf('Error comparing with baseline: %s\n', e.message);
    end
else
    fprintf('Skipping baseline comparison (not configured or baseline file not found)\n');
end

% Visualize results
try
    visualizeResults(metrics, logs, environment, params);
catch e
    fprintf('Error visualizing results: %s\n', e.message);
end

% Save results and logs
try
    if ~exist(params.output_dir, 'dir')
        mkdir(params.output_dir);
    end
    save(fullfile(params.output_dir, 'simulation_results.mat'), 'metrics', 'logs', 'aggregateMetrics', 'params');
    fprintf('Results saved to %s\n', fullfile(params.output_dir, 'simulation_results.mat'));
catch e
    fprintf('Error saving results: %s\n', e.message);
end

fprintf('Simulation complete!\n');

% The loadBaselineResults and compareWithBaseline functions would follow here
% but are omitted for brevity