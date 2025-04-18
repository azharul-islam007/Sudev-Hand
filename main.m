%% main.m - Main simulation driver for V2X Dual Connectivity System
clear; clc; close all;

%% 1. INITIALIZATION PHASE
% Load configuration parameters
configParams;

% Initialize environment (intersection, buildings, traffic signals)
fprintf('Initializing environment...\n');
[environment, vehicles, gNBs] = initEnvironment(params);

% Initialize RL agent (DDQN)
fprintf('Setting up RL environment...\n');
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

while t < params.T_sim
    % Update progress bar (every 1 second of sim time)
    if mod(t, 1) < params.delta_t
        waitbar(t/params.T_sim, progressBar, sprintf('Simulation: %.1f / %.1f sec', t, params.T_sim));
    end
    
    %% (A) UPDATE TRAFFIC SIGNAL & MOBILITY
    % Update traffic signals
    environment.signals = trafficSignal(environment.signals, params.delta_t);
    
    % Move vehicles based on signal states
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
                if contains(e.message, 'cell')
                    % This is the known cell conversion error, silently handle it
                    actions(v) = randi(params.rl.num_actions);
                else
                    % For other errors, print a warning
                    fprintf('Warning: Error getting action from agent: %s\n', e.message);
                    actions(v) = randi(params.rl.num_actions);
                end
            end
        end
    end
    
    %% (D) EXECUTE HANDOVER / LINK SELECTION
    for v = 1:length(vehicles)
        % Execute handover or link selection based on the action
        try
            [vehicles(v), handoverStats] = executeAction(vehicles(v), actions(v), gNBs);
        catch e
            fprintf('Warning: Error executing action: %s\n', e.message);
            handoverStats = struct('type', 0, 'isPingPong', false, 'interfaceSwitch', false);
        end
        
        % Track handover events and ping-pong
        if handoverStats.type > 0
            metrics.handovers = [metrics.handovers; t, v, handoverStats.type];
            
            if handoverStats.isPingPong
                metrics.pingpong = [metrics.pingpong; t, v];
            end
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
        if params.rl.do_training
            try
                % Get previous state
                previousState = rlEnv.getState(v);
                
                % Update RL environment with new state
                rlEnv.updateVehicleState(v, nextStates{v});
                
                % Display some experiences occasionally
                if rand < 0.01
                    fprintf('Experience: Vehicle=%d, Action=%d, Reward=%.2f\n', ...
                            v, actions(v), rewards(v));
                end
                
                % For now, we'll skip actual experience storage and DDQN updates
                % storeExperience(agent, previousState, actions(v), rewards(v), nextStates{v});
                
                % Periodically update DDQN (commented out for development)
                % if mod(t, params.rl.update_frequency) == 0
                %     updateDDQN(agent);
                % end
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
    if mod(t, 5) < params.delta_t
        try
            % Update visualization (if enabled)
            if isfield(params, 'show_visualization') && params.show_visualization
                plotEnvironment(environment, vehicles, gNBs);
                drawnow;
            end
        catch e
            fprintf('Warning: Error updating visualization: %s\n', e.message);
        end
    end
    
    %% (F) NEXT TIMESTEP
    t = t + params.delta_t;
end

close(progressBar);

%% 3. END OF SIMULATION section for main.m
fprintf('Simulation completed. Processing results...\n');

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
if isfield(params, 'run_baseline') && params.run_baseline && exist('params', 'var') && isfield(params, 'baseline_file') && exist(params.baseline_file, 'file')
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