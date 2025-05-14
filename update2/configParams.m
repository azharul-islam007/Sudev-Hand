%% configParams.m - Optimized simulation parameters

% Simulation control
params.T_sim = 300;      % Total simulation time (seconds)
params.delta_t = 0.1;    % Time step (seconds)
params.seed = 42;        % Random seed for reproducibility
params.output_dir = 'results';  % Directory to save results
params.run_baseline = true;
params.baseline_file = 'baseline_results.mat';
params.show_visualization = true;  % Enable real-time visualization
params.create_animation = false;   % Create animation at the end (computationally expensive)

% Environment parameters
params.env.intersection_size = [500, 500];  % Size of intersection area (m)
params.env.num_lanes = 2;      % Lanes per direction
params.env.lane_width = 3.5;   % Lane width (m)
params.env.building_density = 0.3;  % Building density
params.env.traffic_light_cycle = [30, 30];  % [green, red] duration (seconds)

% Vehicle parameters
params.vehicle.count = 50;     % Number of vehicles
params.vehicle.max_speed = 14; % Maximum speed (m/s) ~50 km/h
params.vehicle.queue_spacing = 2; % Spacing between queued vehicles (m)
params.vehicle.arrival_rate = 0.2; % Poisson arrival rate (vehicles/second)

% Cellular network parameters - Optimized for 5G NR
params.network.macro_count = 1;     % Number of macro cells
params.network.small_count = 4;     % Number of small cells
params.network.macro_power = 46;    % Macro cell transmit power (dBm)
params.network.small_power = 33;    % Small cell transmit power (dBm) - increased from 30
params.network.macro_height = 25;   % Macro cell height (m)
params.network.small_height = 10;   % Small cell height (m)
params.network.ue_power = 23;       % UE transmit power (dBm)
params.network.ue_height = 1.5;     % UE height (m)
params.network.carrier_frequency = 3.5e9;  % Carrier frequency (Hz) - 3.5 GHz for 5G FR1
params.network.bandwidth = 100e6;   % Bandwidth (Hz) - 100 MHz 5G NR bandwidth

% Channel model parameters - Improved accuracy based on 3GPP models
params.channel.model = 'WINNER+';   % Channel model ('WINNER+' or '3GPP_V2X')
params.channel.scenario = 'UMi';    % Urban Micro
params.channel.pathloss_exponent_los = 2.2;   % Path loss exponent for LoS
params.channel.pathloss_exponent_nlos = 3.5;  % Path loss exponent for NLoS (improved from 3.8)
params.channel.shadow_std_los = 3;  % Shadow fading std dev for LoS (dB) - reduced from 4
params.channel.shadow_std_nlos = 6; % Shadow fading std dev for NLoS (dB)
params.channel.noise_figure = 7;    % UE noise figure (dB) - improved from 9
params.channel.thermal_noise = -174; % Thermal noise density (dBm/Hz)
params.channel.k_factor_los = 10;   % Rician K-factor for LoS
params.channel.correlation_distance = 50; % Shadow fading correlation distance (m)

% Data transmission parameters - Realistic 5G V2X values
params.data.safety_size = 300;      % Safety message size (bytes) - increased from 200
params.data.safety_interval = 0.1;  % Safety message interval (seconds) - 10 Hz
params.data.nonsafety_size = 1500;  % Non-safety message size (bytes)
params.data.nonsafety_rate = 5e6;   % Non-safety data rate (bps) - increased from 2 Mbps
params.data.safety_latency_req = 0.1; % Safety message latency requirement (s) - 100 ms
params.data.reliability_req = 0.9;  % Safety message reliability requirement (90% PDR)

% SINR thresholds for MCS selection (simplified 5G NR MCS table)
params.sinr.mcs = [-5, 0, 5, 10, 15, 20, 25, 30];  % SINR thresholds (dB)
params.sinr.efficiency = [0.2, 0.4, 0.8, 1.5, 2.5, 4.0, 5.5, 6.5];  % Spectral efficiency (bps/Hz)

% Handover parameters - Optimized to reduce ping-pong
params.handover.rsrp_threshold = -110;  % RSRP threshold for handover (dBm)
params.handover.hysteresis = 3;        % Handover hysteresis margin (dB)
params.handover.time_to_trigger = 1.0;  % Time-to-trigger for handover (s)
params.handover.pingpong_time = 5.0;    % Time threshold for ping-pong detection (s)
params.handover.a3_offset = 2;         % A3 event offset (dB)

% PC5 parameters - Optimized V2X sidelink
params.pc5.frequency = 5.9e9;      % PC5 carrier frequency (Hz) - 5.9 GHz for ITS
params.pc5.bandwidth = 20e6;       % PC5 bandwidth (Hz) - 20 MHz
params.pc5.transmit_power = 23;    % PC5 transmit power (dBm)
params.pc5.resource_pool_size = 20; % Number of resource blocks in pool
params.pc5.max_range = 400;        % Maximum PC5 range (m)
params.pc5.reliable_range = 250;   % Reliable PC5 range (m)
params.pc5.sensing_gain = 3;       % Sensing-based resource selection gain (dB)

% RL parameters - Optimized for DDQN
params.rl.state_dim = 10;           % State dimension
params.rl.num_actions = 8;           % Number of actions
params.rl.hidden_layers = [128, 128]; % Hidden layer sizes - increased from [64, 64]
params.rl.learning_rate = 5e-4;     % Learning rate
params.rl.discount = 0.95;          % Discount factor
params.rl.epsilon = 0.2;            % Initial exploration rate - increased from 0.1
params.rl.epsilon_decay = 0.995;    % Epsilon decay rate
params.rl.epsilon_min = 0.05;       % Minimum exploration rate
params.rl.buffer_size = 50000;      % Replay buffer size - increased from 10000
params.rl.batch_size = 64;          % Mini-batch size - increased from 32
params.rl.target_update = 500;      % Target network update frequency
params.rl.do_training = true;       % Enable training
params.rl.update_frequency = 10;    % Update frequency (time steps)

% Reward function weights - Prioritize safety
params.rl.reward.w_safety = 0.7;    % Weight for safety message reliability - increased from 0.6
params.rl.reward.w_throughput = 0.2; % Weight for throughput - decreased from 0.3
params.rl.reward.w_overhead = 0.1;  % Weight for handover overhead
params.rl.reward.safety_threshold = 0.9; % PDR threshold for full safety reward
params.rl.reward.latency_target = 0.1;  % Target latency for safety messages (s)
params.rl.reward.handover_penalty = 1.0; % Penalty for handovers
params.rl.reward.pingpong_penalty = 5.0; % Severe penalty for ping-pong handovers

% Print configuration summary
fprintf('Loaded optimized configuration with:\n');
fprintf('  - %d vehicles in urban intersection\n', params.vehicle.count);
fprintf('  - %d macro cells, %d small cells\n', params.network.macro_count, params.network.small_count);
fprintf('  - %s channel model with %s scenario\n', params.channel.model, params.channel.scenario);
fprintf('  - %.1f GHz carrier frequency, %.0f MHz bandwidth\n', params.network.carrier_frequency/1e9, params.network.bandwidth/1e6);
fprintf('  - %.1f s simulation time, %.3f s time step\n', params.T_sim, params.delta_t);
fprintf('  - DDQN with %d state dimensions, %d actions\n', params.rl.state_dim, params.rl.num_actions);