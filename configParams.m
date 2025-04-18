%% configParams.m - Simulation parameters

% Simulation control
params.T_sim = 300;      % Total simulation time (seconds)
params.delta_t = 0.1;    % Time step (seconds)
params.seed = 42;        % Random seed for reproducibility
params.output_dir = 'results';  % Directory to save results
params.run_baseline = true;
params.baseline_file = 'baseline_results.mat';

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

% Cellular network parameters
params.network.macro_count = 1;     % Number of macro cells
params.network.small_count = 4;     % Number of small cells
params.network.macro_power = 46;    % Macro cell transmit power (dBm)
params.network.small_power = 30;    % Small cell transmit power (dBm)
params.network.macro_height = 25;   % Macro cell height (m)
params.network.small_height = 10;   % Small cell height (m)
params.network.ue_power = 23;       % UE transmit power (dBm)
params.network.ue_height = 1.5;     % UE height (m)
params.network.carrier_frequency = 3.5e9;  % Carrier frequency (Hz)
params.network.bandwidth = 100e6;   % Bandwidth (Hz)

% Channel model parameters
params.channel.model = 'WINNER+';   % Channel model ('WINNER+' or '3GPP_V2X')
params.channel.scenario = 'UMi';    % Urban Micro
params.channel.pathloss_exponent_los = 2.2;   % Path loss exponent for LoS
params.channel.pathloss_exponent_nlos = 3.8;  % Path loss exponent for NLoS
params.channel.shadow_std_los = 4;  % Shadow fading std dev for LoS (dB)
params.channel.shadow_std_nlos = 6; % Shadow fading std dev for NLoS (dB)
params.channel.noise_figure = 9;    % UE noise figure (dB)
params.channel.thermal_noise = -174; % Thermal noise density (dBm/Hz)

% Data transmission parameters
params.data.safety_size = 200;      % Safety message size (bytes)
params.data.safety_interval = 0.1;  % Safety message interval (seconds)
params.data.nonsafety_size = 1500;  % Non-safety message size (bytes)
params.data.nonsafety_rate = 2e6;   % Non-safety data rate (bps)
params.data.safety_latency_req = 0.1; % Safety message latency requirement (s)

% RL parameters
params.rl.state_dim = 10;           % State dimension
params.rl.num_actions = 8;          % Number of actions
params.rl.hidden_layers = [64, 64]; % Hidden layer sizes
params.rl.learning_rate = 1e-4;     % Learning rate
params.rl.discount = 0.95;          % Discount factor
params.rl.epsilon = 0.1;            % Exploration rate
params.rl.buffer_size = 10000;      % Replay buffer size
params.rl.batch_size = 32;          % Mini-batch size
params.rl.target_update = 100;      % Target network update frequency
params.rl.do_training = true;       % Enable training
params.rl.update_frequency = 10;    % Update frequency (time steps)

% Reward function weights
params.rl.reward.w_safety = 0.6;    % Weight for safety message reliability
params.rl.reward.w_throughput = 0.3; % Weight for throughput
params.rl.reward.w_overhead = 0.1;  % Weight for handover overhead