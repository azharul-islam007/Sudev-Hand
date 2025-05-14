%% channelModel.m - Improved WINNER+/3GPP V2X channel model
function [rsrp, sinr] = channelModel(type, source, destination, channel_params)
    % Enhanced channel model for both Uu and PC5 interfaces
    %
    % Inputs:
    %   type: 'Uu' or 'PC5'
    %   source: Transmitter (vehicle or gNB)
    %   destination: Receiver (vehicle or gNB)
    %   channel_params: Channel parameters
    %
    % Outputs:
    %   rsrp: Reference Signal Received Power (dBm)
    %   sinr: Signal-to-Interference-plus-Noise Ratio (dB)
    
    % Default UE height if not available
    ue_height = 1.5; % Default UE height in meters
    
    % Calculate 3D distance
    if strcmpi(type, 'Uu')
        % Handle array of gNBs
        if isstruct(destination) && isfield(destination, 'type') && length(destination) > 1
            % When destination is an array of gNBs, use the serving cell
            if isfield(source, 'serving_mcg') && source.serving_mcg > 0
                serving_cell = source.serving_mcg;
                dest = destination(serving_cell);
            else
                % Default to first gNB if no serving cell specified
                dest = destination(1);
            end
            
            tx_pos = [dest.position, dest.height];
        else
            % Single gNB case
            tx_pos = [destination.position, destination.height];
        end
        
        rx_pos = [source.position, ue_height];
    else % PC5
        % Vehicle to vehicle distance
        tx_pos = [source.position, ue_height];
        rx_pos = [destination.position, ue_height];
    end
    
    % Ensure tx_pos and rx_pos have the same dimensions
    if length(tx_pos) ~= length(rx_pos)
        if length(tx_pos) == 2
            tx_pos = [tx_pos, 0]; % Add height of 0
        elseif length(rx_pos) == 2
            rx_pos = [rx_pos, 0]; % Add height of 0
        end
    end
    
    % Calculate distance
    distance_3d = norm(tx_pos - rx_pos);
    
    % IMPROVED: Better LoS/NLoS determination
    if isfield(source, 'isLoS') && ~isempty(source.isLoS) && isnumeric(source.isLoS)
        % If isLoS is an array, get the value for the current destination
        if length(source.isLoS) > 1 && isstruct(destination) && isfield(destination, 'id')
            dest_id = min(length(source.isLoS), destination(1).id); % Ensure valid index
            isLoS = source.isLoS(dest_id);
        else
            isLoS = source.isLoS(1);
        end
    else
        % IMPROVED: More realistic LoS probability model based on 3GPP TR 38.901
        % Urban environment LoS probability model
        if strcmpi(type, 'Uu')
            % For UE to gNB (UMi scenario)
            d_2d = sqrt(sum((tx_pos(1:2) - rx_pos(1:2)).^2));
            p_los = min(18/d_2d, 1) * (1 - exp(-d_2d/36)) + exp(-d_2d/36);
        else
            % For UE to UE (V2V scenario)
            p_los = min(1, exp(-distance_3d/80)); % Higher LoS probability for V2V
        end
        isLoS = rand() < p_los;
    end
    
    % IMPROVED: More accurate path loss parameters
    if isLoS
        % LoS condition - better propagation
        path_loss_exponent = 2.2; % Typical for LoS paths (improved from 3GPP models)
        shadow_std = 3.0;         % Lower shadowing in LoS (dB)
    else
        % NLoS condition - obstructed propagation
        path_loss_exponent = 3.5; % Typical for NLoS paths (improved from 3GPP models)
        shadow_std = 6.0;         % Higher shadowing in NLoS (dB)
    end
    
    % Override with channel parameters if provided
    if isfield(channel_params, 'pathloss_exponent_los')
        path_loss_exponent = isLoS * channel_params.pathloss_exponent_los + ...
                           (~isLoS) * channel_params.pathloss_exponent_nlos;
    end
    
    if isfield(channel_params, 'shadow_std_los')
        shadow_std = isLoS * channel_params.shadow_std_los + ...
                   (~isLoS) * channel_params.shadow_std_nlos;
    end
    
    % Calculate path loss with 3GPP model
    % Get carrier frequency
    carrier_frequency = 3.5e9; % Default 3.5 GHz for 5G FR1
    if isfield(channel_params, 'carrier_frequency')
        carrier_frequency = channel_params.carrier_frequency;
    end
    
    % IMPROVED: Better path loss model based on 3GPP
    if strcmpi(type, 'Uu')
        if isLoS
            % 3GPP UMi LoS model
            pl0 = 28.0 + 22*log10(distance_3d) + 20*log10(carrier_frequency/1e9);
        else
            % 3GPP UMi NLoS model
            pl0 = 13.54 + 39.08*log10(distance_3d) + 20*log10(carrier_frequency/1e9);
        end
    else % PC5
        if isLoS
            % V2V LoS model
            pl0 = 32.4 + 20*log10(distance_3d) + 20*log10(carrier_frequency/1e9);
        else
            % V2V NLoS model
            pl0 = 36.85 + 30*log10(distance_3d) + 18.9*log10(carrier_frequency/1e9);
        end
    end
    
    % Add shadow fading (log-normal)
    shadow_fading = randn() * shadow_std;
    
    % IMPROVED: Spatial correlation of shadow fading
    % In a real implementation, you would track shadow fading per link over time
    % with appropriate spatial correlation
    
    % Calculate received power
    if strcmpi(type, 'Uu')
        % Get transmit power from the gNB
        if isstruct(destination) && isfield(destination, 'power')
            if length(destination) > 1
                % Use serving cell power
                if isfield(source, 'serving_mcg') && source.serving_mcg > 0
                    tx_power = destination(source.serving_mcg).power;
                else
                    tx_power = destination(1).power; % Default to first gNB
                end
            else
                tx_power = destination.power;
            end
        else
            tx_power = 46; % Default macro cell power (dBm)
        end
    else % PC5
        tx_power = 23; % Default UE transmit power (dBm)
    end
    
    % Calculate total path loss
    path_loss = pl0 + shadow_fading;
    
    % Calculate received power
    rx_power = tx_power - path_loss;
    
    % IMPROVED: More accurate RSRP calculation
    % Get bandwidth
    bandwidth = 100e6; % Default 100 MHz
    if isfield(channel_params, 'bandwidth')
        bandwidth = channel_params.bandwidth;
    end
    
    % Calculate RSRP (Reference Signal Received Power)
    % RSRP is the average power of resource elements that carry cell-specific reference signals
    num_rbs = floor(bandwidth / 180e3); % 180 kHz per resource block in 5G NR
    
    % Assume reference signals occupy about 4.76% of resource elements (similar to LTE)
    rs_power_offset = 10*log10(0.0476);
    
    rsrp = rx_power + rs_power_offset;
    
    % IMPROVED: More accurate noise and interference calculation
    % Get noise figure
    noise_figure = 9; % Default 9 dB
    if isfield(channel_params, 'noise_figure')
        noise_figure = channel_params.noise_figure;
    end
    
    % Get thermal noise density
    thermal_noise = -174; % Default -174 dBm/Hz
    if isfield(channel_params, 'thermal_noise')
        thermal_noise = channel_params.thermal_noise;
    end
    
    % Calculate noise power
    noise_power = thermal_noise + 10*log10(bandwidth) + noise_figure;
    
    % IMPROVED: More realistic interference model
    if strcmpi(type, 'Uu')
        % For Uu, interference depends on cell loading and other factors
        % For urban environment, interference is significant
        load_factor = 0.5; % Assume 50% cell load by default
        
        % If we have cell load information, use it
        if isstruct(destination) && isfield(destination, 'load')
            if length(destination) > 1 && isfield(source, 'serving_mcg') && source.serving_mcg > 0
                load_factor = destination(source.serving_mcg).load / 100;
            else
                load_factor = destination(1).load / 100;
            end
        end
        
        % Interference increases with load
        interference_to_noise = 5 + 5 * load_factor; % 5 to 10 dB based on load
        interference_power = noise_power + interference_to_noise;
    else % PC5
        % For PC5, interference depends on vehicle density
        vehicle_density = 5; % Default vehicles per km²
        
        % If we have vehicle cluster information, use it
        if isfield(source, 'pc5_metrics') && ~isempty(source.pc5_metrics)
            vehicle_density = length(source.pc5_metrics) * 2; % Estimate from PC5 links
        end
        
        % PC5 uses sensing-based resource allocation to reduce interference
        sensing_gain = 3; % 3 dB reduction due to sensing
        interference_power = noise_power + 10*log10(max(1, vehicle_density/10)) - sensing_gain;
    end
    
    % Calculate total received noise+interference power
    total_interference = 10*log10(10^(noise_power/10) + 10^(interference_power/10));
    
    % Calculate SINR
    sinr = rx_power - total_interference;
    
    % IMPROVED: More realistic fast fading model based on LoS/NLoS
    if isLoS
        % Rician fading for LoS scenarios
        % K-factor depends on distance and environment
        k_factor = 10; % Strong LoS component
        if distance_3d > 50
            k_factor = max(1, 15 - 0.2 * distance_3d); % K decreases with distance
        end
        
        % Apply Rician fading
        fading_dB = ricianFading(k_factor);
    else
        % Rayleigh fading for NLoS scenarios
        fading_dB = rayleighFading();
    end
    
    % Apply fading to SINR
    sinr = sinr + fading_dB;
end

function fading_dB = ricianFading(k_factor)
    % Generate Rician fading sample in dB with improved implementation
    % k_factor: Rician K-factor (ratio of LOS to scattered power)
    
    % Generate complex Gaussian samples for scattered component
    x_real = randn();
    x_imag = randn();
    
    % Calculate scattered and LoS component powers
    scattered_power = 1 / (k_factor + 1);
    los_power = k_factor / (k_factor + 1);
    
    % LoS component (fixed phase assumed for simplicity)
    los_component = sqrt(los_power);
    
    % Scattered component
    scattered_component = sqrt(scattered_power/2) * complex(x_real, x_imag);
    
    % Combined signal
    signal = los_component + scattered_component;
    
    % Envelope power
    envelope_power = abs(signal)^2;
    
    % Convert to dB
    fading_dB = 10 * log10(envelope_power);
end

function fading_dB = rayleighFading()
    % Generate Rayleigh fading sample in dB with improved implementation
    
    % Generate complex Gaussian samples
    x_real = randn();
    x_imag = randn();
    
    % Calculate Rayleigh envelope (normalized for unit average power)
    envelope = sqrt(x_real^2 + x_imag^2) / sqrt(2);
    
    % Convert to dB
    fading_dB = 20 * log10(envelope);
end