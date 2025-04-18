%% channelModel.m - Simplified WINNER+/3GPP V2X channel model
function [rsrp, sinr] = channelModel(type, source, destination, channel_params)
    % Simplified channel model for both Uu and PC5 interfaces
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
    if type == "Uu"
        % For Uu, we need to handle multiple gNBs
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
        % Adjust to match dimensions, assuming 2D positions as minimum
        if length(tx_pos) == 2
            tx_pos = [tx_pos, 0]; % Add height of 0
        elseif length(rx_pos) == 2
            rx_pos = [rx_pos, 0]; % Add height of 0
        end
    end
    
    distance_3d = norm(tx_pos - rx_pos);
    
    % Determine LoS or NLoS
    if isfield(source, 'isLoS') && ~isempty(source.isLoS)
        isLoS = source.isLoS;
        
        % If isLoS is an array, get the value for the current destination
        if length(isLoS) > 1 && isstruct(destination) && isfield(destination, 'id')
            dest_id = destination(1).id; % Use first gNB by default
            if isfield(source, 'serving_mcg') && source.serving_mcg > 0
                dest_id = source.serving_mcg;
            end
            isLoS = isLoS(dest_id);
        end
    else
        % Simple probability model for LoS
        p_los = exp(-(distance_3d/100));
        isLoS = rand() < p_los;
    end
    
    % Select appropriate path loss exponent
    if isLoS
        path_loss_exponent = channel_params.pathloss_exponent_los;
        shadow_std = channel_params.shadow_std_los;
    else
        path_loss_exponent = channel_params.pathloss_exponent_nlos;
        shadow_std = channel_params.shadow_std_nlos;
    end
    
    % Calculate path loss
    % Path loss model: PL = PL0 + 10*n*log10(d/d0) + Xσ
    carrier_frequency = 3.5e9; % Default if not specified
    if isfield(channel_params, 'carrier_frequency')
        carrier_frequency = channel_params.carrier_frequency;
    end
    
    carrier_wavelength = 3e8 / carrier_frequency;
    pl0 = 20 * log10(4 * pi / carrier_wavelength); % Free space path loss at 1m
    
    path_loss = pl0 + 10 * path_loss_exponent * log10(max(distance_3d, 1));
    
    % Add shadow fading (log-normal)
    shadow_fading = shadow_std * randn();
    
    % Calculate received power
    if type == "Uu"
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
    
    rx_power = tx_power - path_loss - shadow_fading;
    
    % RSRP calculation (reference signal received power)
    % Simplified: RSRP is rx_power normalized by the number of resource blocks
    % In a real system, this would be more complex
    bandwidth = 100e6; % Default bandwidth
    if isfield(channel_params, 'bandwidth')
        bandwidth = channel_params.bandwidth;
    end
    
    num_rbs = bandwidth / 180e3; % 180 kHz per resource block
    rsrp = rx_power - 10*log10(num_rbs);
    
    % SINR calculation (simplified)
    noise_figure = 9; % Default noise figure (dB)
    if isfield(channel_params, 'noise_figure')
        noise_figure = channel_params.noise_figure;
    end
    
    thermal_noise = -174; % Default thermal noise density (dBm/Hz)
    if isfield(channel_params, 'thermal_noise')
        thermal_noise = channel_params.thermal_noise;
    end
    
    noise_power = thermal_noise + 10*log10(bandwidth) + noise_figure;
    
    % Simplified interference model
    if type == "Uu"
        % For Uu, interference comes from other cells
        % Simplified: assume fixed interference-to-noise ratio
        inr = 3; % 3 dB interference over noise (simplified assumption)
        interference = noise_power + inr;
    else % PC5
        % For PC5, interference from other vehicles in the vicinity
        % Simplified: proportional to vehicle density
        vehicle_density = 5; % Assumed constant for simplicity
        interference = noise_power + 10*log10(vehicle_density);
    end
    
    % Calculate SINR
    sinr = rx_power - 10*log10(10^(interference/10) + 10^(noise_power/10));
    
    % Apply small-scale fading (simplified Rayleigh or Rician)
    if isLoS
        % Rician fading (K-factor based on distance)
        k_factor = 10 - 0.03 * distance_3d; % Simplified K-factor model
        k_factor = max(k_factor, 1); % Ensure K-factor is at least 1
        
        % Generate Rician fading
        fading_dB = ricianFading(k_factor);
    else
        % Rayleigh fading
        fading_dB = rayleighFading();
    end
    
    % Apply fading to SINR
    sinr = sinr + fading_dB;
end

function fading_dB = ricianFading(k_factor)
    % Generate Rician fading sample in dB
    % k_factor: Rician K-factor (ratio of LOS to scattered power)
    
    % Generate complex Gaussian samples
    x = randn() + 1i * randn();
    
    % Rician envelope with K-factor
    los_amplitude = sqrt(k_factor / (k_factor + 1));
    nlos_amplitude = sqrt(1 / (k_factor + 1));
    
    envelope = abs(los_amplitude + nlos_amplitude * x);
    fading_dB = 20 * log10(envelope);
end

function fading_dB = rayleighFading()
    % Generate Rayleigh fading sample in dB
    
    % Generate complex Gaussian samples
    x = randn() + 1i * randn();
    
    % Rayleigh envelope
    envelope = abs(x) / sqrt(2);
    fading_dB = 20 * log10(envelope);
end