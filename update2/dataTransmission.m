%% dataTransmission.m - Enhanced data transmission simulation
function [transmissionStats, qosMetrics] = dataTransmission(vehicles, gNBs, clusters, params)
    % Simulate data transmission with improved reliability and performance metrics
    
    % Initialize statistics
    num_vehicles = length(vehicles);
    transmissionStats(num_vehicles) = struct('throughput', 0, 'max_throughput', 1e6, ...
        'safety_pdr', 0, 'latency', 0, 'safety_sent', 0, 'safety_received', 0, ...
        'nonsafety_sent', 0, 'nonsafety_received', 0);
    
    % Initialize QoS metrics
    qosMetrics = struct('avg_throughput', 0, 'avg_safety_pdr', 0, 'avg_latency', 0);
    
    % For each vehicle, simulate data transmission with improved models
    for v = 1:num_vehicles
        % IMPROVED: Safety message simulation with realistic parameters
        if strcmpi(vehicles(v).safety_interface, 'PC5')
            % Safety over PC5
            if ~isempty(clusters{v})
                % Calculate the number of safety messages to send in this time step
                num_safety_msgs = round(params.delta_t / params.data.safety_interval);
                
                % IMPROVED: More realistic success probability based on SINR and distance
                total_received = 0;
                avg_latency = 0;
                
                % For each potential receiver, calculate reception probability
                for peer_idx = 1:length(clusters{v})
                    peer = clusters{v}(peer_idx);
                    
                    % Check if PC5 metrics exist for this peer
                    if isfield(vehicles(v), 'pc5_metrics') && ...
                       length(vehicles(v).pc5_metrics) >= peer && ...
                       isfield(vehicles(v).pc5_metrics(peer), 'sinr')
                        
                        sinr = vehicles(v).pc5_metrics(peer).sinr;
                        
                        % IMPROVED: More accurate success probability model
                        % Based on 3GPP performance curves for NR-V2X
                        % SINR to BLER (Block Error Rate) mapping - simplified
                        if sinr >= 20
                            bler = 0.001;  % Very low error rate at high SINR
                        elseif sinr >= 10
                            bler = 0.01;   % Low error rate
                        elseif sinr >= 0
                            bler = 0.1;    % Moderate error rate
                        elseif sinr >= -5
                            bler = 0.5;    % High error rate but still usable
                        else
                            bler = 0.9;    % Very high error rate
                        end
                        
                        % Success probability = 1 - BLER
                        success_prob = 1 - bler;
                        
                        % Calculate number of successfully received messages by this peer
                        recv_msgs = binornd(num_safety_msgs, success_prob);
                        total_received = total_received + recv_msgs;
                        
                        % Calculate latency for this link (based on SINR and distance)
                        % Higher SINR = lower latency
                        distance = norm(vehicles(v).position - vehicles(peer).position);
                        propagation_delay = distance / 3e8; % Speed of light
                        processing_delay = 0.01; % Base processing delay (10ms)
                        
                        % Additional delay based on SINR (channel quality)
                        if sinr >= 10
                            channel_delay = 0.005; % 5ms
                        elseif sinr >= 0
                            channel_delay = 0.02; % 20ms
                        else
                            channel_delay = 0.05; % 50ms
                        end
                        
                        link_latency = propagation_delay + processing_delay + channel_delay;
                        avg_latency = avg_latency + link_latency * recv_msgs;
                    end
                end
                
                % Calculate average PDR and latency
                total_possible = num_safety_msgs * length(clusters{v});
                
                if total_possible > 0
                    transmissionStats(v).safety_pdr = total_received / total_possible;
                else
                    transmissionStats(v).safety_pdr = 0;
                end
                
                if total_received > 0
                    transmissionStats(v).latency = avg_latency / total_received;
                else
                    transmissionStats(v).latency = 1.0; % Default high latency
                end
                
                transmissionStats(v).safety_sent = num_safety_msgs;
                transmissionStats(v).safety_received = total_received;
            else
                % No PC5 cluster, zero reception
                transmissionStats(v).safety_pdr = 0;
                transmissionStats(v).latency = 1.0;
                transmissionStats(v).safety_sent = 0;
                transmissionStats(v).safety_received = 0;
            end
        else
            % Safety over Uu (cellular)
            if vehicles(v).serving_mcg > 0
                % IMPROVED: More realistic Uu safety message transmission model
                num_safety_msgs = round(params.delta_t / params.data.safety_interval);
                
                % Calculate success based on Uu SINR and cell load
                uu_sinr = vehicles(v).sinr_uu;
                serving_mcg_idx = vehicles(v).serving_mcg;
                
                % Get cell load (percentage of capacity used)
                cell_load = 0.5; % Default 50% load
                if serving_mcg_idx <= length(gNBs)
                    cell_load = gNBs(serving_mcg_idx).load / 100;
                end
                
                % SINR to success probability mapping - improved model
                if uu_sinr >= 15
                    base_success = 0.99;
                elseif uu_sinr >= 10
                    base_success = 0.95;
                elseif uu_sinr >= 5
                    base_success = 0.9;
                elseif uu_sinr >= 0
                    base_success = 0.8;
                elseif uu_sinr >= -5
                    base_success = 0.6;
                else
                    base_success = 0.3;
                end
                
                % Cell load affects success probability
                % Higher load = more congestion = lower success
                load_factor = max(0.6, 1 - 0.4 * cell_load); % Load impacts success by at most 40%
                success_prob = base_success * load_factor;
                
                % Determine message success
                num_success = binornd(num_safety_msgs, success_prob);
                
                transmissionStats(v).safety_sent = num_safety_msgs;
                transmissionStats(v).safety_received = num_success;
                transmissionStats(v).safety_pdr = num_success / max(1, num_safety_msgs);
                
                % Calculate latency for Uu (cellular network)
                % Typical cellular latencies for V2X are higher than PC5
                base_latency = 0.04; % Base latency 40ms
                load_latency = 0.06 * cell_load; % Up to additional 60ms based on load
                sinr_factor = max(0, min(1, (15 - uu_sinr) / 20)); % SINR impact
                sinr_latency = 0.05 * sinr_factor; % Up to additional 50ms for poor SINR
                
                transmissionStats(v).latency = base_latency + load_latency + sinr_latency;
                
                % Increase serving cell load based on safety message traffic
                % Each safety message is small (typically <1KB)
                gNBs(serving_mcg_idx).load = min(99, gNBs(serving_mcg_idx).load + 0.1);
            else
                % No cellular connection
                transmissionStats(v).safety_pdr = 0;
                transmissionStats(v).latency = 1.0;
                transmissionStats(v).safety_sent = 0;
                transmissionStats(v).safety_received = 0;
            end
        end
        
        % IMPROVED: Non-safety data simulation with realistic parameters
        if strcmpi(vehicles(v).nonsafety_interface, 'Uu')
            % Non-safety over Uu (cellular)
            if vehicles(v).serving_mcg > 0
                % Calculate throughput based on SINR, bandwidth, and cell load
                uu_sinr = vehicles(v).sinr_uu;
                serving_mcg_idx = vehicles(v).serving_mcg;
                
                % Get cell load
                cell_load = 0.5;
                if serving_mcg_idx <= length(gNBs)
                    cell_load = gNBs(serving_mcg_idx).load / 100;
                end
                
                % IMPROVED: More accurate Shannon capacity with overhead
                bandwidth = params.network.bandwidth / 10; % Using 10% of bandwidth
                spectral_efficiency = min(7.8, log2(1 + 10^(uu_sinr/10))); % Capped at 256QAM
                
                % 5G overhead factor and implementation efficiency
                overhead_factor = 0.8; % 80% efficiency after protocol overhead
                impl_efficiency = 0.75; % 75% of theoretical capacity
                
                max_throughput = bandwidth * spectral_efficiency * overhead_factor * impl_efficiency;
                
                % Available capacity depends on cell load
                available_factor = max(0.2, 1 - cell_load);
                actual_throughput = max_throughput * available_factor;
                
                % Limit by maximum application rate
                app_max_throughput = params.data.nonsafety_rate;
                throughput = min(app_max_throughput, actual_throughput);
                
                transmissionStats(v).throughput = throughput;
                transmissionStats(v).max_throughput = app_max_throughput;
                
                % Increase serving cell load based on data usage
                load_increase = throughput / 10e6; % Load increase proportional to throughput
                gNBs(serving_mcg_idx).load = min(99, gNBs(serving_mcg_idx).load + load_increase * 2);
            else
                % No cellular connection
                transmissionStats(v).throughput = 0;
                transmissionStats(v).max_throughput = params.data.nonsafety_rate;
            end
        else
            % Non-safety over PC5
            if ~isempty(clusters{v})
                % PC5 can also be used for non-safety data
                % But typically with lower priority and throughput than safety
                
                if isfield(vehicles(v), 'pc5_metrics') && ~isempty(vehicles(v).pc5_metrics)
                    % Get average SINR across all PC5 links
                    all_sinrs = [];
                    for peer_idx = 1:length(clusters{v})
                        peer = clusters{v}(peer_idx);
                        if peer <= length(vehicles(v).pc5_metrics) && isfield(vehicles(v).pc5_metrics(peer), 'sinr')
                            all_sinrs = [all_sinrs, vehicles(v).pc5_metrics(peer).sinr];
                        end
                    end
                    
                    if ~isempty(all_sinrs)
                        avg_pc5_sinr = mean(all_sinrs);
                        
                        % IMPROVED: More accurate PC5 throughput calculation
                        % PC5 uses different resource allocation than cellular
                        pc5_bandwidth = 10e6; % 10 MHz for PC5 data (typical allocation)
                        spectral_efficiency = min(4.8, log2(1 + 10^(avg_pc5_sinr/10))); % Capped at 64QAM for PC5
                        
                        % PC5 overhead is higher than cellular
                        pc5_overhead = 0.7; % 70% efficiency after protocol overhead
                        pc5_efficiency = 0.6; % 60% of theoretical capacity
                        
                        max_pc5_throughput = pc5_bandwidth * spectral_efficiency * pc5_overhead * pc5_efficiency;
                        
                        % PC5 is shared among all vehicles in range
                        sharing_factor = 1 / max(1, length(clusters{v}));
                        actual_throughput = max_pc5_throughput * sharing_factor;
                        
                        % Limit by application maximum
                        pc5_app_max = params.data.nonsafety_rate * 0.5; % PC5 typically supports lower rates than cellular
                        throughput = min(pc5_app_max, actual_throughput);
                        
                        transmissionStats(v).throughput = throughput;
                        transmissionStats(v).max_throughput = pc5_app_max;
                    else
                        transmissionStats(v).throughput = 0;
                        transmissionStats(v).max_throughput = params.data.nonsafety_rate * 0.5;
                    end
                else
                    transmissionStats(v).throughput = 0;
                    transmissionStats(v).max_throughput = params.data.nonsafety_rate * 0.5;
                end
            else
                % No PC5 cluster
                transmissionStats(v).throughput = 0;
                transmissionStats(v).max_throughput = params.data.nonsafety_rate * 0.5;
            end
        end
    end
    
    % Calculate average QoS metrics
    qosMetrics.avg_throughput = mean([transmissionStats.throughput]);
    qosMetrics.avg_safety_pdr = mean([transmissionStats.safety_pdr]);
    qosMetrics.avg_latency = mean([transmissionStats.latency]);
end