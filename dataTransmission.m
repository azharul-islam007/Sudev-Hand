%% dataTransmission.m - Simulate data transmission and calculate KPIs
function [transmissionStats, qosMetrics] = dataTransmission(vehicles, gNBs, clusters, params)
    % Simulate data transmission and calculate performance metrics
    
    % Initialize statistics
    num_vehicles = length(vehicles);
    transmissionStats(num_vehicles) = struct('throughput', 0, 'max_throughput', 1e6, ...
        'safety_pdr', 0, 'latency', 0, 'safety_sent', 0, 'safety_received', 0, ...
        'nonsafety_sent', 0, 'nonsafety_received', 0);
    
    % Initialize QoS metrics
    qosMetrics = struct('avg_throughput', 0, 'avg_safety_pdr', 0, 'avg_latency', 0);
    
    % For each vehicle, simulate data transmission
    for v = 1:num_vehicles
        % Safety message simulation
        if strcmp(vehicles(v).safety_interface, 'PC5')
            % Safety over PC5
            if ~isempty(clusters{v})
                % Calculate average PC5 SINR
                if isfield(vehicles(v), 'pc5_metrics') && ~isempty(vehicles(v).pc5_metrics)
                    pc5_sinrs = [vehicles(v).pc5_metrics.sinr];
                    avg_pc5_sinr = mean(pc5_sinrs);
                    
                    % Success probability based on SINR
                    success_prob = min(1, max(0, (avg_pc5_sinr + 5) / 30));
                    
                    % Determine message success
                    num_safety_msgs = round(params.delta_t / params.data.safety_interval);
                    num_success = binornd(num_safety_msgs, success_prob);
                    
                    transmissionStats(v).safety_sent = num_safety_msgs;
                    transmissionStats(v).safety_received = num_success;
                    transmissionStats(v).safety_pdr = num_success / max(1, num_safety_msgs);
                    
                    % Latency estimation (based on SINR)
                    transmissionStats(v).latency = params.data.safety_latency_req * ...
                        (1 + max(0, (5 - avg_pc5_sinr) / 10));
                else
                    % No PC5 links available
                    transmissionStats(v).safety_pdr = 0;
                    transmissionStats(v).latency = 1;  % High latency when no links
                end
            else
                % No PC5 cluster, zero reception
                transmissionStats(v).safety_pdr = 0;
                transmissionStats(v).latency = 1;
            end
        else
            % Safety over Uu
            if vehicles(v).serving_mcg > 0
                % Calculate success based on Uu SINR
                uu_sinr = vehicles(v).sinr_uu;
                success_prob = min(1, max(0, (uu_sinr + 5) / 25));
                
                % Determine message success
                num_safety_msgs = round(params.delta_t / params.data.safety_interval);
                num_success = binornd(num_safety_msgs, success_prob);
                
                transmissionStats(v).safety_sent = num_safety_msgs;
                transmissionStats(v).safety_received = num_success;
                transmissionStats(v).safety_pdr = num_success / max(1, num_safety_msgs);
                
                % Latency estimation (higher than PC5 generally)
                transmissionStats(v).latency = 1.5 * params.data.safety_latency_req * ...
                    (1 + max(0, (5 - uu_sinr) / 10));
                
                % Increase serving cell load
                serving_mcg = vehicles(v).serving_mcg;
                gNBs(serving_mcg).load = min(100, gNBs(serving_mcg).load + 1);
            else
                % No cellular connection
                transmissionStats(v).safety_pdr = 0;
                transmissionStats(v).latency = 1;
            end
        end
        
        % Non-safety data simulation
        if strcmp(vehicles(v).nonsafety_interface, 'Uu')
            % Non-safety over Uu
            if vehicles(v).serving_mcg > 0
                % Calculate throughput based on SINR
                uu_sinr = vehicles(v).sinr_uu;
                
                % Shannon capacity formula with efficiency factor
                bw = params.network.bandwidth / 10;  % Fraction of bandwidth allocated
                efficiency = 0.6;  % Practical efficiency factor
                
                max_throughput = params.data.nonsafety_rate;
                shannon_capacity = bw * log2(1 + 10^(uu_sinr/10)) * efficiency;
                
                % Limit by maximum application rate
                throughput = min(max_throughput, shannon_capacity);
                
                transmissionStats(v).throughput = throughput;
                transmissionStats(v).max_throughput = max_throughput;
                
                % Account for cell load (simplified)
                serving_mcg = vehicles(v).serving_mcg;
                load_factor = 1 - (gNBs(serving_mcg).load / 200);  % Diminishing returns with load
                transmissionStats(v).throughput = transmissionStats(v).throughput * load_factor;
                
                % Increase serving cell load
                gNBs(serving_mcg).load = min(100, gNBs(serving_mcg).load + 5);
            else
                % No cellular connection
                transmissionStats(v).throughput = 0;
            end
        else
            % Non-safety over PC5
            if ~isempty(clusters{v})
                % Calculate throughput based on PC5 SINR
                if isfield(vehicles(v), 'pc5_metrics') && ~isempty(vehicles(v).pc5_metrics)
                    pc5_sinrs = [vehicles(v).pc5_metrics.sinr];
                    avg_pc5_sinr = mean(pc5_sinrs);
                    
                    % Shannon capacity formula with efficiency factor
                    bw = 20e6;  % Sidelink bandwidth (smaller than Uu)
                    efficiency = 0.5;  % Lower efficiency for PC5
                    
                    max_throughput = params.data.nonsafety_rate * 0.5;  % Lower rate for PC5
                    shannon_capacity = bw * log2(1 + 10^(avg_pc5_sinr/10)) * efficiency;
                    
                    % Limit by maximum application rate
                    throughput = min(max_throughput, shannon_capacity);
                    
                    transmissionStats(v).throughput = throughput;
                    transmissionStats(v).max_throughput = max_throughput;
                else
                    % No PC5 metrics available
                    transmissionStats(v).throughput = 0;
                end
            else
                % No PC5 cluster
                transmissionStats(v).throughput = 0;
            end
        end
    end
    
    % Calculate average QoS metrics
    qosMetrics.avg_throughput = mean([transmissionStats.throughput]);
    qosMetrics.avg_safety_pdr = mean([transmissionStats.safety_pdr]);
    qosMetrics.avg_latency = mean([transmissionStats.latency]);
end