%% rewardFunction.m - Calculate the reward for reinforcement learning (ultra robust version)
function reward = rewardFunction(vehicle, transmission_stats, handover_stats)
    % Calculate reward based on safety reliability, throughput, and handover overhead
    
    % Extract metrics with error checking
    try
        % Safety PDR
        if isfield(transmission_stats, 'safety_pdr')
            safety_pdr = transmission_stats.safety_pdr;
            if ~isscalar(safety_pdr)
                safety_pdr = mean(safety_pdr(:));
            end
        else
            safety_pdr = 0;
        end
        
        % Throughput
        if isfield(transmission_stats, 'throughput')
            throughput = transmission_stats.throughput;
            if ~isscalar(throughput)
                throughput = mean(throughput(:));
            end
        else
            throughput = 0;
        end
        
        % Max throughput
        if isfield(transmission_stats, 'max_throughput')
            max_throughput = transmission_stats.max_throughput;
            if ~isscalar(max_throughput)
                max_throughput = mean(max_throughput(:));
            end
            % Ensure non-zero
            max_throughput = max(max_throughput, 1);
        else
            max_throughput = 1;
        end
        
        % Handover overhead
        if isfield(handover_stats, 'type')
            handover_penalty = (handover_stats.type > 0);
        else
            handover_penalty = 0;
        end
        
        % Ping-pong penalty
        if isfield(handover_stats, 'isPingPong') && handover_stats.isPingPong
            pingpong_penalty = 3;
        else
            pingpong_penalty = 0;
        end
        
        % Interface switching overhead
        if isfield(handover_stats, 'interfaceSwitch') && handover_stats.interfaceSwitch
            switch_penalty = 0.5;
        else
            switch_penalty = 0;
        end
        
        % Safety message latency
        if isfield(transmission_stats, 'latency')
            avg_latency = transmission_stats.latency;
            if ~isscalar(avg_latency)
                avg_latency = mean(avg_latency(:));
            end
        else
            avg_latency = 1;
        end
        
        latency_target = 0.1;  % 100ms target latency
        
        if avg_latency <= latency_target
            latency_reward = 1;
        else
            latency_ratio = latency_target / avg_latency;
            latency_reward = max(0, latency_ratio);
        end
        
        % Normalize throughput to [0,1]
        norm_throughput = min(1, double(throughput) / double(max_throughput));
        
        % Calculate total reward
        w_safety = 0.6;
        w_throughput = 0.3;
        w_overhead = 0.1;
        
        reward = w_safety * (0.7 * safety_pdr + 0.3 * latency_reward) + ...
                 w_throughput * norm_throughput - ...
                 w_overhead * (handover_penalty + pingpong_penalty + switch_penalty);
        
        % Ensure reward is bounded
        reward = max(-1, min(1, reward));
        
    catch e
        % If anything fails, return a default reward and log the error
        fprintf('ERROR in rewardFunction: %s\n', e.message);
        reward = 0;
    end
end