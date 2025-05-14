%% rewardFunction.m - Enhanced reward function for reinforcement learning
function reward = rewardFunction(vehicle, transmission_stats, handover_stats)
    % Calculate reward based on safety reliability, throughput, and handover overhead
    % with improved weighting and priorities
    
    % Extract metrics with error checking
    try
        % Safety PDR (Packet Delivery Ratio)
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
            max_throughput = max(max_throughput, 1e3);
        else
            max_throughput = 1e6;
        end
        
        % Handover overhead
        if isfield(handover_stats, 'type')
            handover_type = handover_stats.type;
            
            % Different penalties for different types of handovers
            if handover_type == 1 % MCG Handover - high cost
                handover_penalty = 1.0;
            elseif handover_type == 2 % SCG Add - medium cost
                handover_penalty = 0.7;
            elseif handover_type == 3 % SCG Remove - medium cost
                handover_penalty = 0.7;
            elseif handover_type >= 4 && handover_type <= 7 % Interface switches - low cost
                handover_penalty = 0.3;
            else
                handover_penalty = 0;
            end
        else
            handover_penalty = 0;
        end
        
        % Ping-pong penalty - severe penalty
        if isfield(handover_stats, 'isPingPong') && handover_stats.isPingPong
            pingpong_penalty = 5.0;
        else
            pingpong_penalty = 0;
        end
        
        % Interface switching overhead - low penalty
        if isfield(handover_stats, 'interfaceSwitch') && handover_stats.interfaceSwitch
            switch_penalty = 0.3;
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
            avg_latency = 1; % Default high latency
        end
        
        % IMPROVED: Better latency reward calculation
        latency_target = 0.1;  % 100ms target latency for safety messages
        
        if avg_latency <= latency_target
            latency_reward = 1;  % Full reward if under target
        else
            % Exponential decay of reward for higher latencies
            latency_ratio = latency_target / avg_latency;
            latency_reward = max(0, exp(-2 * (1 - latency_ratio))); % Exponential decay
        end
        
        % IMPROVED: Normalize throughput with logarithmic scaling
        % This prevents small absolute changes at high throughput from dominating the reward
        if throughput > 0 && max_throughput > 0
            log_throughput = log10(max(1, throughput));
            log_max_throughput = log10(max(1, max_throughput));
            norm_throughput = min(1, log_throughput / log_max_throughput);
        else
            norm_throughput = 0;
        end
        
        % IMPROVED: Safety PDR reward with higher threshold and steeper curve
        if safety_pdr >= 0.9
            safety_pdr_reward = 1.0; % Full reward for >=90% PDR
        elseif safety_pdr >= 0.8
            safety_pdr_reward = 0.7 + 3.0 * (safety_pdr - 0.8); % Steeper reward 0.7-1.0
        elseif safety_pdr >= 0.7
            safety_pdr_reward = 0.4 + 3.0 * (safety_pdr - 0.7); % Steeper reward 0.4-0.7
        else
            safety_pdr_reward = safety_pdr * 0.6; % Lower baseline
        end
        
        % Adjust base weights
        base_w_safety = 0.7;      % Increase from 0.6
        base_w_throughput = 0.2;  % Decrease from 0.3
        base_w_overhead = 0.1;    % Keep the same
        
        % Adjust weights based on safety performance
        if safety_pdr < 0.5 || avg_latency > 0.3
            % Increase safety weight when safety performance is poor
            safety_boost = 0.2;
            w_safety = base_w_safety + safety_boost;
            w_throughput = base_w_throughput - safety_boost/2;
            w_overhead = base_w_overhead - safety_boost/2;
        else
            w_safety = base_w_safety;
            w_throughput = base_w_throughput;
            w_overhead = base_w_overhead;
        end
        
        % Calculate total reward
        safety_component = w_safety * (0.7 * safety_pdr_reward + 0.3 * latency_reward);
        throughput_component = w_throughput * norm_throughput;
        overhead_component = w_overhead * (handover_penalty + pingpong_penalty + switch_penalty);
        
        % Combined reward with improved formula
        reward = safety_component + throughput_component - overhead_component;
        
        % IMPROVED: Ensure reward is properly bounded
        reward = max(-1, min(1, reward));
        
    catch err
        % If anything fails, return a default reward and log the error
        warning('ERROR in rewardFunction: %s', err.identifier);
        disp(err.message);
        reward = 0;
    end
end