%% calculateAggregateMetrics.m - Calculate aggregate metrics from simulation results
function aggregateMetrics = calculateAggregateMetrics(metrics)
    % Calculate aggregate metrics from simulation data
    
    % Initialize result structure
    aggregateMetrics = struct();
    
    % Average throughput
    if ~isempty(metrics.throughput)
        aggregateMetrics.avg_throughput = mean(metrics.throughput(:,3));
    else
        aggregateMetrics.avg_throughput = 0;
    end
    
    % Average safety PDR
    if ~isempty(metrics.safety_pdr)
        aggregateMetrics.avg_safety_pdr = mean(metrics.safety_pdr(:,3));
    else
        aggregateMetrics.avg_safety_pdr = 0;
    end
    
    % Average latency
    if ~isempty(metrics.latency)
        aggregateMetrics.avg_latency = mean(metrics.latency(:,3));
    else
        aggregateMetrics.avg_latency = 0;
    end
    
    % Handover statistics
    if ~isempty(metrics.handovers)
        aggregateMetrics.total_handovers = size(metrics.handovers, 1);
        
        % Count different types of handovers
        aggregateMetrics.mcg_handovers = sum(metrics.handovers(:,3) == 1);
        aggregateMetrics.scg_add = sum(metrics.handovers(:,3) == 2);
        aggregateMetrics.scg_remove = sum(metrics.handovers(:,3) == 3);
        aggregateMetrics.interface_switches = sum(metrics.handovers(:,3) >= 4);
    else
        aggregateMetrics.total_handovers = 0;
        aggregateMetrics.mcg_handovers = 0;
        aggregateMetrics.scg_add = 0;
        aggregateMetrics.scg_remove = 0;
        aggregateMetrics.interface_switches = 0;
    end
    
    % Ping-pong statistics
    if ~isempty(metrics.pingpong)
        aggregateMetrics.ping_pong_count = size(metrics.pingpong, 1);
        aggregateMetrics.ping_pong_ratio = aggregateMetrics.ping_pong_count / ...
                                          max(1, aggregateMetrics.total_handovers);
    else
        aggregateMetrics.ping_pong_count = 0;
        aggregateMetrics.ping_pong_ratio = 0;
    end
    
    % Average reward
    if ~isempty(metrics.rewards)
        aggregateMetrics.avg_reward = mean(metrics.rewards(:,3));
    else
        aggregateMetrics.avg_reward = 0;
    end
end