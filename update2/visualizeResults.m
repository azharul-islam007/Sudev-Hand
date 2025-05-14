%% visualizeResults.m - Enhanced visualization of simulation results
function visualizeResults(metrics, logs, environment, params)
    % Create comprehensive plots of key performance metrics with enhanced visualization
    
    % Create a figure to hold all plots
    figure('Name', 'V2X Simulation Results', 'Position', [100, 100, 1200, 800]);
    
    % Extract time steps
    time_steps = unique(metrics.rewards(:,1));
    num_vehicles = max(metrics.rewards(:,2));
    
    % Define colors for consistent use across plots
    colors = struct('safety', [0.2, 0.7, 0.3], ...  % Green
                   'throughput', [0, 0.4, 0.8], ... % Blue
                   'latency', [0.8, 0.3, 0.1], ...  % Orange
                   'handover', [0.6, 0.2, 0.6], ... % Purple
                   'reward', [0.1, 0.1, 0.1]);      % Black
    
    %% 1. Average Reward over Time
    subplot(3, 2, 1);
    
    % Calculate average reward per time step
    avg_rewards = zeros(length(time_steps), 1);
    for t = 1:length(time_steps)
        t_idx = metrics.rewards(:,1) == time_steps(t);
        avg_rewards(t) = mean(metrics.rewards(t_idx, 3));
    end
    
    % Plot with confidence interval
    hold on;
    % Calculate standard deviation
    std_rewards = zeros(length(time_steps), 1);
    for t = 1:length(time_steps)
        t_idx = metrics.rewards(:,1) == time_steps(t);
        if sum(t_idx) > 1
            std_rewards(t) = std(metrics.rewards(t_idx, 3));
        end
    end
    
    % Plot confidence interval
    fill([time_steps; flipud(time_steps)], ...
         [avg_rewards-std_rewards; flipud(avg_rewards+std_rewards)], ...
         colors.reward, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    % Plot mean line
    plot(time_steps, avg_rewards, 'Color', colors.reward, 'LineWidth', 2);
    
    title('Average Reward vs. Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Average Reward');
    grid on;
    
    %% 2. Safety Message PDR over Time
    subplot(3, 2, 2);
    
    % Calculate average PDR per time step
    avg_pdr = zeros(length(time_steps), 1);
    std_pdr = zeros(length(time_steps), 1);
    
    for t = 1:length(time_steps)
        t_idx = metrics.safety_pdr(:,1) == time_steps(t);
        if any(t_idx)
            avg_pdr(t) = mean(metrics.safety_pdr(t_idx, 3));
            if sum(t_idx) > 1
                std_pdr(t) = std(metrics.safety_pdr(t_idx, 3));
            end
        end
    end
    
    hold on;
    % Plot confidence interval
    fill([time_steps; flipud(time_steps)], ...
         [max(0, avg_pdr-std_pdr); min(1, flipud(avg_pdr+std_pdr))], ...
         colors.safety, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    % Plot mean line
    plot(time_steps, avg_pdr, 'Color', colors.safety, 'LineWidth', 2);
    
    % Add threshold line for minimum acceptable PDR (e.g., 90%)
    yline(0.9, '--', '90% Target', 'Color', [0.5, 0, 0], 'LineWidth', 1.5);
    
    title('Safety Message PDR vs. Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Packet Delivery Ratio');
    ylim([0, 1]);
    grid on;
    
    %% 3. Latency over Time
    subplot(3, 2, 3);
    
    % Calculate average latency per time step (in ms for better readability)
    avg_latency = zeros(length(time_steps), 1);
    std_latency = zeros(length(time_steps), 1);
    
    for t = 1:length(time_steps)
        t_idx = metrics.latency(:,1) == time_steps(t);
        if any(t_idx)
            avg_latency(t) = mean(metrics.latency(t_idx, 3)) * 1000; % Convert to ms
            if sum(t_idx) > 1
                std_latency(t) = std(metrics.latency(t_idx, 3)) * 1000;
            end
        end
    end
    
    hold on;
    % Plot confidence interval
    fill([time_steps; flipud(time_steps)], ...
         [max(0, avg_latency-std_latency); flipud(avg_latency+std_latency)], ...
         colors.latency, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    % Plot mean line
    plot(time_steps, avg_latency, 'Color', colors.latency, 'LineWidth', 2);
    
    % Add threshold line for maximum acceptable latency (e.g., 100ms)
    yline(100, '--', '100 ms Target', 'Color', [0.5, 0, 0], 'LineWidth', 1.5);
    
    title('Average Safety Message Latency vs. Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Latency (ms)');
    grid on;
    
    %% 4. Throughput over Time
    subplot(3, 2, 4);
    
    % Calculate average throughput per time step (in Mbps for better readability)
    avg_throughput = zeros(length(time_steps), 1);
    std_throughput = zeros(length(time_steps), 1);
    
    for t = 1:length(time_steps)
        t_idx = metrics.throughput(:,1) == time_steps(t);
        if any(t_idx)
            avg_throughput(t) = mean(metrics.throughput(t_idx, 3)) / 1e6; % Convert to Mbps
            if sum(t_idx) > 1
                std_throughput(t) = std(metrics.throughput(t_idx, 3)) / 1e6;
            end
        end
    end
    
    hold on;
    % Plot confidence interval
    fill([time_steps; flipud(time_steps)], ...
         [max(0, avg_throughput-std_throughput); flipud(avg_throughput+std_throughput)], ...
         colors.throughput, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    % Plot mean line
    plot(time_steps, avg_throughput, 'Color', colors.throughput, 'LineWidth', 2);
    
    title('Average Throughput vs. Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Throughput (Mbps)');
    grid on;
    
    %% 5. Handover Events Histogram
    subplot(3, 2, 5);
    
    % Count handovers by type over time windows
    window_size = params.T_sim / 10; % 10 windows
    num_windows = 10;
    
    handover_counts = zeros(num_windows, 4);  % MCG, SCG_Add, SCG_Remove, Interface
    window_times = zeros(num_windows, 1);
    
    for w = 1:num_windows
        window_start = (w-1) * window_size;
        window_end = w * window_size;
        window_times(w) = (window_start + window_end) / 2;
        
        % Filter handovers in this window
        window_idx = metrics.handovers(:,1) >= window_start & metrics.handovers(:,1) < window_end;
        window_handovers = metrics.handovers(window_idx, :);
        
        % Count by type
        handover_counts(w, 1) = sum(window_handovers(:,3) == 1);  % MCG
        handover_counts(w, 2) = sum(window_handovers(:,3) == 2);  % SCG Add
        handover_counts(w, 3) = sum(window_handovers(:,3) == 3);  % SCG Remove
        handover_counts(w, 4) = sum(window_handovers(:,3) >= 4 & window_handovers(:,3) <= 7);  % Interface
    end
    
    bar(window_times, handover_counts, 'stacked');
    title('Handover Events Over Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Number of Handovers');
    legend('MCG Handover', 'SCG Add', 'SCG Remove', 'Interface Switch', 'Location', 'northwest');
    grid on;
    
    %% 6. Interface Selection Pie Charts
    subplot(3, 2, 6);
    
    % Get the interface selection at the end of simulation
    last_time = max(logs.link_states(:,1));
    last_time_idx = logs.link_states(:,1) == last_time;
    
    if any(last_time_idx)
        final_states = logs.link_states(last_time_idx, :);
        
        % Count safety interface usage
        safety_uu = sum(final_states(:,3) == 1);
        safety_pc5 = sum(final_states(:,3) == 2);
        
        % Count non-safety interface usage
        nonsafety_uu = sum(final_states(:,4) == 1);
        nonsafety_pc5 = sum(final_states(:,4) == 2);
        
        % Create pie chart
        pie_data = [safety_uu, safety_pc5, nonsafety_uu, nonsafety_pc5];
        labels = {'Safety on Uu', 'Safety on PC5', 'Non-safety on Uu', 'Non-safety on PC5'};
        
        % Filter out zero values
        non_zero = pie_data > 0;
        pie(pie_data(non_zero), labels(non_zero));
        title('Final Interface Selection', 'FontWeight', 'bold');
    else
        text(0.5, 0.5, 'No interface data available', 'HorizontalAlignment', 'center');
    end
    
    % Adjust figure layout
    sgtitle('V2X Dual Connectivity Simulation Results', 'FontSize', 16, 'FontWeight', 'bold');
    set(gcf, 'Color', 'w');
    
    % Save figure to file
    if isfield(params, 'output_dir') && ~isempty(params.output_dir)
        if ~exist(params.output_dir, 'dir')
            mkdir(params.output_dir);
        end
        saveas(gcf, fullfile(params.output_dir, 'simulation_results.png'));
        saveas(gcf, fullfile(params.output_dir, 'simulation_results.fig'));
    end
    
    %% Create additional detailed visualizations
    
    % Vehicle positions and cluster heatmap
    createVehicleHeatmap(logs, environment, params);
    
    % Interface selection over time
    visualizeInterfaceSelection(logs, params);
    
    % Compare PC5 vs Uu performance
    compareInterfacePerformance(metrics, logs, params);
end

function createVehicleHeatmap(logs, environment, params)
    % Create a heatmap showing vehicle density throughout the simulation
    
    % Create a new figure
    figure('Name', 'Vehicle Position Heatmap', 'Position', [150, 150, 800, 700]);
    
    % Define grid for heatmap
    grid_size = 20; % Number of cells in each dimension
    x_min = -environment.size(1)/2;
    x_max = environment.size(1)/2;
    y_min = -environment.size(2)/2;
    y_max = environment.size(2)/2;
    
    x_edges = linspace(x_min, x_max, grid_size + 1);
    y_edges = linspace(y_min, y_max, grid_size + 1);
    
    % Calculate position counts
    position_counts = zeros(grid_size, grid_size);
    
    % Process vehicle positions
    x_positions = logs.vehicle_positions(:, 3);
    y_positions = logs.vehicle_positions(:, 4);
    
    % Count occurrences in each grid cell
    for i = 1:length(x_positions)
        x = x_positions(i);
        y = y_positions(i);
        
        % Find the corresponding grid cell
        x_idx = max(1, min(grid_size, ceil((x - x_min) / (x_max - x_min) * grid_size)));
        y_idx = max(1, min(grid_size, ceil((y - y_min) / (y_max - y_min) * grid_size)));
        
        position_counts(y_idx, x_idx) = position_counts(y_idx, x_idx) + 1;
    end
    
    % Create heatmap
    imagesc(x_edges(1:end-1), y_edges(1:end-1), position_counts);
    colormap('hot');
    colorbar;
    axis equal;
    
    % Add intersection visualization
    hold on;
    % Draw roads
    plot([x_min, x_max], [0, 0], 'w-', 'LineWidth', 2); % Horizontal road
    plot([0, 0], [y_min, y_max], 'w-', 'LineWidth', 2); % Vertical road
    
    % Draw buildings if available
    if isfield(environment, 'buildings')
        for i = 1:size(environment.buildings, 1)
            building = environment.buildings(i, :);
            rectangle('Position', [building(1)-building(3)/2, building(2)-building(4)/2, building(3), building(4)], ...
                      'EdgeColor', 'w', 'LineWidth', 1);
        end
    end
    
    title('Vehicle Position Density Heatmap', 'FontWeight', 'bold');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    
    % Save figure
    if isfield(params, 'output_dir') && ~isempty(params.output_dir)
        saveas(gcf, fullfile(params.output_dir, 'vehicle_heatmap.png'));
    end
end

function visualizeInterfaceSelection(logs, params)
    % Visualize how interface selection changes over time
    
    % Create a new figure
    figure('Name', 'Interface Selection Over Time', 'Position', [200, 200, 1000, 600]);
    
    % Get unique time steps
    time_steps = unique(logs.link_states(:,1));
    
    % Initialize counters
    safety_uu_count = zeros(length(time_steps), 1);
    safety_pc5_count = zeros(length(time_steps), 1);
    nonsafety_uu_count = zeros(length(time_steps), 1);
    nonsafety_pc5_count = zeros(length(time_steps), 1);
    
    % Count interface usage at each time step
    for t = 1:length(time_steps)
        t_idx = logs.link_states(:,1) == time_steps(t);
        
        if any(t_idx)
            t_states = logs.link_states(t_idx, :);
            
            % Count safety interface usage
            safety_uu_count(t) = sum(t_states(:,3) == 1);
            safety_pc5_count(t) = sum(t_states(:,3) == 2);
            
            % Count non-safety interface usage
            nonsafety_uu_count(t) = sum(t_states(:,4) == 1);
            nonsafety_pc5_count(t) = sum(t_states(:,4) == 2);
        end
    end
    
    % Create stacked area plots
    subplot(2, 1, 1);
    area(time_steps, [safety_uu_count, safety_pc5_count]);
    title('Safety Message Interface Selection Over Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Number of Vehicles');
    legend('Uu (Cellular)', 'PC5 (Direct)', 'Location', 'best');
    grid on;
    
    subplot(2, 1, 2);
    area(time_steps, [nonsafety_uu_count, nonsafety_pc5_count]);
    title('Non-Safety Message Interface Selection Over Time', 'FontWeight', 'bold');
    xlabel('Time (s)');
    ylabel('Number of Vehicles');
    legend('Uu (Cellular)', 'PC5 (Direct)', 'Location', 'best');
    grid on;
    
    % Save figure
    if isfield(params, 'output_dir') && ~isempty(params.output_dir)
        saveas(gcf, fullfile(params.output_dir, 'interface_selection.png'));
    end
end

function compareInterfacePerformance(metrics, logs, params)
    % Compare performance metrics between PC5 and Uu interfaces
    
    % Create a new figure
    figure('Name', 'Interface Performance Comparison', 'Position', [250, 250, 1000, 800]);
    
    % Get safety interface information
    time_steps = unique(logs.link_states(:,1));
    
    % Initialize arrays to store metrics by interface
    pc5_pdr = [];
    uu_pdr = [];
    pc5_latency = [];
    uu_latency = [];
    
    % For each time step, get performance metrics by interface
    for t = 1:length(time_steps)
        t_time = time_steps(t);
        
        % Get interface selections at this time
        t_links_idx = logs.link_states(:,1) == t_time;
        t_links = logs.link_states(t_links_idx, :);
        
        % Get safety PDR at this time
        t_pdr_idx = metrics.safety_pdr(:,1) == t_time;
        t_pdr = metrics.safety_pdr(t_pdr_idx, :);
        
        % Get latency at this time
        t_latency_idx = metrics.latency(:,1) == t_time;
        t_latency = metrics.latency(t_latency_idx, :);
        
        % Match vehicles
        for i = 1:size(t_links, 1)
            vehicle_id = t_links(i, 2);
            safety_interface = t_links(i, 3);
            
            % Find matching PDR and latency entries
            pdr_idx = find(t_pdr(:,2) == vehicle_id);
            latency_idx = find(t_latency(:,2) == vehicle_id);
            
            if ~isempty(pdr_idx) && ~isempty(latency_idx)
                if safety_interface == 1  % Uu
                    uu_pdr = [uu_pdr; t_pdr(pdr_idx(1), 3)];
                    uu_latency = [uu_latency; t_latency(latency_idx(1), 3)];
                else  % PC5
                    pc5_pdr = [pc5_pdr; t_pdr(pdr_idx(1), 3)];
                    pc5_latency = [pc5_latency; t_latency(latency_idx(1), 3)];
                end
            end
        end
    end
    
    % Create PDR comparison
    subplot(2, 2, 1);
    boxplot([pc5_pdr; uu_pdr], [ones(size(pc5_pdr)); 2*ones(size(uu_pdr))], ...
            'Labels', {'PC5', 'Uu'}, 'Colors', [0.2, 0.7, 0.3; 0, 0.4, 0.8]);
    title('Safety Message PDR by Interface', 'FontWeight', 'bold');
    ylabel('Packet Delivery Ratio');
    ylim([0, 1]);
    grid on;
    
    % Create latency comparison (in ms)
    subplot(2, 2, 2);
    boxplot([pc5_latency*1000; uu_latency*1000], [ones(size(pc5_latency)); 2*ones(size(uu_latency))], ...
            'Labels', {'PC5', 'Uu'}, 'Colors', [0.2, 0.7, 0.3; 0, 0.4, 0.8]);
    title('Safety Message Latency by Interface', 'FontWeight', 'bold');
    ylabel('Latency (ms)');
    grid on;
    
    % PDR distribution
    subplot(2, 2, 3);
    hold on;
    
    % Create histograms
    histogram(pc5_pdr, 10, 'Normalization', 'probability', 'FaceColor', [0.2, 0.7, 0.3], 'FaceAlpha', 0.7);
    histogram(uu_pdr, 10, 'Normalization', 'probability', 'FaceColor', [0, 0.4, 0.8], 'FaceAlpha', 0.7);
    
    title('PDR Distribution by Interface', 'FontWeight', 'bold');
    xlabel('Packet Delivery Ratio');
    ylabel('Probability');
    legend('PC5', 'Uu', 'Location', 'northwest');
    grid on;
    
    % Latency distribution
    subplot(2, 2, 4);
    hold on;
    
    % Create histograms
    histogram(pc5_latency*1000, 10, 'Normalization', 'probability', 'FaceColor', [0.2, 0.7, 0.3], 'FaceAlpha', 0.7);
    histogram(uu_latency*1000, 10, 'Normalization', 'probability', 'FaceColor', [0, 0.4, 0.8], 'FaceAlpha', 0.7);
    
    title('Latency Distribution by Interface', 'FontWeight', 'bold');
    xlabel('Latency (ms)');
    ylabel('Probability');
    legend('PC5', 'Uu', 'Location', 'northeast');
    grid on;
    
    % Add overall title
    sgtitle('PC5 vs. Uu Interface Performance Comparison', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Save figure
    if isfield(params, 'output_dir') && ~isempty(params.output_dir)
        saveas(gcf, fullfile(params.output_dir, 'interface_comparison.png'));
    end
end