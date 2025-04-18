%% visualizeResults.m - Visualize simulation results
function visualizeResults(metrics, logs, environment, params)
    % Create plots of key performance metrics
    
    % Extract time steps
    time_steps = unique(metrics.rewards(:,1));
    num_vehicles = max(metrics.rewards(:,2));
    
    % Average reward over time
    figure('Name', 'Average Reward');
    hold on;
    
    avg_rewards = zeros(length(time_steps), 1);
    for t = 1:length(time_steps)
        t_idx = metrics.rewards(:,1) == time_steps(t);
        avg_rewards(t) = mean(metrics.rewards(t_idx, 3));
    end
    
    plot(time_steps, avg_rewards, 'b-', 'LineWidth', 2);
    title('Average Reward vs. Time');
    xlabel('Time (s)');
    ylabel('Average Reward');
    grid on;
    
    % Safety PDR over time
    figure('Name', 'Safety Message PDR');
    hold on;
    
    avg_pdr = zeros(length(time_steps), 1);
    for t = 1:length(time_steps)
        t_idx = metrics.safety_pdr(:,1) == time_steps(t);
        avg_pdr(t) = mean(metrics.safety_pdr(t_idx, 3));
    end
    
    plot(time_steps, avg_pdr, 'g-', 'LineWidth', 2);
    title('Average Safety Message PDR vs. Time');
    xlabel('Time (s)');
    ylabel('Packet Delivery Ratio');
    ylim([0, 1]);
    grid on;
    
    % Throughput over time
    figure('Name', 'Average Throughput');
    hold on;
    
    avg_throughput = zeros(length(time_steps), 1);
    for t = 1:length(time_steps)
        t_idx = metrics.throughput(:,1) == time_steps(t);
        avg_throughput(t) = mean(metrics.throughput(t_idx, 3)) / 1e6;  % Convert to Mbps
    end
    
    plot(time_steps, avg_throughput, 'r-', 'LineWidth', 2);
    title('Average Throughput vs. Time');
    xlabel('Time (s)');
    ylabel('Throughput (Mbps)');
    grid on;
    
    % Handover events
    figure('Name', 'Handover Events');
    hold on;
    
    % Count handovers by type over time windows
    window_size = 10;  % 10-second windows
    num_windows = ceil(params.T_sim / window_size);
    
    handover_counts = zeros(num_windows, 4);  % MCG, SCG_Add, SCG_Remove, Interface
    
    for w = 1:num_windows
        window_start = (w-1) * window_size;
        window_end = w * window_size;
        
        % Filter handovers in this window
        window_idx = metrics.handovers(:,1) >= window_start & metrics.handovers(:,1) < window_end;
        window_handovers = metrics.handovers(window_idx, :);
        
        % Count by type
        handover_counts(w, 1) = sum(window_handovers(:,3) == 1);  % MCG
        handover_counts(w, 2) = sum(window_handovers(:,3) == 2);  % SCG Add
        handover_counts(w, 3) = sum(window_handovers(:,3) == 3);  % SCG Remove
        handover_counts(w, 4) = sum(window_handovers(:,3) >= 4 & window_handovers(:,3) <= 7);  % Interface
    end
    
    window_times = (1:num_windows) * window_size - window_size/2;
    
    bar(window_times, handover_counts, 'stacked');
    title('Handover Events Over Time');
    xlabel('Time (s)');
    ylabel('Number of Handovers');
    legend('MCG Handover', 'SCG Add', 'SCG Remove', 'Interface Switch');
    grid on;
    
    % Ping-pong count
    total_pingpong = size(metrics.pingpong, 1);
    total_handovers = size(metrics.handovers, 1);
    
    fprintf('Total ping-pong handovers: %d (%.1f%% of all handovers)\n', ...
        total_pingpong, 100 * total_pingpong / max(1, total_handovers));
    
    % Create animation of vehicle movement (if requested)
    if isfield(params, 'create_animation') && params.create_animation
        createAnimation(logs, environment, params);
    end
end

function createAnimation(logs, environment, params)
    % Create animation of vehicle movements
    fprintf('Creating animation...\n');
    
    % Create video writer object
    if ~exist(params.output_dir, 'dir')
        mkdir(params.output_dir);
    end
    
    video_file = fullfile(params.output_dir, 'simulation_animation.mp4');
    vidObj = VideoWriter(video_file, 'MPEG-4');
    vidObj.FrameRate = 10;
    vidObj.Quality = 90;
    open(vidObj);
    
    % Get time steps
    time_steps = unique(logs.vehicle_positions(:,1));
    
    % Create figure
    fig = figure('Position', [100, 100, 800, 600]);
    
    % For each time step
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        % Clear figure
        clf;
        hold on;
        
        % Plot environment (roads, buildings)
        % ... (similar to plotEnvironment function)
        
        % Plot vehicles at this time step
        t_positions = logs.vehicle_positions(logs.vehicle_positions(:,1) == t, :);
        
        for v = 1:size(t_positions, 1)
            vehicle_id = t_positions(v, 2);
            x = t_positions(v, 3);
            y = t_positions(v, 4);
            
            % Color based on interface
            link_state = logs.link_states(logs.link_states(:,1) == t & ...
                                         logs.link_states(:,2) == vehicle_id, :);
            
            if ~isempty(link_state)
                safety_interface = link_state(3);
                nonsafety_interface = link_state(4);
                
                if safety_interface == 1 && nonsafety_interface == 1
                    % Both on Uu
                    color = 'r';
                elseif safety_interface == 2 && nonsafety_interface == 2
                    % Both on PC5
                    color = 'g';
                else
                    % Mixed
                    color = 'b';
                end
            else
                color = 'k';
            end
            
            % Plot vehicle
            plot(x, y, 'o', 'MarkerSize', 8, 'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
        end
        
        % Set plot limits
        xlim([-environment.size(1)/2, environment.size(1)/2]);
        ylim([-environment.size(2)/2, environment.size(2)/2]);
        
        % Add title with time
        title(sprintf('Simulation Time: %.1f s', t));
        
        % Add legend
        legend('Uu Only', 'PC5 Only', 'Mixed', 'Location', 'northeast');
        
        % Capture frame
        frame = getframe(fig);
        writeVideo(vidObj, frame);
        
        % Display progress
        if mod(t_idx, 10) == 0
            fprintf('Animation progress: %.1f%%\n', 100 * t_idx / length(time_steps));
        end
    end
    
    % Close video writer
    close(vidObj);
    fprintf('Animation saved to %s\n', video_file);
end