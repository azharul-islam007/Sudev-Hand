%% initEnvironment.m - Initialize simulation environment
function [environment, vehicles, gNBs] = initEnvironment(params)
    % Set random seed for reproducibility
    rng(params.seed);
    
    % Create intersection layout
    environment.size = params.env.intersection_size;
    environment.lanes = createLanes(params.env.num_lanes, params.env.lane_width);
    
    % Create buildings
    environment.buildings = generateBuildings(environment.size, params.env.building_density);
    
    % Initialize traffic signals
    environment.signals = initTrafficSignals(params.env.traffic_light_cycle);
    
    % Initialize gNBs (macro and small cells)
    gNBs = initializeGNBs(params.network);
    
    % Initialize vehicles
    vehicles = initializeVehicles(params.vehicle, environment);
    
    fprintf('Environment initialized with:\n');
    fprintf('  - %d vehicles\n', length(vehicles));
    fprintf('  - %d macro cells and %d small cells\n', ...
        sum([gNBs.type] == "macro"), sum([gNBs.type] == "small"));
    fprintf('  - %d buildings\n', size(environment.buildings, 1));
    
    % Create figure for visualization
    environment.fig = figure('Name', 'V2X Simulation', 'Position', [100, 100, 800, 600]);
    
    % Plot initial environment
    plotEnvironment(environment, vehicles, gNBs);
end

% Helper functions
function lanes = createLanes(num_lanes, lane_width)
    % Create lane structure for intersection
    % North-South and East-West lanes with proper offsets
    ns_offsets = ((1:num_lanes) - 0.5) * lane_width;
    ew_offsets = ((1:num_lanes) - 0.5) * lane_width;
    
    lanes = struct('direction', {}, 'offset', {}, 'start', {}, 'end', {});
    idx = 1;
    
    % North to South lanes
    for i = 1:num_lanes
        lanes(idx).direction = 'NS';
        lanes(idx).offset = ns_offsets(i);
        lanes(idx).start = [ns_offsets(i), -250];
        lanes(idx).end = [ns_offsets(i), 250];
        idx = idx + 1;
    end
    
    % South to North lanes
    for i = 1:num_lanes
        lanes(idx).direction = 'SN';
        lanes(idx).offset = -ns_offsets(i);
        lanes(idx).start = [-ns_offsets(i), 250];
        lanes(idx).end = [-ns_offsets(i), -250];
        idx = idx + 1;
    end
    
    % East to West lanes
    for i = 1:num_lanes
        lanes(idx).direction = 'EW';
        lanes(idx).offset = ew_offsets(i);
        lanes(idx).start = [250, ew_offsets(i)];
        lanes(idx).end = [-250, ew_offsets(i)];
        idx = idx + 1;
    end
    
    % West to East lanes
    for i = 1:num_lanes
        lanes(idx).direction = 'WE';
        lanes(idx).offset = -ew_offsets(i);
        lanes(idx).start = [-250, -ew_offsets(i)];
        lanes(idx).end = [250, -ew_offsets(i)];
        idx = idx + 1;
    end
end

function buildings = generateBuildings(area_size, density)
    % Generate random buildings in the urban environment
    max_buildings = floor(area_size(1) * area_size(2) * density / 1000);
    buildings = zeros(max_buildings, 4); % [x, y, width, height]
    
    for i = 1:max_buildings
        % Avoid placing buildings directly on the roads
        if rand() < 0.5
            % Buildings on the corners of the intersection
            quadrant = randi(4);
            switch quadrant
                case 1 % NE
                    x = 50 + rand() * (area_size(1)/2 - 50);
                    y = 50 + rand() * (area_size(2)/2 - 50);
                case 2 % NW
                    x = -50 - rand() * (area_size(1)/2 - 50);
                    y = 50 + rand() * (area_size(2)/2 - 50);
                case 3 % SW
                    x = -50 - rand() * (area_size(1)/2 - 50);
                    y = -50 - rand() * (area_size(2)/2 - 50);
                case 4 % SE
                    x = 50 + rand() * (area_size(1)/2 - 50);
                    y = -50 - rand() * (area_size(2)/2 - 50);
            end
        else
            % Buildings further from the intersection
            x = (rand() - 0.5) * area_size(1);
            y = (rand() - 0.5) * area_size(2);
            
            % Ensure buildings are not on the roads
            if abs(x) < 20 && abs(y) < 20
                x = x + sign(x) * 20;
                y = y + sign(y) * 20;
            end
        end
        
        width = 10 + rand() * 30;   % Building width between 10-40m
        height = 10 + rand() * 30;  % Building height between 10-40m
        
        buildings(i, :) = [x, y, width, height];
    end
end

function signals = initTrafficSignals(cycle_durations)
    % Initialize traffic signals with phases
    signals = struct();
    signals.ns_state = 'green';  % Initial state for North-South
    signals.ew_state = 'red';    % Initial state for East-West
    signals.ns_timer = cycle_durations(1);  % Green time for North-South
    signals.ew_timer = 0;        % Red time for East-West
    signals.cycle_durations = cycle_durations;  % [green, red] durations
end

function gNBs = initializeGNBs(network_params)
    % Initialize base stations (macro and small cells)
    total_cells = network_params.macro_count + network_params.small_count;
    gNBs(total_cells) = struct('id', 0, 'type', '', 'position', [], 'power', 0, 'height', 0, 'load', 0);
    
    % Place macro cells
    for i = 1:network_params.macro_count
        gNBs(i).id = i;
        gNBs(i).type = "macro";
        
        % Place macro at a higher position overlooking the intersection
        if i == 1
            gNBs(i).position = [0, 0];  % Center of intersection
        else
            angle = 2 * pi * (i-1) / network_params.macro_count;
            distance = 300;  % 300m from center
            gNBs(i).position = [distance * cos(angle), distance * sin(angle)];
        end
        
        gNBs(i).power = network_params.macro_power;
        gNBs(i).height = network_params.macro_height;
        gNBs(i).load = 0;  % Initial load
    end
    
    % Place small cells
    for i = 1:network_params.small_count
        idx = network_params.macro_count + i;
        gNBs(idx).id = idx;
        gNBs(idx).type = "small";
        
        % Place small cells around the intersection
        angle = 2 * pi * (i-1) / network_params.small_count;
        distance = 150;  % 150m from center
        gNBs(idx).position = [distance * cos(angle), distance * sin(angle)];
        
        gNBs(idx).power = network_params.small_power;
        gNBs(idx).height = network_params.small_height;
        gNBs(idx).load = 0;  % Initial load
    end
end

function vehicles = initializeVehicles(vehicle_params, environment)
    % Initialize vehicles
    vehicles(vehicle_params.count) = struct('id', 0, 'position', [], 'velocity', [], ...
        'acceleration', 0, 'lane', 0, 'direction', '', 'stopped', false, ...
        'serving_mcg', 0, 'serving_scg', 0, 'rsrp_uu', [], 'sinr_uu', [], ...
        'pc5_metrics', [], 'safety_interface', 'PC5', 'nonsafety_interface', 'Uu', ...
        'safety_queue', 0, 'nonsafety_queue', 0, 'handover_history', []);
    
    % Place vehicles on the lanes
    lanes = environment.lanes;
    lane_directions = {lanes.direction};
    
    for i = 1:vehicle_params.count
        vehicles(i).id = i;
        
        % Randomly select a lane
        lane_idx = randi(length(lanes));
        vehicles(i).lane = lane_idx;
        vehicles(i).direction = lanes(lane_idx).direction;
        
        % Position along the lane with some spacing
        lane_length = norm(lanes(lane_idx).end - lanes(lane_idx).start);
        position_along_lane = rand() * lane_length;
        
        % Calculate absolute position
        lane_vector = lanes(lane_idx).end - lanes(lane_idx).start;
        lane_unit = lane_vector / norm(lane_vector);
        vehicles(i).position = lanes(lane_idx).start + position_along_lane * lane_unit;
        
        % Initial velocity (random between 0 and max_speed)
        vehicles(i).velocity = lane_unit * (0.5 + 0.5 * rand()) * vehicle_params.max_speed;
        
        % Initial acceleration
        vehicles(i).acceleration = 0;
        
        % Not stopped initially
        vehicles(i).stopped = false;
        
        % Initial serving cell (closest macro)
        vehicles(i).serving_mcg = 1;  % Simplified: assign to first macro
        vehicles(i).serving_scg = 0;  % No SCG initially
        
        % Initial interface selection
        vehicles(i).safety_interface = 'PC5';  % Safety messages on PC5 by default
        vehicles(i).nonsafety_interface = 'Uu';  % Non-safety on Uu by default
        
        % Initialize empty pc5_metrics array
        vehicles(i).pc5_metrics = [];
        
        % Initial message queues
        vehicles(i).safety_queue = 0;
        vehicles(i).nonsafety_queue = randi([0, 10]);  % Random initial non-safety queue
        
        % Empty handover history
        vehicles(i).handover_history = [];
    end
end

function plotEnvironment(environment, vehicles, gNBs)
    % Plot the environment, vehicles, and base stations
    figure(environment.fig);
    clf;
    hold on;
    
    % Plot lanes
    for i = 1:length(environment.lanes)
        lane = environment.lanes(i);
        plot([lane.start(1), lane.end(1)], [lane.start(2), lane.end(2)], 'k-', 'LineWidth', 1);
    end
    
    % Plot buildings
    for i = 1:size(environment.buildings, 1)
        building = environment.buildings(i, :);
        rectangle('Position', [building(1)-building(3)/2, building(2)-building(4)/2, building(3), building(4)], ...
            'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'k');
    end
    
    % Plot gNBs
    for i = 1:length(gNBs)
        if gNBs(i).type == "macro"
            plot(gNBs(i).position(1), gNBs(i).position(2), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        else
            plot(gNBs(i).position(1), gNBs(i).position(2), 'm^', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        end
    end
    
    % Plot vehicles
    for i = 1:length(vehicles)
        plot(vehicles(i).position(1), vehicles(i).position(2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    end
    
    % Plot traffic signals
    if environment.signals.ns_state == "green"
        plot(0, 0, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'NS: Green');
    else
        plot(0, 0, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', 'NS: Red');
    end
    
    % Set plot limits and labels
    xlim([-environment.size(1)/2, environment.size(1)/2]);
    ylim([-environment.size(2)/2, environment.size(2)/2]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('V2X Urban Environment Simulation');
    grid on;
    legend('Location', 'northeast');
    drawnow;
    hold off;
end