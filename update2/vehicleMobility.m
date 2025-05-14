%% vehicleMobility.m - Enhanced mobility and PC5 clustering
function [vehicles, clusters] = vehicleMobility(vehicles, environment, delta_t)
    % Update vehicle positions based on traffic signal states with improved realism
    
    % Get lane info and signal states
    lanes = environment.lanes;
    signals = environment.signals;
    
    % Initialize clusters
    clusters = cell(length(vehicles), 1);
    
    % Track vehicles that need to slow down due to vehicles ahead
    vehicles_ahead = false(length(vehicles), 1);
    
    % First pass: identify vehicles that need to slow down
    for i = 1:length(vehicles)
        lane_idx = vehicles(i).lane;
        
        % Find vehicles ahead in the same lane
        for j = 1:length(vehicles)
            if i ~= j && vehicles(j).lane == lane_idx
                % Check if vehicle j is ahead of vehicle i in the same lane
                dir_vector = vehicles(i).velocity / (norm(vehicles(i).velocity) + eps); % Avoid division by zero
                rel_pos = vehicles(j).position - vehicles(i).position;
                distance = norm(rel_pos);
                
                % Check if vehicle is ahead and close enough to require slowing down
                if dot(dir_vector, rel_pos) > 0 && distance < 25  % Check if ahead and within 25m
                    relative_speed = dot(dir_vector, vehicles(i).velocity - vehicles(j).velocity);
                    
                    % If approaching the vehicle ahead, mark for slowdown
                    if relative_speed > 0 && distance < 10 + relative_speed * 1.5  % Safety distance
                        vehicles_ahead(i) = true;
                        break;
                    end
                end
            end
        end
    end
    
    % Second pass: update vehicle positions with realistic mobility
    for i = 1:length(vehicles)
        % Current lane and direction
        lane_idx = vehicles(i).lane;
        direction = vehicles(i).direction;
        
        % Check if vehicle should stop at red light
        should_stop = false;
        
        % Determine if vehicle should stop based on signal state and position
        if (strcmp(direction, 'NS') || strcmp(direction, 'SN')) && signals.ns_state == "red"
            % Check if vehicle is approaching intersection
            if strcmp(direction, 'NS') && vehicles(i).position(2) < 50 && vehicles(i).position(2) > -50
                should_stop = true;
            elseif strcmp(direction, 'SN') && vehicles(i).position(2) > -50 && vehicles(i).position(2) < 50
                should_stop = true;
            end
        elseif (strcmp(direction, 'EW') || strcmp(direction, 'WE')) && signals.ew_state == "red"
            % Check if vehicle is approaching intersection
            if strcmp(direction, 'EW') && vehicles(i).position(1) > -50 && vehicles(i).position(1) < 50
                should_stop = true;
            elseif strcmp(direction, 'WE') && vehicles(i).position(1) < 50 && vehicles(i).position(1) > -50
                should_stop = true;
            end
        end
        
        % Vehicle ahead also requires stopping or slowing down
        if vehicles_ahead(i)
            should_stop = true;
        end
        
        % IMPROVED: More realistic vehicle acceleration/deceleration
        if should_stop
            if norm(vehicles(i).velocity) > 0
                % Compute a realistic deceleration - not immediate stop
                % Typical vehicle can decelerate at ~3-5 m/s²
                current_speed = norm(vehicles(i).velocity);
                desired_decel = -3.5; % 3.5 m/s² deceleration
                
                % Calculate how much we can slow down in this time step
                speed_reduction = abs(desired_decel) * delta_t;
                
                if speed_reduction >= current_speed
                    % Come to a complete stop
                    vehicles(i).velocity = [0, 0];
                    vehicles(i).stopped = true;
                    vehicles(i).acceleration = 0;
                else
                    % Gradual slowdown
                    new_speed = current_speed - speed_reduction;
                    dir_unit = vehicles(i).velocity / current_speed;
                    vehicles(i).velocity = dir_unit * new_speed;
                    vehicles(i).acceleration = desired_decel;
                    vehicles(i).stopped = false;
                end
            else
                vehicles(i).stopped = true;
                vehicles(i).acceleration = 0;
            end
        else
            % Accelerate to max speed if not at max
            max_speed = 14;  % m/s (about 50 km/h)
            curr_speed = norm(vehicles(i).velocity);
            
            if curr_speed < max_speed
                % Get lane unit vector
                lane_vector = lanes(lane_idx).end - lanes(lane_idx).start;
                lane_unit = lane_vector / norm(lane_vector);
                
                % IMPROVED: More realistic acceleration curves
                % Typical vehicle can accelerate at ~2-3 m/s²
                accel = 2.5; % 2.5 m/s² acceleration
                
                % Speed increases gradually
                speed_increase = accel * delta_t;
                new_speed = min(max_speed, curr_speed + speed_increase);
                
                vehicles(i).velocity = lane_unit * new_speed;
                vehicles(i).acceleration = accel;
                vehicles(i).stopped = false;
            else
                % Maintain max speed
                vehicles(i).acceleration = 0;
            end
        end
        
        % Update position using trapezoidal integration for smoother movement
        avg_velocity = vehicles(i).velocity - 0.5 * vehicles(i).acceleration * delta_t;
        vehicles(i).position = vehicles(i).position + avg_velocity * delta_t;
        
        % Check if vehicle has reached the end of the lane
        lane_end = lanes(lane_idx).end;
        if norm(vehicles(i).position - lane_end) < 10
            % Loop back to the start of the lane
            vehicles(i).position = lanes(lane_idx).start;
        end
    end
    
    % IMPROVED: More realistic PC5 clustering
    identifyPC5Clusters(vehicles, clusters);
end

function identifyPC5Clusters(vehicles, clusters)
    % Enhanced PC5 cluster identification with realistic V2X parameters
    
    % NR-V2X PC5 communications range (typical values)
    pc5_max_range = 400;    % Maximum theoretical range (m)
    pc5_reliable_range = 250; % Reliable communication range (m)
    
    % Building structures and other obstacles can reduce effective range
    % We'll use a distance-based reliability model
    
    for i = 1:length(vehicles)
        % Find all vehicles that could potentially communicate with this one
        potential_peers = [];
        
        for j = 1:length(vehicles)
            if i ~= j
                % Calculate 2D distance
                dist = norm(vehicles(i).position - vehicles(j).position);
                
                % Check if within maximum PC5 range
                if dist <= pc5_max_range
                    % Calculate reliability based on distance
                    reliability = calculatePC5Reliability(dist, pc5_reliable_range);
                    
                    % Only include if reliability is above threshold
                    if reliability > 0.5
                        potential_peers = [potential_peers, j];
                    end
                end
            end
        end
        
        clusters{i} = potential_peers;
        
        % Store extra information for later use
        vehicles(i).cluster_size = length(potential_peers);
    end
end

function reliability = calculatePC5Reliability(distance, reliable_range)
    % Calculate PC5 communication reliability based on distance
    % Using a model where reliability decays beyond the reliable range
    
    if distance <= reliable_range
        % High reliability within reliable range
        reliability = 0.95;
    else
        % Exponential decay beyond reliable range
        excess_distance = distance - reliable_range;
        decay_factor = 0.01; % Controls how quickly reliability drops
        reliability = 0.95 * exp(-decay_factor * excess_distance);
    end
end