%% vehicleMobility.m - Vehicle mobility and clustering
function [vehicles, clusters] = vehicleMobility(vehicles, environment, delta_t)
    % Update vehicle positions based on traffic signal states
    
    % Get lane info and signal states
    lanes = environment.lanes;
    signals = environment.signals;
    
    % Initialize clusters
    clusters = cell(length(vehicles), 1);
    
    % Update each vehicle's position
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
        
        % Check for vehicle ahead (simplified collision avoidance)
        min_spacing = 5;  % Minimum spacing between vehicles
        for j = 1:length(vehicles)
            if i ~= j && vehicles(j).lane == lane_idx
                % Check if vehicle j is ahead of vehicle i in the same lane
                dist = norm(vehicles(j).position - vehicles(i).position);
                
                % Simple check if vehicle j is ahead in the direction of travel
                dir_vector = vehicles(i).velocity / norm(vehicles(i).velocity);
                rel_pos = vehicles(j).position - vehicles(i).position;
                ahead = dot(dir_vector, rel_pos) > 0;
                
                if ahead && dist < min_spacing
                    should_stop = true;
                    break;
                end
            end
        end
        
        % Update velocity and position
        if should_stop
            % Decelerate to stop
            if norm(vehicles(i).velocity) > 0
                decel = -norm(vehicles(i).velocity) / delta_t;  % Stop within delta_t
                vehicles(i).velocity = vehicles(i).velocity * (1 + decel * delta_t);
                
                % Ensure velocity doesn't go negative
                if norm(vehicles(i).velocity) < 0.1
                    vehicles(i).velocity = [0, 0];
                    vehicles(i).stopped = true;
                end
            else
                vehicles(i).stopped = true;
            end
        else
            % Accelerate to max speed if not at max
            max_speed = 14;  % m/s (about 50 km/h)
            curr_speed = norm(vehicles(i).velocity);
            
            if curr_speed < max_speed
                % Get lane unit vector
                lane_vector = lanes(lane_idx).end - lanes(lane_idx).start;
                lane_unit = lane_vector / norm(lane_vector);
                
                % Accelerate
                accel = 2;  % m/s^2
                new_speed = min(max_speed, curr_speed + accel * delta_t);
                vehicles(i).velocity = lane_unit * new_speed;
                vehicles(i).stopped = false;
            end
        end
        
        % Update position
        vehicles(i).position = vehicles(i).position + vehicles(i).velocity * delta_t;
        
        % Check if vehicle has reached the end of the lane
        lane_end = lanes(lane_idx).end;
        if norm(vehicles(i).position - lane_end) < 10
            % Either remove vehicle or loop back (for continuous simulation)
            % For now, we'll loop back to the start of the lane
            vehicles(i).position = lanes(lane_idx).start;
        end
    end
    
    % Identify vehicle clusters for PC5 communications
    pc5_range = 300;  % Maximum range for PC5 (m)
    
    for i = 1:length(vehicles)
        % Find all vehicles within PC5 range
        for j = 1:length(vehicles)
            if i ~= j
                dist = norm(vehicles(i).position - vehicles(j).position);
                
                if dist <= pc5_range
                    clusters{i} = [clusters{i}, j];
                end
            end
        end
    end
end