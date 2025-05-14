%% determineLoS.m - Determine Line-of-Sight conditions
function isLoS = determineLoS(vehicle, gNBs, buildings)
    % Determine if there is Line-of-Sight between a vehicle and gNBs
    
    % Initialize LoS vector
    isLoS = false(length(gNBs), 1);
    
    % For each gNB, check if any building blocks the path
    for i = 1:length(gNBs)
        % Get start and end points of path
        start_point = vehicle.position;
        end_point = gNBs(i).position;
        
        % By default, assume LoS
        has_los = true;
        
        % Check intersection with each building
        for b = 1:size(buildings, 1)
            building = buildings(b, :);
            
            % Building corners (simplified as rectangles)
            building_min = [building(1) - building(3)/2, building(2) - building(4)/2];
            building_max = [building(1) + building(3)/2, building(2) + building(4)/2];
            
            % Check if line intersects building using simplified check
            if lineIntersectsRectangle(start_point, end_point, building_min, building_max)
                has_los = false;
                break;
            end
        end
        
        isLoS(i) = has_los;
    end
end