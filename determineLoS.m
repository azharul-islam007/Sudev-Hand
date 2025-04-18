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

function intersects = lineIntersectsRectangle(p1, p2, rect_min, rect_max)
    % Simplified check if line segment from p1 to p2 intersects rectangle
    
    % Line parametric equation: p1 + t*(p2-p1), t in [0,1]
    dir = p2 - p1;
    
    % X planes intersection
    tx1 = (rect_min(1) - p1(1)) / dir(1);
    tx2 = (rect_max(1) - p1(1)) / dir(1);
    
    if isnan(tx1) || isnan(tx2) % Handle division by zero
        tx_min = -inf;
        tx_max = inf;
    else
        tx_min = min(tx1, tx2);
        tx_max = max(tx1, tx2);
    end
    
    % Y planes intersection
    ty1 = (rect_min(2) - p1(2)) / dir(2);
    ty2 = (rect_max(2) - p1(2)) / dir(2);
    
    if isnan(ty1) || isnan(ty2) % Handle division by zero
        ty_min = -inf;
        ty_max = inf;
    else
        ty_min = min(ty1, ty2);
        ty_max = max(ty1, ty2);
    end
    
    % Intersection exists if intervals overlap
    t_min = max(tx_min, ty_min);
    t_max = min(tx_max, ty_max);
    
    intersects = (t_min <= t_max) && (t_max >= 0) && (t_min <= 1);
end