%% calculateDistances.m - Calculate distances from vehicle to gNBs
function [dist_macro, dist_small] = calculateDistances(vehicle, gNBs)
    % Calculate distances from vehicle to macro and small cells
    
    % Count number of macro and small cells
    num_macro = sum(strcmp({gNBs.type}, "macro"));
    num_small = sum(strcmp({gNBs.type}, "small"));
    
    % Initialize distance arrays
    dist_macro = zeros(num_macro, 1);
    dist_small = zeros(num_small, 1);
    
    % Calculate distances
    macro_idx = 1;
    small_idx = 1;
    
    for i = 1:length(gNBs)
        % Calculate 2D distance (horizontal plane)
        dist_2d = norm(vehicle.position - gNBs(i).position);
        
        if strcmp(gNBs(i).type, "macro")
            dist_macro(macro_idx) = dist_2d;
            macro_idx = macro_idx + 1;
        else % small cell
            dist_small(small_idx) = dist_2d;
            small_idx = small_idx + 1;
        end
    end
end