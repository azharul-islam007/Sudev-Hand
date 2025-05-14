%% plotEnvironment.m - Visualize the simulation environment
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
        if strcmp(gNBs(i).type, "macro")
            plot(gNBs(i).position(1), gNBs(i).position(2), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        else
            plot(gNBs(i).position(1), gNBs(i).position(2), 'm^', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        end
    end
    
    % Plot vehicles with different colors based on interface
    for i = 1:length(vehicles)
        % Color based on interface
        if strcmp(vehicles(i).safety_interface, 'Uu')
            if strcmp(vehicles(i).nonsafety_interface, 'Uu')
                % Both on Uu - red
                color = 'r';
            else
                % Safety on Uu, non-safety on PC5 - purple
                color = 'm';
            end
        else
            if strcmp(vehicles(i).nonsafety_interface, 'Uu')
                % Safety on PC5, non-safety on Uu - blue
                color = 'b';
            else
                % Both on PC5 - green
                color = 'g';
            end
        end
        
        % Plot vehicle
        plot(vehicles(i).position(1), vehicles(i).position(2), 'o', 'MarkerSize', 6, ...
             'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
    end
    
    % Plot traffic signals
    if environment.signals.ns_state == "green"
        plot(5, 0, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'NS: Green');
        plot(0, 5, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', 'EW: Red');
    else
        plot(5, 0, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'DisplayName', 'NS: Red');
        plot(0, 5, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'EW: Green');
    end
    
    % Set plot limits and labels
    xlim([-environment.size(1)/2, environment.size(1)/2]);
    ylim([-environment.size(2)/2, environment.size(2)/2]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title(sprintf('V2X Simulation - t=%.1fs', 0));
    grid on;
    
    % Add legend for vehicle colors
    legend('', '', '', 'Macro Cell', 'Small Cell', ...
           'Vehicle (both Uu)', 'Vehicle (mix)', 'Vehicle (mix)', 'Vehicle (both PC5)', ...
           'NS Signal', 'EW Signal', 'Location', 'northeast');
    
    drawnow;
    hold off;
end