%% logData.m - Log simulation data for analysis
function logs = logData(logs, t, vehicles, environment, actions, transmissionStats)
    % Log various simulation data for later analysis
    %
    % Inputs:
    %   logs: The existing logs structure
    %   t: Current simulation time
    %   vehicles: Array of vehicle structures
    %   environment: Environment structure
    %   actions: Array of actions taken
    %   transmissionStats: Transmission statistics
    %
    % Outputs:
    %   logs: Updated logs structure with new data appended
    
    % Log vehicle positions
    for v = 1:length(vehicles)
        logs.vehicle_positions = [logs.vehicle_positions; 
                               t, v, vehicles(v).position(1), vehicles(v).position(2)];
    end
    
    % Log link states (which interface is being used)
    for v = 1:length(vehicles)
        % Convert safety interface to numeric (1=Uu, 2=PC5)
        if strcmp(vehicles(v).safety_interface, 'Uu')
            safety_interface = 1;
        else
            safety_interface = 2;
        end
        
        % Convert non-safety interface to numeric
        if strcmp(vehicles(v).nonsafety_interface, 'Uu')
            nonsafety_interface = 1;
        else
            nonsafety_interface = 2;
        end
        
        logs.link_states = [logs.link_states;
                         t, v, safety_interface, nonsafety_interface];
    end
    
    % Log actions taken
    for v = 1:length(vehicles)
        logs.actions = [logs.actions;
                     t, v, actions(v)];
    end
    
    % Periodically log simulation progress (every 5 seconds of sim time)
    if mod(t, 5) < 0.1
        fprintf('Simulation time: %.1f sec, Vehicles active: %d\n', t, length(vehicles));
    end
end