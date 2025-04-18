%% trafficSignal.m - Traffic signal control
function signals = trafficSignal(signals, delta_t)
    % Update traffic signal state based on timers
    
    % Decrement the active timer
    if signals.ns_state == "green"
        signals.ns_timer = signals.ns_timer - delta_t;
        
        % If timer expired, switch states
        if signals.ns_timer <= 0
            signals.ns_state = "red";
            signals.ew_state = "green";
            signals.ns_timer = signals.cycle_durations(2);  % Set to red duration
            signals.ew_timer = signals.cycle_durations(1);  % Set to green duration
        end
    else  % East-West is green
        signals.ew_timer = signals.ew_timer - delta_t;
        
        % If timer expired, switch states
        if signals.ew_timer <= 0
            signals.ns_state = "green";
            signals.ew_state = "red";
            signals.ns_timer = signals.cycle_durations(1);  % Set to green duration
            signals.ew_timer = signals.cycle_durations(2);  % Set to red duration
        end
    end
end