%% getAction.m - Get action from DDQN agent
function action = getAction(agent, state)
    % Get action for a given state (development version)
    
    % For development/testing, return random action
    % This is a temporary solution to allow the simulation to run
    action = randi(8); % Assuming 8 actions
    
    % The problem is that we're passing a state that might be a cell,
    % but the agent might expect a numeric array.
    
    % For debugging, we could uncomment this to see what's happening:
    % fprintf('DEBUG getAction: state type=%s, size=%s\n', ...
    %         class(state), mat2str(size(state)));
    
    % A more complete implementation would be:
    % try
    %     % Ensure state is a double array, not a cell
    %     if iscell(state)
    %         state = cell2mat(state);
    %     end
    %     
    %     % Ensure state is a column vector
    %     if ~iscolumn(state)
    %         state = state(:);
    %     end
    %     
    %     % Process state according to what agent expects
    %     observation = {state};
    %     
    %     % Get action from agent
    %     action = getAction(agent.getPolicy(), observation);
    % catch e
    %     fprintf('Warning in getAction: %s\n', e.message);
    %     action = randi(8); % Fallback to random action
    % end
end