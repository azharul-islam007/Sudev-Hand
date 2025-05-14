%% storeExperience.m - Store experience in replay buffer
function storeExperience(agent, state, action, reward, nextState)
    % For development/testing, simply log the experience
    % This is a temporary solution to allow the simulation to run
    
    % In a real implementation with a properly configured agent, we would use:
    % experience.Observation = {state};
    % experience.Action = action;
    % experience.Reward = reward;
    % experience.NextObservation = {nextState};
    % agent.memory.add(experience);
    
    % During development, we'll just print occasionally to show progress
    if rand < 0.01  % Only print 1% of experiences to avoid console spam
        fprintf('Experience: Action=%d, Reward=%.2f\n', action, reward);
    end
    
    % No actual storage during development phase
end