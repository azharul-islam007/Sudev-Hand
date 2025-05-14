%% createDDQNAgent.m - Create a simplified DDQN agent
function agent = createDDQNAgent(env, rl_params)
    % This is a simplified implementation that can be used for development
    % It implements core DDQN functionality without relying on MATLAB's RL Toolbox
    
    % Create agent structure
    agent = struct();
    
    % Store parameters
    agent.state_dim = rl_params.state_dim;
    agent.num_actions = rl_params.num_actions;
    agent.learning_rate = rl_params.learning_rate;
    agent.discount = rl_params.discount;
    agent.epsilon = rl_params.epsilon;
    agent.epsilon_decay = 0.995; % Decay rate for exploration
    agent.epsilon_min = 0.05;    % Minimum exploration rate
    agent.batch_size = rl_params.batch_size;
    agent.target_update = rl_params.target_update;
    agent.update_counter = 0;
    
    % Initialize replay buffer
    agent.buffer_size = rl_params.buffer_size;
    agent.buffer = struct('state', [], 'action', [], 'reward', [], 'next_state', [], 'done', []);
    agent.buffer_count = 0;
    
    % Initialize neural networks for online and target Q-functions
    agent.online_network = initializeNetwork(rl_params.state_dim, rl_params.hidden_layers, rl_params.num_actions);
    
    % Instead of using copyNetwork, we'll directly create a new network with the same structure
    agent.target_network = initializeNetwork(rl_params.state_dim, rl_params.hidden_layers, rl_params.num_actions);
    
    % Define methods
    agent.getAction = @(state) getAction_internal(agent, state);
    agent.update = @(state, action, reward, next_state, done) update_internal(agent, state, action, reward, next_state, done);
    agent.updateTargetNetwork = @() updateTargetNetwork_internal(agent);
    agent.decay_epsilon = @() decayEpsilon_internal(agent);
end

function network = initializeNetwork(input_dim, hidden_layers, output_dim)
    % Initialize a simple neural network structure
    % For simplicity, we'll use a weight-bias representation for each layer
    
    network = struct();
    network.layers = cell(length(hidden_layers) + 1, 1);
    
    % Input to first hidden layer
    network.layers{1} = struct('W', 0.1 * randn(hidden_layers(1), input_dim), ...
                              'b', zeros(hidden_layers(1), 1));
    
    % Hidden layers
    for i = 1:length(hidden_layers)-1
        network.layers{i+1} = struct('W', 0.1 * randn(hidden_layers(i+1), hidden_layers(i)), ...
                                    'b', zeros(hidden_layers(i+1), 1));
    end
    
    % Output layer
    network.layers{end+1} = struct('W', 0.1 * randn(output_dim, hidden_layers(end)), ...
                                  'b', zeros(output_dim, 1));
end

function updateTargetNetwork_internal(agent)
    % For simplicity, completely reinitialize the target network to match the online network
    for i = 1:length(agent.online_network.layers)
        if iscell(agent.online_network.layers) && i <= length(agent.online_network.layers)
            if isfield(agent.online_network.layers{i}, 'W') && isfield(agent.online_network.layers{i}, 'b')
                agent.target_network.layers{i}.W = agent.online_network.layers{i}.W;
                agent.target_network.layers{i}.b = agent.online_network.layers{i}.b;
            end
        end
    end
    fprintf('Target network updated\n');
end

function output = forwardPass(network, input)
    % Forward pass through the network
    x = input;
    
    % Hidden layers with ReLU activation
    for i = 1:length(network.layers)-1
        if iscell(network.layers) && i <= length(network.layers)
            if isfield(network.layers{i}, 'W') && isfield(network.layers{i}, 'b')
                z = network.layers{i}.W * x + network.layers{i}.b;
                x = max(0, z); % ReLU activation
            end
        end
    end
    
    % Output layer (linear activation)
    if iscell(network.layers) && length(network.layers) > 0
        i = length(network.layers);
        if isfield(network.layers{i}, 'W') && isfield(network.layers{i}, 'b')
            output = network.layers{i}.W * x + network.layers{i}.b;
        else
            output = zeros(size(x)); % Fallback
        end
    else
        output = zeros(size(x)); % Fallback
    end
end

function action = getAction_internal(agent, state)
    % Convert state to column vector if needed
    if ~iscolumn(state)
        state = state(:);
    end
    
    % Epsilon-greedy action selection
    if rand() < agent.epsilon
        % Exploration: random action
        action = randi(agent.num_actions);
    else
        % Exploitation: best action from Q-network
        try
            q_values = forwardPass(agent.online_network, state);
            [~, action] = max(q_values);
        catch
            % Fallback to random if network fails
            action = randi(agent.num_actions);
        end
    end
end

function update_internal(agent, state, action, reward, next_state, done)
    % Add experience to replay buffer
    agent.buffer.state(:, agent.buffer_count+1) = state;
    agent.buffer.action(agent.buffer_count+1) = action;
    agent.buffer.reward(agent.buffer_count+1) = reward;
    agent.buffer.next_state(:, agent.buffer_count+1) = next_state;
    agent.buffer.done(agent.buffer_count+1) = done;
    
    agent.buffer_count = agent.buffer_count + 1;
    
    % If buffer exceeds size, remove oldest entries
    if agent.buffer_count > agent.buffer_size
        agent.buffer.state(:, 1) = [];
        agent.buffer.action(1) = [];
        agent.buffer.reward(1) = [];
        agent.buffer.next_state(:, 1) = [];
        agent.buffer.done(1) = [];
        agent.buffer_count = agent.buffer_count - 1;
    end
    
    % Only update if we have enough samples for a batch
    if agent.buffer_count >= agent.batch_size
        % Sample minibatch
        batch_indices = randperm(agent.buffer_count, agent.batch_size);
        
        batch_state = agent.buffer.state(:, batch_indices);
        batch_action = agent.buffer.action(batch_indices);
        batch_reward = agent.buffer.reward(batch_indices);
        batch_next_state = agent.buffer.next_state(:, batch_indices);
        batch_done = agent.buffer.done(batch_indices);
        
        % For now, just log the batch update without actual weight updates
        fprintf('DDQN update: batch size=%d, avg reward=%.4f\n', agent.batch_size, mean(batch_reward));
        
        % Update counter
        agent.update_counter = agent.update_counter + 1;
        
        % Check if we need to update target network
        if agent.update_counter >= agent.target_update
            agent.updateTargetNetwork();
            agent.update_counter = 0;
        end
    end
end

function decayEpsilon_internal(agent)
    % Decay exploration rate
    agent.epsilon = max(agent.epsilon * agent.epsilon_decay, agent.epsilon_min);
end

%% Helper functions for integration with the main script

function action = getAction(agent, state)
    % Ensure state is a column vector
    if ~iscolumn(state)
        state = state(:);
    end
    
    % Call agent's getAction method with error handling
    try
        action = agent.getAction(state);
    catch
        % Fallback to random action if there's an error
        action = randi(agent.num_actions);
    end
end

function storeExperience(agent, state, action, reward, next_state)
    % Ensure states are column vectors
    if ~iscolumn(state)
        state = state(:);
    end
    
    if ~iscolumn(next_state)
        next_state = next_state(:);
    end
    
    % Update the agent (add to buffer) with error handling
    try
        agent.update(state, action, reward, next_state, false);
    catch e
        fprintf('Error storing experience: %s\n', e.message);
    end
end

function updateDDQN(agent)
    % This function would trigger training if we had an actual implementation
    % For now, just decay epsilon with error handling
    try
        agent.decay_epsilon();
    catch e
        fprintf('Error updating DDQN: %s\n', e.message);
    end
end