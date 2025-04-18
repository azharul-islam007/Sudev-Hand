%% createDDQNAgent.m - Create the DDQN agent
function agent = createDDQNAgent(env, rl_params)
    % Create observation and action paths
    obsPath = rlNumericSpec([rl_params.state_dim 1]);
    actPath = rlFiniteSetSpec(1:rl_params.num_actions);
    
    % Create Q-network layers
    layers = [
        featureInputLayer(rl_params.state_dim, 'Normalization', 'none', 'Name', 'input')
        fullyConnectedLayer(rl_params.hidden_layers(1), 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(rl_params.hidden_layers(2), 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(rl_params.num_actions, 'Name', 'output')
    ];
    
    % Create the online and target networks
    network = layerGraph(layers);
    
    % Create critic representation for DDQN
    criticOpts = rlRepresentationOptions('LearnRate', rl_params.learning_rate, ...
                                         'GradientThreshold', 1);
                                     
    critic = rlQValueRepresentation(network, obsPath, actPath, ...
                                    'Observation', {'input'}, ...
                                    'Action', {'output'}, ...
                                    criticOpts);
    
    % Set agent options
    agentOpts = rlDQNAgentOptions(...
        'SampleTime', 1, ...
        'TargetSmoothFactor', 1, ...
        'TargetUpdateFrequency', rl_params.target_update, ...
        'ExperienceBufferLength', rl_params.buffer_size, ...
        'MiniBatchSize', rl_params.batch_size, ...
        'DiscountFactor', rl_params.discount);
    
    % Create agent
    agent = rlDQNAgent(critic, agentOpts);
end