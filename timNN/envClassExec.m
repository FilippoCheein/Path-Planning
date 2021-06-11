% initialize environment class object
env=envClass;
validateEnvironment(env);

% use class functions to initialize observation and actions
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% NN time
criticNetwork=[
    imageInputLayer([obsInfo.Dimension 1],'Normalization','none','Name','r=4 Observation Input')
%     sequenceInputLayer(prod(obsInfo.Dimension),'Name','r=4 Observation Input')
%     after input, processed by two layers of 2-D convolutions of 16
%     neurons
%     16 neurons = output height * output width
    convolution2dLayer([3 3],2,'Padding','Same','Name','2D 16 Neurons #1')
%     convolution2dLayer([3 3],1,'Stride',1,'Name','2D 16 Neurons #2')
%     convolution2dLayer([3 3],2,'Name','2D 16 Neurons #1')
%     convolution2dLayer([2 2],1,'Name','2D 16 Neurons #2')
%     with leaky relu activation function
    leakyReluLayer('Name','Leaky Relu Activation')
%     operated by maxpooling operation
    maxPooling2dLayer([3 3],'Stride',[2 2],'Name','Max Pooling 2x2')
%     followed by two fully-connected layers of 32 neurons
    fullyConnectedLayer(32,'Name','Fully-Connected Penultimate Output')
%     ouputs the 9 action's different estimated reward value
    fullyConnectedLayer(numel(actInfo.Elements),'Name','9 Actions Estimated Rewards')
%     softmaxLayer('Name','Softmax Output')
%     classificationLayer('Name','Classification Output')
    ];

% construct NN deep learning object
dlCriticNet=dlnetwork(criticNetwork);
% use GPU as its better optimized, also crank down learning rate
criticOptions=rlRepresentationOptions('UseDevice','gpu','LearnRate',0.001);
% create critic object for use in agent creation
critic=rlQValueRepresentation(dlCriticNet,obsInfo,actInfo,'Observation','r=4 Observation Input',criticOptions);
agentOptions=rlDQNAgentOptions;
% specify DQN epsilon greedy decay rate and minimum
agentOptions.EpsilonGreedyExploration.EpsilonDecay=0.05;
agentOptions.EpsilonGreedyExploration.EpsilonMin=0.05;
% create agent object
agent=rlDQNAgent(critic,agentOptions);


trainOpts=rlTrainingOptions;
% specify training so each run only has 250 steps, and 
trainOpts.MaxEpisodes=10000;
trainOpts.MaxStepsPerEpisode=250;
trainOpts.StopTrainingCriteria = "EpisodeCount";
trainOpts.StopTrainingValue = 10000;
trainOpts.ScoreAveragingWindowLength = 5;
trainOpts.SaveAgentDirectory = "savedAgents";
trainOpts.Verbose = false;
trainOpts.Plots = "training-progress";

plot(env)
trainingInfo = train(agent,env,trainOpts);