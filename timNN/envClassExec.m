env=envClass;
validateEnvironment(env);

% % define RL observation info (specs of our observation)
% ObservationInfo=rlNumericSpec([9 9]);
% ObservationInfo.Name='Robot Position (y,x)';
% 
% % define RL action info (possible actions)
% actions={[0 0],[-1 0],[-1 1],[0 1],[1 1],[1 0],[1 -1],[0 -1],[-1 -1]};
% ActionInfo=rlFiniteSetSpec(actions);
% ActionInfo.Name='Chosen [y,x] Velocity';

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
dlCriticNet=dlnetwork(criticNetwork);

criticOptions=rlRepresentationOptions('UseDevice','gpu');
critic=rlQValueRepresentation(dlCriticNet,obsInfo,actInfo,'Observation','r=4 Observation Input',criticOptions);
agentOptions=rlDQNAgentOptions;
agentOptions.EpsilonGreedyExploration.EpsilonDecay=0.05;
agentOptions.EpsilonGreedyExploration.EpsilonMin=0.05;

agent=rlDQNAgent(critic,agentOptions);

% copy paste training specs from MATLAB, remove stop criteria as our reward
% function shoots to 10^13 or something stupid like that
trainOpts=rlTrainingOptions;

trainOpts.MaxEpisodes=1000;
% trainOpts.MaxStepsPerEpisode=500;
trainOpts.StopTrainingCriteria = "EpisodeCount";
trainOpts.StopTrainingValue = 1000;
trainOpts.ScoreAveragingWindowLength = 5;
% trainOpts.SaveAgentCriteria = "EpisodeReward";
% trainOpts.SaveAgentValue = 500;
trainOpts.SaveAgentDirectory = "savedAgents";
trainOpts.Verbose = false;
trainOpts.Plots = "training-progress";

% trainOpts.UseParallel=true;
% trainOpts.ParallelizationOptions.Mode="async";
% trainOpts.ParallelizationOptions.DataToSendFromWorkers="experiences";
% trainOpts.ParallelizationOptions.StepsUntilDataIsSent=32;

plot(env)
trainingInfo = train(agent,env,trainOpts);