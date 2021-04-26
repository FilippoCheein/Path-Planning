env=envClass;
validateEnvironment(env);

% define RL observation info (specs of our observation)
ObservationInfo=rlNumericSpec([9 9]);
ObservationInfo.Name='Robot Position (y,x)';

% define RL action info (possible actions)
actions={[0 0],[-1 0],[-1 1],[0 1],[1 1],[1 0],[1 -1],[0 -1],[-1 -1]};
ActionInfo=rlFiniteSetSpec(actions);
ActionInfo.Name='Chosen [y,x] Velocity';

agent=rlDQNAgent(ObservationInfo,ActionInfo);

% copy paste training specs from MATLAB, remove stop criteria as our reward
% function shoots to 10^13 or something stupid like that
trainOpts=rlTrainingOptions;

trainOpts.MaxEpisodes=1000;
trainOpts.MaxStepsPerEpisode=500;
% trainOpts.StopTrainingCriteria = "AverageReward";
% trainOpts.StopTrainingValue = 500;
trainOpts.ScoreAveragingWindowLength = 5;
% trainOpts.SaveAgentCriteria = "EpisodeReward";
% trainOpts.SaveAgentValue = 500;
trainOpts.SaveAgentDirectory = "savedAgents";
trainOpts.Verbose = false;
trainOpts.Plots = "training-progress";
plot(env)
trainingInfo = train(agent,env,trainOpts);