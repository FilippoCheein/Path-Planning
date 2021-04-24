function [NextObs,Reward,IsDone,LoggedSignals] = myStepFunction(Action,LoggedSignals)

katt = 1;
krep = 8;
ro = 3;
Wcollision = -10;
Wgoal = 10;
Wtime = 0.2;
alpha = -20;



PrevState = LoggedSignal.State;

UtPrev = 

%Check Terminal Position
WallC = min(LoggedSignal.State(1)) < 1 || max(LoggedSignal.State(1)) > 100;
ObsC = norm(LoggedSignal.State(1) - LoggedSignal.State(3)) <= 0.5;
Collision = ObsC || WallC;
IsDone = norm(LoggedSignal.State(1) - LoggedSignal.State(2)) <= 0.5;

%Get reward
Wmove = alpha*(Ut - UtPrev);
Reward = Wtime + Wmove + Wgoal*(IsDone) + Wcollision*(Collision);