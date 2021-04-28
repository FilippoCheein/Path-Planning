function [NextObs,Reward,IsDone,LoggedSignals]=multiObstacleStep(Action,LoggedSignals)
    
%     constants
    wCol=-10;
    wGoal=10;
    wTime=0.2;
    alpha=-20;
%     alpha=-1;
    
%     previous state and current location's potential value
    prevState=LoggedSignals.State;
    uPrev=LoggedSignals.uField(prevState(1),prevState(2));
    
%     apply the given action to get nextState
    nextState=round(prevState+Action);
    LoggedSignals.State=nextState;
    
%     check if outside of 100x100 environment
    wallC=min(LoggedSignals.State)<1||max(LoggedSignals.State)>100;
%     check if inside any obstacle
    obsC=false;
    for n=1:size(LoggedSignals.Obst,1)
        currObsC=norm(LoggedSignals.State-LoggedSignals.Obst(n,:))<=0.5;
        obsC=obsC||currObsC;
    end
%     both of those two are collision checks
    IsCol=obsC||wallC;
%     if we collided, then zero out the next observation cause we're gonna
%     terminate this run, and repeat single pt potential to not influence
%     reward function
    if IsCol
        uCurr=uPrev;
        NextObs=zeros(9);
    else
%         otherwise do normal single pt potential and observation
%         calculations
        uCurr=LoggedSignals.uField(LoggedSignals.State(1),LoggedSignals.State(2));
        NextObs=subsetBounded(LoggedSignals.uFieldPad,nextState,4);
    end
%     check if at goal, then done
    IsGoal=norm(LoggedSignals.State-LoggedSignals.Goal)<=0.5;
%     reward for change of the potential field
    wMove=alpha*(uCurr-uPrev);
%     apply reward
%     make sure not to give goal reward when "done" via collision
    Reward=wTime+wMove+wGoal*(IsGoal&&~IsCol)+wCol*(IsCol);
%     mark run as done in the case of success or failure via collision
    IsDone=IsGoal||IsCol;
end