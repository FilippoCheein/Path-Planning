function [InitialObservation,LoggedSignals]=multiObstacleReset()

%     Choose # of obstacles randomly from 1-10
    nObst=randi(10);
%     Generate Y and X co-ords of obstacle(s), goal, and start point
%     randomly, with all Y,X pairs being unique
    yVec=randperm(100,2+nObst);
    xVec=randperm(100,2+nObst);
%     set the state goal and obstacle's pairs
    init=[yVec(1),xVec(1)];
    LoggedSignals.State=init;
    LoggedSignals.Goal=[yVec(2),xVec(2)];
    LoggedSignals.Obst=[yVec(3:end).',xVec(3:end).'];
%     use non-padding coords to potential function to make uField
    LoggedSignals.uField=coordsToU(LoggedSignals,0);
%     use padding coords to potential fucntion to then use for observation
%     bounding box
    rPad=4;
    LoggedSignals.uFieldPad=coordsToUPad(LoggedSignals,rPad,0);
%     for observation use padded field w/ subsetBounded
    InitialObservation=subsetBounded(LoggedSignals.uFieldPad,init,rPad);
end