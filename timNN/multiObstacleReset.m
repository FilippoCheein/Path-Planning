% function [InitialObservation,LoggedSignals]=multiObstacleReset()

    nObst=randi(10);
    yVec=randperm(100,2+nObst);
    xVec=randperm(100,2+nObst);
    LoggedSignals.State=[yVec(1),xVec(1)];
    LoggedSignals.Goal=[yVec(2),xVec(2)];
    LoggedSignals.Obst=[yVec(3:end).',xVec(3:end).'];
    LoggedSignals.uField=coordsToU(LoggedSignals);
    InitialObservation=LoggedSignals.State;

% end