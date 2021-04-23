% ObservationInfo = rlNumericSpec([9 9]);
% ObservationInfo.Name = 'Robot View';
% ObservationInfo.Description = 'LocalPotential';
% 
% ActionInfo = rlFiniteSetSpec({[-1 1], [-1,1]});
% ActionInfo.Name = {'X speed' ; 'Y speed'};

function [InitialObservation, LoggedSignal] = myResetFunction()

Start = randi(100,1,2);
End = randi(100,1,2);
while (norm(Start - End) < 2)
    End = randi(100,1,2);
end
Obstacle = randi(100,1,2);
while (norm(Obstacle - End) < 2 || norm(Obstacle - Start) < 2)
    Obstacle = randi(100,1,2);
end

% Return initial environment state variables as logged signals.
LoggedSignal.State = [Start;End;Obstacle];
InitialObservation = LoggedSignal.State;
end
