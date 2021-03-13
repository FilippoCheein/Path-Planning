function path = apf1D(fmap, start_pos, goal_pos, v, tol, iter)
% Inputs:   fmap = force map
%           start_pos = starting x position
%           goal_pos = x position of the goal
%           v = step size for each point in the path
%           tol = tolerance for the end position
%           iter = maximum number of iterations 

% Output:   path = list of positions to move to, to end at the goal
%           within the tolerance 

% start the path at the starting positon
currPt = start_pos;
path = [currPt];

% find the negative gradient of the force map
gX=gradient(-fmap);

% while loop to control number of iterations/steps
    while iter>0
% get the gradient for the current point
        dirX=gX(floor(currPt));
% get the direction that the robot is moving in
        dirNorm=dirX/norm(dirX);
% scale the direction unit vector by the speed and assign-add to
% get the next current point
        currPt=currPt+v.*dirNorm;
% add this point to the path, as a new row not column
        path=[path;currPt];
% break the loop if we are within the goal's radius tol
        if norm(goal_pos-currPt)<tol
            break;
        end
% decrement iterations to keep track
        iter=iter-1;
    end
end