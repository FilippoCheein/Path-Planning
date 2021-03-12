function path=APFglobal(Fmap,initPos,endPos,iter)
%     path=APFglobal(Fmap,initPos,endPos,iter)
%     inputs:
%       Fmap=force map, a matrix that holds every point's force
%       initPos=agent starting point in [y x] format
%       endPos=agent goal point in [y x] format
%       iter=limit to how many iterations before stopping code execution so
%       we can get see failed path planning scenarios
%     outputs:
%       path=all points along the path, a Mx2 matrix with M being number of
%       steps, maximum of 1000, each row is in the form [y x]
    
%     initialize the path and currPt variables
    path=initPos;
    currPt=initPos;
%     speed of movement (scales gradient direction)
    v=3*[1 1];
%     max distance from goal to consider the run a success
    tol=50;
    
%     calculate the negative gradient of the attractive force along the x
%     and y axis
    [gX,gY]=gradient(-Fmap);
    
%     while loop to control number of iterations/steps to 1000
    while iter>0
%         break the loop if we are within the goal's radius tol
        if norm(endPos-currPt)<tol
            break;
        end
%         get the gradient x and y for the current point
        dirX=gX(floor(currPt(1)),floor(currPt(2)));
        dirY=gY(floor(currPt(1)),floor(currPt(2)));
%         store these in a vector
        dirVec=[dirY dirX];
%         convert this to a unit vector for direction only by dividing by
%         the norm
        dirNorm=dirVec./norm(dirVec);
%         scale the direction unit vector by the speed and assign-add to
%         get the next current point
        currPt=currPt+v.*dirNorm;
%         add this point to the path, as a new row not column
        path=[path;currPt];
%         decrement iterations to keep track
        iter=iter-1;
    end
end