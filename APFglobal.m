% Fmap=Fmap1;
% startPt=[700 300];
% initPos=startPt;
% endPos=endPt;
% iter=1E3;

function path=APFglobal(Fmap,initPos,endPos,iter)
    [gX,gY]=gradient(-Fmap);
    
    path=initPos;
    currPt=initPos;
    v=3*[1 1];
    tol=50;
    
    while iter>0
        if norm(endPos-currPt)<tol
            break;
        end
        dirX=gX(floor(currPt(1)),floor(currPt(2)));
        dirY=gY(floor(currPt(1)),floor(currPt(2)));
        
        dirVec=[dirY dirX];
        
        dirNorm=dirVec./norm(dirVec);
        
        currPt=currPt+v.*dirNorm;
        
        path=[path;currPt];
        
        iter=iter-1;
    end
end