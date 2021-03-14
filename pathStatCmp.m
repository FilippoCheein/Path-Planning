function [statS,path,uMap]=pathStatCmp(inS,figN)
%     [statS,path,uMap]=pathStatCmp(inS,figN)
%     inputs:
%       inS input fields:
%           inS.gray=grayscale map array
%           inS.startPt=map's starting point in [y x] format
%           inS.endPt=map's ending point in [y x] format
%           inS.name=map's name as a string

%       figN=figure number to plot all 3 2D and 3D map representations
%     outputs:
%       statS output fields:
%           statS.methodN=Obstacle Potential Method Number
%           statS.pathDist=Distance Along Path
%           statS.numIterations=Number of Iterations for Path
%           statS.calcTime=Calculation Time
%           statS.kGoal=Goal Potential Scaling Coefficient
%           statS.kObst=Obstacle Potential Scaling Coefficient
%           statS.rObst=Obstacle Radius of Influence
%       path=cell array containing the input map's three different path
%           sets, one for each method, the path is a Mx2 matrix where M is
%           number of points on path and each point is [y x]
%       uMap=cell array containing the input map's three different
%           potential field matrices, one for each method, the uMap is the same
%           size as the input map, and each point is the potential for the
%           given method

%     initialize output variables
    statS=struct;
    path=cell(3,1);
    uMap=path;
    
%     initialize constants with each column representing its method's
%     paramater
    kGoal=[1/400 1/400 1/100];
    kObst=[500 5 1/1000];
    rObst=[1.5 1.2 1.2];
%     for polygon2
%     kGoal=[1/400 1/400 1/100];
%     kObst=[500 6 1/10];
%     rObst=[1.5 1.3 1.5];
    
    method=[0 1 3];
    maxIter=1E3;
%     iterate through all three methods
    for n=1:3
%         store the method number for table outputting
        statS(n).methodN=method(n);
%         start a timer to measure computation time
        tic
%         compute the potential map for the current method
        uMap{n}=imgToU2(inS.gray,kObst(n),inS.endPt,kGoal(n),rObst(n),method(n));
%         compute the path for the current method
        path{n}=APFglobal(uMap{n},inS.startPt,inS.endPt,maxIter);
%         end the timer as the rest is just data re-assignment
        currCalc=toc;
%         calculate the distance along the path
        statS(n).pathDist=sum(vecnorm(diff(path{n}),2,2));
%         calculate the number of iterations (how long path{n} is)
        statS(n).numIterations=length(path{n});
%         store the timer result
        statS(n).calcTime=currCalc;
%         store the constants for this method
        statS(n).kGoal=kGoal(n);
        statS(n).kObst=kObst(n);
        statS(n).rObst=rObst(n);
    end
%     plotting time
methodName={'Basic APF','n=1 Euclidean Distance APF','n=3 Euclidean Distance APF'};
    if figN>0
        figure(figN)
        clf
        tiledlayout(2,3,'TileSpacing','tight','Padding','tight')
        sgtitle(sprintf('%s Map',inS.name))
        for n=1:3
%             calculate current method's x and y vectors
            x=double(round(path{n}(:,2)));
            y=double(round(path{n}(:,1)));
%             use sub2ind to convert the two input row vectors into a
%             single row vector of indices rather than a square matrix of
%             indices
            z=(double(uMap{n}(sub2ind(size(uMap{n}),y,x))));
            nexttile
            mesh(uMap{n})
            shading interp
            colormap parula
            title(methodName{n})
            hold on
            plot3(x,y,z,'ro')
%             calculate the starting and ending z co-ord based on uMap
            startZ=double(uMap{n}(inS.startPt(1),inS.startPt(2)));
            endZ=double(uMap{n}(inS.endPt(1),inS.endPt(2)));
%             mark the start point as a green circle
            plot3(inS.startPt(2),inS.startPt(1),startZ...
                ,'wo','MarkerSize',15,'MarkerFaceColor','#77AC30')
%             mark the end point as a green diamond
            plot3(inS.endPt(2),inS.endPt(1),endZ...
                ,'wd','MarkerSize',15,'MarkerFaceColor','#77AC30')
        end
        for n=1:3
%             make pretty 1/x instead of 0.000x format if needed
            if kObst(n)<1
                titles=sprintf('$K_{Object} = \\frac{1}{%i}, K_{Goal} = \\frac{1}{%i}, R_{Object} = %3.1f$',1/kObst(n),1/kGoal(n),rObst(n));
            else
                titles=sprintf('$K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %3.1f$',kObst(n),1/kGoal(n),rObst(n));
            end
%             calculate current method's x and y vectors
            x=double(round(path{n}(:,2)));
            y=double(round(path{n}(:,1)));
            nexttile
            imshow(inS.gray)
            title(titles,'Interpreter','latex')
%             use xlabel to display the number of iterations for quick look
%             on how efficient a given method us
            xlabel(sprintf('# of Iterations = %i',statS(n).numIterations))
            hold on
            plot(x,y,'ro')
%             mark the start point as a green circle
            plot(inS.startPt(2),inS.startPt(1)...
                ,'wo','MarkerSize',15,'MarkerFaceColor','#77AC30')
%             mark the end point as a green diamond
            plot(inS.endPt(2),inS.endPt(1)...
                ,'wd','MarkerSize',15,'MarkerFaceColor','#77AC30')
%             add a black border to better see map boundaries
            rectangle('Position',[1 1 800-1 450-1])
%             flip the map so the bottom left correspond with the 3d mesh's
%             bottom left for easier intuition viewing
            ax=gca;
            ax.YDir='normal';
        end
    end
end