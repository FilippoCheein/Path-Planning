function [statS,path,uMap]=pathStatCmp(inS,figN)


%     output fields:
%       statS.methodN=Obstacle Potential Method Number
%       statS.pathDist=Distance Along Path
%       statS.numIterations=Number of Iterations for Path
%       statS.calcTime=Calculation Time
%       statS.kGoal=Goal Potential Scaling Coefficient
%       statS.kObst=Obstacle Potential Scaling Coefficient
%       statS.rObst=Obstacle Radius of Influence
    statS=struct;
    path=cell(3,1);
    uMap=path;
    kGoal=[1/400 1/400 1/100];
    kObst=[500 5 1/1000];
    rObst=[1.5 1.2 1.2];
    method=[0 1 3];
    maxIter=1E3;
    for n=1:3
        statS(n).methodN=method(n);
        tic
        uMap{n}=imgToU2(inS.gray,kObst(n),inS.endPt,kGoal(n),rObst(n),method(n));
        path{n}=APFglobal(uMap{n},inS.startPt,inS.endPt,maxIter);
        currCalc=toc;
        currIter=length(path{n});
        statS(n).pathDist=sum(vecnorm(diff(path{n}),2,2));
        statS(n).numIterations=currIter;
        statS(n).calcTime=currCalc;
        statS(n).kGoal=kGoal(n);
        statS(n).kObst=kObst(n);
        statS(n).rObst=rObst(n);
    end
    
%     plotting time
    if figN>0
        figure(figN)
        clf
        tiledlayout(2,3,'TileSpacing','tight','Padding','tight')
        for n=1:3
            if kObst(n)<1
                titles=sprintf('$K_{Object} = \\frac{1}{%i}, K_{Goal} = \\frac{1}{%i}, R_{Object} = %3.1f$',1/kObst(n),1/kGoal(n),rObst(n));
            else
                titles=sprintf('$K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %3.1f$',kObst(n),1/kGoal(n),rObst(n));
            end
            x=double(round(path{n}(:,2)));
            y=double(round(path{n}(:,1)));
            z=(double(uMap{n}(sub2ind(size(uMap{n}),y,x))));
            nexttile
            mesh(uMap{n})
            shading interp
            colormap parula
            title(titles,'Interpreter','latex')
            hold on
            plot3(x,y,z,'ro')
            % mark the start point as a green diamond and end point as a green circle
            startZ=double(uMap{n}(inS.startPt(1),inS.startPt(2)));
            endZ=double(uMap{n}(inS.endPt(1),inS.endPt(2)));
            plot3(inS.startPt(2),inS.startPt(1),startZ...
                ,'wo','MarkerSize',15,'MarkerFaceColor','#77AC30')
            plot3(inS.endPt(2),inS.endPt(1),endZ...
                ,'wd','MarkerSize',15,'MarkerFaceColor','#77AC30')
        end
        for n=1:3
            if kObst(n)<1
                titles=sprintf('$K_{Object} = \\frac{1}{%i}, K_{Goal} = \\frac{1}{%i}, R_{Object} = %3.1f$',1/kObst(n),1/kGoal(n),rObst(n));
            else
                titles=sprintf('$K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %3.1f$',kObst(n),1/kGoal(n),rObst(n));
            end
            x=double(round(path{n}(:,2)));
            y=double(round(path{n}(:,1)));
            nexttile
            imshow(inS.gray)
            title(titles,'Interpreter','latex')
            xlabel(sprintf('# of Iterations = %i',statS(n).numIterations))
            hold on
            plot(x,y,'ro')
            % mark the start point as a green diamond and end point as a green circle
            plot(inS.startPt(2),inS.startPt(1)...
                ,'wo','MarkerSize',15,'MarkerFaceColor','#77AC30')
            plot(inS.endPt(2),inS.endPt(1)...
                ,'wd','MarkerSize',15,'MarkerFaceColor','#77AC30')
            rectangle('Position',[1 1 800-1 450-1])
            ax=gca;
            ax.YDir='normal';
        end
    end
end