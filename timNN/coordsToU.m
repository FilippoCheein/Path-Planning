function [uField]=coordsToU(LoggedSignals,figN)
    kAtt=1;
    kRep=20;
    rRep=3;
    
    [x,y]=meshgrid(1:100,1:100);
    yGoal=y-LoggedSignals.Goal(1);
    xGoal=x-LoggedSignals.Goal(2);
    
    uAtt=0.5*kAtt*(yGoal.^2+xGoal.^2);
    
    uRep=zeros(100);
%     regions=zeros(100);
%     tiledlayout(3,1,'TileSpacing','tight','Padding','tight')
%     nexttile
    for n=1:size(LoggedSignals.Obst,1)
        currY=y-LoggedSignals.Obst(n,1);
        currX=x-LoggedSignals.Obst(n,2);
        currDist=sqrt(currY.^2+currX.^2);
        currRep=0.5*kRep*(1./(currDist+0.05)-1/rRep).^2;
        uRep=uRep+currRep.*(currDist<=rRep);
%         regions=imoverlay(regions,currDist);
%         imshow(regions)
    end
    uField=uAtt+uRep;
%     max(uRep,[],'all')
%     nexttile
%     mesh(uRep)
%     nexttile
    if figN>0
        figure(figN)
        clf
        mesh(uField)
        shading interp
        colormap parula
        xlabel('x axis')
        ylabel('y axis')
        hold on
%         calculate the starting and ending z co-ord based on uMap
        startZ=double(uField(LoggedSignals.State(1),LoggedSignals.State(2)));
        endZ=double(uField(LoggedSignals.Goal(1),LoggedSignals.Goal(2)));
%         mark the start point as a green circle
        plot3(LoggedSignals.State(2),LoggedSignals.State(1),startZ...
            ,'wo','MarkerSize',15,'MarkerFaceColor','g')
%         mark the end point as a green diamond
        plot3(LoggedSignals.Goal(2),LoggedSignals.Goal(1),endZ...
            ,'wd','MarkerSize',15,'MarkerFaceColor','r')
    end
    
% end