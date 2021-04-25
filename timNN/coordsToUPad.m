function [uField]=coordsToUPad(LoggedSignals,rPad,figN)
%     [uField]=coordsToUPad(LoggedSignalsm,rPad,figN)
%     inputs:
%       LoggedSignals.State=[y,x] co-ords of robot
%       LoggedSignals.Goal=[y,x] co-ords of goal
%       LoggedSignals.Obst=Nx2 matrix where the n-th column pair [y,x]
%       represents the n-th obstacle, with N = number of obstacles
%       rPad=how many pixels to pad (extra calculate) uField by
%       figN=zero for no figure plotting, otherwise specify figure number
%     outputs:
%         uField=potential field array
    
%     constants for APF
    kAtt=1;
    kRep=20;
    rRep=3;
    
%     run x and y from +/- rPad so that offsets are nice
    [x,y]=meshgrid(1-rPad:100+rPad,1-rPad:100+rPad);
    
%     calculate distance from every point to the goal
    yGoal=y-LoggedSignals.Goal(1);
    xGoal=x-LoggedSignals.Goal(2);
    
%     calculate attractive potential based on above distance
    uAtt=0.5*kAtt*(yGoal.^2+xGoal.^2);
    
%     pre-allocate repulsive field so MATLAB doesn't yell at me
    uRep=zeros(100+2*rPad);
    for n=1:size(LoggedSignals.Obst,1)
%         calculate distance from every point to n-th obstacle
        currY=y-LoggedSignals.Obst(n,1);
        currX=x-LoggedSignals.Obst(n,2);
        currDist=sqrt(currY.^2+currX.^2);
        
%         calculate n-th obstacle's potential repulsive field
        currRep=0.5*kRep*(1./(currDist+0.05)-1/rRep).^2;
%         add up all with a step function for domain'ing
        uRep=uRep+currRep.*(currDist<=rRep);
    end
    
%     combine attractive and repulsive for net field
    uField=uAtt+uRep;
%     plot current environment if figN is greater than 0
    if figN>0
        figure(figN)
        clf
        mesh(uField)
        shading interp
        colormap parula
        xlabel('x axis')
        ylabel('y axis')
        hold on
%         calculate the starting and ending z co-ord based on uField
        startZ=double(uField(LoggedSignals.State(1),LoggedSignals.State(2)));
        endZ=double(uField(LoggedSignals.Goal(1),LoggedSignals.Goal(2)));
%         mark the start point as a green circle
        plot3(LoggedSignals.State(2),LoggedSignals.State(1),startZ...
            ,'wo','MarkerSize',15,'MarkerFaceColor','g')
%         mark the end point as a green diamond
        plot3(LoggedSignals.Goal(2),LoggedSignals.Goal(1),endZ...
            ,'wd','MarkerSize',15,'MarkerFaceColor','r')
    end
end