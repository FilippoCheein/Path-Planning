function [uField]=coordsToU(LoggedSignals)
    kAtt=1;
    kRep=20;
    rRep=3;
    
    [x,y]=meshgrid(1:100,1:100);
    yGoal=y-LoggedSignals.Goal(1);
    xGoal=x-LoggedSignals.Goal(2);
    
    uAtt=0.5*kAtt*(yGoal.^2+xGoal.^2);
    
    uRep=zeros(100);
%     regions=zeros(100);
    figure(1)
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
    mesh(uField)
    shading interp
    colormap parula
    xlabel('x axis')
    ylabel('y axis')
    
% end