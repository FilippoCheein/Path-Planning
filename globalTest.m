% map1=rgb2gray(imread('mapPoly1.png'));
map1=rgb2gray(imread('warehouse1.png'));
% map1=rgb2gray(imread('map1.png'));
% map1=rgb2gray(imread('mapBlank.png'));

startPt=[300 700];
endPt=[140 125];

Kobj=500;
Kgoal=1/400;
Robj=1.5;

Fmap1=imgToF1(map1,Kobj,endPt,Kgoal,Robj);
path=APFglobal(Fmap1,startPt,endPt,1E3);

figure(1)
clf
imshow(map1)
title(sprintf('Initial APF with $K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %i$',Kobj,1/Kgoal,Robj),'Interpreter','latex')
hold on
ax=gca;
ax.YDir='normal';

figure(2)
clf
mesh(Fmap1)
title(sprintf('Initial APF with $K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %i$',Kobj,1/Kgoal,Robj),'Interpreter','latex')
hold on
for i=1:length(path)
    x=double(round(path(i,2)));
    y=double(round(path(i,1)));
    z=double(Fmap1(y,x));
    figure(1)
    plot(x,y,'ro')
    figure(2)
    plot3(x,y,z,'ro')
end
shading interp
colormap parula
figure(1)
text(startPt(2),startPt(1),sprintf('Starting Position\nX=%i, Y=%i',startPt(2),startPt(1)),'BackgroundColor','w','EdgeColor','k')
text(endPt(2),endPt(1),sprintf('Ending Position\nX=%i, Y=%i',endPt(2),endPt(1)),'BackgroundColor','w','EdgeColor','k')
rectangle('Position',[1 1 800-1 450-1])

figure(2)
text(startPt(2),startPt(1),200+double(Fmap1(startPt(1),startPt(2))),sprintf('Starting Position\nX=%i, Y=%i',startPt(2),startPt(1)),'BackgroundColor','w','EdgeColor','k')
text(endPt(2),endPt(1),200,sprintf('Ending Position\nX=%i, Y=%i',endPt(2),endPt(1)),'BackgroundColor','w','EdgeColor','k')