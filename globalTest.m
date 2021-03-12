% load current test map and convert to grayscale
% map1=rgb2gray(imread('mapPoly1.png'));
map1=rgb2gray(imread('warehouse1.png'));
% map1=rgb2gray(imread('map1.png'));
% map1=rgb2gray(imread('mapBlank.png'));

% define starting and ending points
startPt=[300 700];
endPt=[140 125];
% define scaling constants and object radius
Kobj=500;
Kgoal=1/400;
Robj=1.5;

% calculate the potential map given current constants and map image
Umap1=imgToU1(map1,Kobj,endPt,Kgoal,Robj);
% run main path planning function
path=APFglobal(Umap1,startPt,endPt,1E3);

% display 2D map and flip y-axis to be consistent with 3D mesh plot
figure(1)
clf
imshow(map1)
title(sprintf('Initial APF with $K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %i$',Kobj,1/Kgoal,Robj),'Interpreter','latex')
hold on
ax=gca;
ax.YDir='normal';

% display 3D mesh representation where height (z-axis) is the force at a
% given point
figure(2)
clf
mesh(Umap1)
% use interpolated parula shading for better representation of height
shading interp
colormap parula
title(sprintf('Initial APF with $K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %i$',Kobj,1/Kgoal,Robj),'Interpreter','latex')
hold on
% iterate through all points along the point, calculate its z co-ord
% based on the force map, then plot the point on the 2D and 3D figures
for i=1:length(path)
    x=double(round(path(i,2)));
    y=double(round(path(i,1)));
    z=double(Umap1(y,x));
    figure(1)
%     make the path a bunch of red circles with 'ro'
    plot(x,y,'ro')
    figure(2)
    plot3(x,y,z,'ro')
end
% create text boxes for the starting and ending positions using sprintf's
% so it auto updates
figure(1)
text(startPt(2),startPt(1),sprintf('Starting Position\nX=%i, Y=%i',startPt(2),startPt(1)),'BackgroundColor','w','EdgeColor','k')
text(endPt(2),endPt(1),sprintf('Ending Position\nX=%i, Y=%i',endPt(2),endPt(1)),'BackgroundColor','w','EdgeColor','k')
% put a visual rectangle to see the field region on a white document
rectangle('Position',[1 1 800-1 450-1])

figure(2)
text(startPt(2),startPt(1),200+double(Umap1(startPt(1),startPt(2))),sprintf('Starting Position\nX=%i, Y=%i',startPt(2),startPt(1)),'BackgroundColor','w','EdgeColor','k')
text(endPt(2),endPt(1),200,sprintf('Ending Position\nX=%i, Y=%i',endPt(2),endPt(1)),'BackgroundColor','w','EdgeColor','k')