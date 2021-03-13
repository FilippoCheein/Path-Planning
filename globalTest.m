% load current test map and convert to grayscale
% map1=rgb2gray(imread('mapPoly1.png'));
map1=rgb2gray(imread('warehouse1.png'));
% map1=rgb2gray(imread('map1.png'));
% map1=rgb2gray(imread('mapBlank.png'));

% define starting and ending points
startPt=[300 700];
endPt=[140 125];
% define scaling constants and object radius
Kobj=300;
Kgoal=1/200;
Robj=1.1;

% calculate the potential map given current constants and map image
% Umap1=imgToU1(map1,Kobj,endPt,Kgoal,Robj);
% use second iteration imgToU2 that deals with GNRON
Umap1=imgToU2(map1,Kobj,endPt,Kgoal,Robj,3);
% run main path planning function
path=APFglobal(Umap1,startPt,endPt,1E3);

% calculate vectors for path's x y and z variables
x=double(round(path(:,2)));
y=double(round(path(:,1)));
% due to x and y being vectors, superposition gets created and we just want
% the minimum value so min function
z=min(double(Umap1(y,x)));

% display 2D map and flip y-axis to be consistent with 3D mesh plot
figure(1)
clf
imshow(map1)
title(sprintf('Initial APF with $K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %i$',Kobj,1/Kgoal,Robj),'Interpreter','latex')
hold on
plot(x,y,'r','LineWidth',3)
% mark the start point as a green diamond and end point as a green circle
plot(endPt(2),endPt(1),'wd','MarkerSize',15,'MarkerFaceColor','#77AC30')
plot(startPt(2),startPt(1),'wo','MarkerSize',15,'MarkerFaceColor','#77AC30')
rectangle('Position',[1 1 800-1 450-1])
ax=gca;
ax.YDir='normal';

% display 3D mesh representation where height (z-axis) is the potential at a
% given point
figure(2)
clf
mesh(Umap1)
% use interpolated parula shading for better visualization of height
shading interp
colormap parula
title(sprintf('Initial APF with $K_{Object} = %i, K_{Goal} = \\frac{1}{%i}, R_{Object} = %i$',Kobj,1/Kgoal,Robj),'Interpreter','latex')
hold on
plot3(x,y,z,'ro')
% mark the start point as a green diamond and end point as a green circle
plot3(endPt(2),endPt(1),double(Umap1(endPt(1),endPt(2))),'wd','MarkerSize',15,'MarkerFaceColor','#77AC30')
plot3(startPt(2),startPt(1),double(Umap1(startPt(1),startPt(2))),'wo','MarkerSize',15,'MarkerFaceColor','#77AC30')