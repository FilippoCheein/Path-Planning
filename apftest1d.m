% 1D artificial potential field test
close all
clear

% setting up the starting and end postions, the speed, the maximum number
% of iterations and the tolerance for the robot
x = 1:1:100;
start_pos = 1;
goal_pos = 50;
v = 3;
tol = 2;
iter = 20;

% creating the attractive force map
ka = 1;
fa = ka*(x-goal_pos(1)).^2;
% find the negative gradient to plot
gx = gradient(-fa);
% get the robots path and end position
path = apf1D(fa,start_pos,goal_pos,v,tol,iter);
end_pos = path(end);

%plot the attractive force map, goal position, and the gradient
figure;
hold on
grid on
plot(x,fa,'r');
plot(x,gx,'m');
plot(goal_pos,0,'go');

% loop through each point in the path and plot it
for i = 1:size(path)
    plot(path(i),0,'b*');
end

% labeling information for the graph
legend('Attractive Force','Gradient of the Attractive Force','Goal Position','Robot Position');
title('1D Artificial Potential Field');
text(start_pos,500,sprintf('Starting Position\nX=%i',start_pos),'BackgroundColor','w','EdgeColor','k')
text(end_pos,500,sprintf('Ending Position\nX=%i',end_pos),'BackgroundColor','w','EdgeColor','k')