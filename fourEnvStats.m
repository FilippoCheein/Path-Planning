stats=cell(1,4);
path=stats;
uMap=stats;
map=struct;

map(1).name='Polygon';
map(2).name='Local Minima';
map(3).name='Warehouse';
map(4).name='Warehouse';

for nMap=1:4
    map(nMap).gray=rgb2gray(imread(['fourEnv\' sprintf('%s Map.png',map(nMap).name)]));
    if (nMap~=4); map(nMap).startPt=[300 700]; end
    if (nMap==4); map(nMap).startPt=[50 700]; end
end
% 
map(1).endPt=[100 100];
map(2).endPt=map(1).endPt;
map(3).endPt=[140 125];
map(4).endPt=[400 25];
% 
% map(1).kGoal=1/400;
% map(2).kGoal=map(1).kGoal;
% map(3).kGoal=1/400;
% map(4).kGoal=map(3).kGoal;
% 
% map(1).kObst=500;
% map(2).kObst=map(1).kObst;
% map(3).kObst=100;
% map(4).kObst=map(3).kObst;
% 
% map(1).rObst=1.5;
% map(2).rObst=map(1).rObst;
% map(3).rObst=5;
% map(4).rObst=map(3).rObst;

for nMap=1:4
    [stats{nMap},path{nMap},uMap{nMap}]=pathStatCmp(map(nMap),nMap);
    currTable=struct2table(stats{nMap});
    writetable(currTable,'metrics.xlsx','Sheet',1,'Range',sprintf('B%i',2+(nMap-1)*8));
end

% excel time
