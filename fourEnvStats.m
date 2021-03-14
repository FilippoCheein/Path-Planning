% initialize output variables
stats=cell(1,4);
path=stats;
uMap=stats;
map=struct;

% store map names in struct field name
map(1).name='Polygon';
map(2).name='Local Minima';
% map(2).name='Polygon2';
map(3).name='Warehouse';
map(4).name='Warehouse';

% load in all four maps based on their name field, also initialize starting
% points
for nMap=1:4
    map(nMap).gray=rgb2gray(imread(['fourEnv\' sprintf('%s Map.png',map(nMap).name)]));
    if (nMap~=4); map(nMap).startPt=[300 700]; end
    if (nMap==4); map(nMap).startPt=[50 700]; end
end

% initialize end points
map(1).endPt=[100 100];
map(2).endPt=map(1).endPt;
map(3).endPt=[140 125];
map(4).endPt=[400 25];

% run single-map three-method comparison
for nMap=1:4
    [stats{nMap},path{nMap},uMap{nMap}]=pathStatCmp(map(nMap),nMap);
%     convert to table then output to excel metrics.xlsx file
    currTable=struct2table(stats{nMap});
    writetable(currTable,'metrics.xlsx','Sheet',1,'Range',sprintf('B%i',2+(nMap-1)*8));
end
