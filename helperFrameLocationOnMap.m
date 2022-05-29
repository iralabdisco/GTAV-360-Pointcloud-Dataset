frame = 330;
runName = 'canyon_sequence';
dataFolder = fullfile(getenv('DEEPGTAV_EXPORT_DIR'), runName, filesep);
locationFile = fullfile(dataFolder, 'location.txt');
framePointCloudFileName = fullfile(dataFolder, 'velodyne_360', sprintf('%06d.pcd', frame));
frameLocationFileName = fullfile(dataFolder, 'location_360', sprintf('%06d.txt', frame));
mapName = strcat(runName,'_complete_map.pcd');
mapFile = fullfile(dataFolder, mapName);


gpsPose = readtable(locationFile);


framePCD = pcread(framePointCloudFileName);


LocationFile = fopen(frameLocationFileName,'r');
frameLocation = cell2mat(textscan(LocationFile,'%f','headerlines',0));

frameLocation(3) = -frameLocation(3);

pctransform = framePCD.Location + frameLocation';

%%
xData = table2array(gpsPose(:, "Var3"));
yData = -table2array(gpsPose(:, "Var2"));
zData = table2array(gpsPose(:, "Var4"));

%%
mapPCD = pcread(mapFile);

%%
pcshow(mapPCD.Location);
hold on
pcshow(pctransform, 'red');
hold on
plot3(xData, yData, zData, 'linewidth', 3);
