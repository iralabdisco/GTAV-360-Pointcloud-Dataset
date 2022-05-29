function helperMapPathVisualizer(runName)
dataFolder = fullfile(getenv('DEEPGTAV_EXPORT_DIR'), runName, filesep);
locationFile = fullfile(dataFolder, 'location.txt');
mapName = strcat(runName,'_complete_map.pcd');
mapFile = fullfile(dataFolder, mapName);

gpsPose = readtable(locationFile);

%%
xData = table2array(gpsPose(:, "Var3"));
yData = -table2array(gpsPose(:, "Var2"));
zData = table2array(gpsPose(:, "Var4"));

%%
mapPCD = pcread(mapFile);

%%
pcshow(mapPCD.Location);
hold on
plot3(xData, yData, zData, 'linewidth', 3);

end