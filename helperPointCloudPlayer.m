dataFolder = fullfile(getenv('DEEPGTAV_EXPORT_DIR'), 'object', filesep);
pointCloudFilePattern = fullfile(dataFolder, 'velodyne_360', '*.pcd');
pointCloudTable = helperReadGTAVDataset(pointCloudFilePattern);

ptCloud = pcread(pointCloudTable{1});
%disp(ptCloud)

xlimits = [-100 100]; % meters
ylimits = [-100 100];
zlimits = [-30 30];

% Create a streaming point cloud display object
lidarPlayer = pcplayer(xlimits, ylimits, zlimits);
xlabel(lidarPlayer.Axes, 'X (m)')
ylabel(lidarPlayer.Axes, 'Y (m)')
zlabel(lidarPlayer.Axes, 'Z (m)')

title(lidarPlayer.Axes, 'GTAV 360 Sensor Data')

skipFrames  = 3;
numFrames   = height(pointCloudTable);

for n = 1 : skipFrames : numFrames
    fileName = pointCloudTable{n};
    ptCloud = pcread(fileName);
    view(lidarPlayer, ptCloud);
    pause(0.01)
end
