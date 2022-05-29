frame = 340;
runName = 'canyon_sequence';
dataFolder = fullfile(getenv('DEEPGTAV_EXPORT_DIR'), runName, filesep);
framePointCloudFileName = fullfile(dataFolder, 'velodyne_360', sprintf('%06d.pcd', frame));

framePCD = pcread(framePointCloudFileName);

%%
pcshow(framePCD.Location);
