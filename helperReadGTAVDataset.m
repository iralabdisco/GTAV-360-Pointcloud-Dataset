function datasetTable = helperReadGTAVDataset(pointCloudFilePattern)

fileDS = fileDatastore(pointCloudFilePattern, 'ReadFcn', @pcread);
pointCloudFiles = fileDS.Files;

datasetTable = pointCloudFiles;

end