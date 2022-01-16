Data = load('C:\dev\sensory-cone-detection\src\lidar\interpolation_data\labels.mat');


writetimetable(Data.gTruth.ROILabelData.depth_pcd,'C:\dev\sensory-cone-detection\src\lidar\interpolation_data\depth_labels.csv')
writetimetable(Data.gTruth.ROILabelData.pcd,'C:\dev\sensory-cone-detection\src\lidar\interpolation_data\pcd_labels.csv')
