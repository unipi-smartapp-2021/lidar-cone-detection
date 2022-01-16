Data = load('T:\pcds\from500_600\out\labels.mat');
DataTruth = Data.gTruth.ROILabelData.pointcloudSequence_500_599.cone;
for i = 1:length(DataTruth.ROILabelData.depth_pcd)
    file = Data.gTruth.DataSource.Name
    cloud = Data.gTruth.ROILabelData.pointcloudSequence_500_599.cone{i}
    if length(cloud(:)) > 0
        dlmwrite(['T:/pcds/from500_600/labels/00' num2str(500+((i-1))) '.txt'], cloud(:,1:6), 'precision','%.2f', 'delimiter',' ');
    end
end