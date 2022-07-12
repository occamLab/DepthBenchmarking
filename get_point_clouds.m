% Needs to be in the same folder as the files
lidar_depth = load("lidar_depth.csv");
midas_depth = load('midas_point_cloud.csv');
lidar_point_cloud = pointCloud(lidar_depth);
midas_point_cloud = pointCloud(midas_depth);

% Plot lidar point cloud
pcshow(lidar_point_cloud)
title("Lidar Point Cloud")
xlabel('X');
ylabel('Y');
zlabel('Z');
% Plot Midas point cloud
figure
pcshow(midas_point_cloud)
title("Midas Point Cloud")
colormap(spring)
xlabel('X');
ylabel('Y');
zlabel('Z');
% Plot both point clouds
figure
pcshowpair(lidar_point_cloud, midas_point_cloud)
title("Comparison of Lidar and Midas")
xlabel('X');
ylabel('Y');
zlabel('Z');
legend("\color{white} Lidar", "\color{white} Midas")