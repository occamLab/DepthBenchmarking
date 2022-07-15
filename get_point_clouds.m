% Needs to be in the same folder as the files
lidar_depth = load("lidar_depth.csv");
midas_depth = load('midas_point_cloud.csv');
lidar_point_cloud = pointCloud(lidar_depth);
midas_point_cloud = pointCloud(midas_depth);
%Load JSON data
fname = 'framemetadata.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);
pose = reshape(getfield(val, "pose"), [4,4]);

% Plot Lidar point cloud
pcshow(lidar_point_cloud)
title("Lidar Point Cloud")
xlabel("X");
ylabel("Y");
zlabel("Z");
% Plot Midas point cloud
figure
pcshow(midas_point_cloud)
title("Midas Point Cloud")
colormap(spring)
xlabel("X");
ylabel("Y");
zlabel("Z");
% Plot both point clouds
figure
pcshowpair(lidar_point_cloud, midas_point_cloud)
title("Comparison of Lidar and Midas")
xlabel("X");
ylabel("Y");
zlabel("Z");
legend("\color{white} Lidar", "\color{white} Midas")
%Plot filtered Lidar point cloud
arraysize = size(lidar_depth);
filtered_lidar = pose * [lidar_depth'; ones(1, arraysize(1))];
filtered_lidar = filtered_lidar(1:3, :)';
filtered_lidar = [(filtered_lidar(:,1) - ((max(filtered_lidar(:,1)) + min(filtered_lidar(:,1)))/2)) ...
    (filtered_lidar(:,2) - min(filtered_lidar(:,2))) ...
    (filtered_lidar(:,3) - max(filtered_lidar(:,3)))];
figure
pcshow(pointCloud(filtered_lidar))
title("Filtered Lidar Point Cloud")
colormap(summer)
xlabel("X");
ylabel("Y");
zlabel("Z");