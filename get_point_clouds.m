close all

% Needs to be in the same folder as the files
% Update CSV and JSON files before running script
lidar_data = load("lidar_depth.csv");
lidar_depth = lidar_data(:, 1:3);
lidar_confidence = lidar_data(:, 4);
midas_depth = load("midas_point_cloud.csv");
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
figure
pcshow(lidar_point_cloud)
title("Lidar Point Cloud")
xlabel("X");
ylabel("Y");
zlabel("Z");

% Plot high confidence Lidar point cloud
lidar_depth = lidar_depth(lidar_confidence == 2, :);
figure
pcshow(pointCloud(lidar_depth))
title("High Confidence Lidar Point Cloud")
colormap(winter)
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

% Convert LiDAR point cloud to global coordinate frame
filtered_lidar = pose(1:3, 1:3) * lidar_depth';
% Adjust yaw
theta = atan2(pose(1, 3), pose(3, 3));
filtered_lidar = (axang2rotm([0 1 0 -theta]) * filtered_lidar)';
% Filter z values
filtered_lidar = filtered_lidar(filtered_lidar(:, 3) >= -4, :);
% Shift zero for y axis
filtered_lidar = [(filtered_lidar(:,1)) ...
    (filtered_lidar(:,2) - min(filtered_lidar(:,2))) filtered_lidar(:,3)];
% Filter x values
filtered_lidar = filtered_lidar(abs(filtered_lidar(:, 1)) <= 0.5, :);
% Filter y values
filtered_lidar = filtered_lidar(filtered_lidar(:, 2) > 0.25, :);

figure
pcshow(pointCloud(filtered_lidar))
title("Filtered Lidar Point Cloud")
colormap(summer)
xlabel("X");
ylabel("Y");
zlabel("Z");

% Histogram Visualization
w = figure;
histogram(filtered_lidar(:,3))
title("Filtered Lidar Depths")
xlabel("Depth (m)")
ylabel("Number of Points")
h = histogram(filtered_lidar(:,3));
% Retrieve some properties from the histogram
V = h.Values;
E = h.BinEdges;
% Use islocalmax
L = islocalmax(V);
% Find the centers of the bins that islocalmax identified as peaks
left = E(L);
right = E([false L]);
center = (left + right)/2;
% Plot markers on those bins
hold on
plot(center, V(L), 'o')
title("Filter Lidar Depths with Local Maximums")
xlabel("Depth (m)")
ylabel("Number of Points")
legend("points", "local max",'Location', "best")

% Histogram object dectection 
z_value = filtered_lidar(:,3);
minZVaule = 0.0;
maxZVaule = -4.0;
step_size = -0.1;
threshhold = 100;
binLeftEdge = minZVaule:step_size:maxZVaule;
hist = [];
for binEdge = binLeftEdge
    leftEdge = binEdge;
    rightEdge = binEdge + step_size;
    filteredZVaules = z_value(z_value <= leftEdge & z_value > rightEdge);
    numberInBin = numel(filteredZVaules);
    hist = [hist,numberInBin];
end
localMaxes = [];
for i = 2:1:numel(binLeftEdge)-1 
    leftCount = hist(i-1);
    centerCount = hist(i);
    rightCount = hist(i+1);
    if centerCount > leftCount && centerCount > rightCount && centerCount > threshhold
        localMaxes = [localMaxes,-binLeftEdge(i)];
    end 
end 
disp(localMaxes)

% Plot filtered MiDaS point cloud
filtered_midas = pose(1:3, 1:3) * midas_depth';
filtered_midas = (axang2rotm([0 1 0 -theta]) * filtered_midas)';
filtered_midas = filtered_midas(filtered_midas(:, 3) >= -4, :);
filtered_midas = [(filtered_midas(:,1)) ...
    (filtered_midas(:,2) - min(filtered_midas(:,2))) filtered_midas(:,3)];
filtered_midas = filtered_midas(abs(filtered_midas(:, 1)) <= 0.5, :);
filtered_midas = filtered_midas(filtered_midas(:, 2) > 0.25, :);
figure
pcshow(pointCloud(filtered_midas));
title("Filtered Midas Point Cloud")
colormap(autumn)
xlabel("X");
ylabel("Y");
zlabel("Z");
