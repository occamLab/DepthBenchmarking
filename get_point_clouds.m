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

% Plot filtered Lidar point cloud
arraysize = size(lidar_depth);

% Convert LiDAR point cloud to global coordinate frame
filtered_lidar = pose * [lidar_depth'; ones(1, arraysize(1))];
% Adjust yaw
theta = atan2(pose(1, 3), pose(3, 3));
filtered_lidar = (axang2rotm([0 1 0 -theta]) * filtered_lidar(1:3, :))';
% Shift point cloud so that zero is in the right place for the z axis
filtered_lidar = [filtered_lidar(:,1:2) (filtered_lidar(:,3) - max(filtered_lidar(:,3)))];
% Filter z values
filtered_lidar = filtered_lidar(filtered_lidar(:, 3) >= -4, :);
% Shift zero for x and y axes
filtered_lidar = [(filtered_lidar(:,1) - ((max(filtered_lidar(:,1)) + min(filtered_lidar(:,1)))/2)) ...
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

% Simple object dectection 
z_min = 0;
z_step = -0.5;
z_max = z_min + z_step;
path_clear = true;
while path_clear && z_max > -4
    current_range = filtered_lidar(filtered_lidar(:,3) >= z_max, :);
    current_range = current_range(current_range(:,3) <= z_min, :);
    s = size(current_range);
    points = s(1);
    if points > 500
        path_clear = false;
    else
        z_min = z_min + z_step;
        z_max = z_max + z_step;
    end 
end
fprintf("Object detected: %g - %g\n", abs(z_min), abs(z_max));
% Histogram object dectection 
figure
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
% Remove values close in range 
tol=0.5;
goodcols=find([1 any(abs(diff(right,1,2))>=tol,1)]);
objects = right(:,goodcols);
print(objects)

% Plot filtered MiDaS point cloud
arraysize = size(midas_depth);
filtered_midas = pose * [midas_depth'; ones(1, arraysize(1))];
filtered_midas = (axang2rotm([0 1 0 -theta]) * filtered_midas(1:3, :))';
filtered_midas = [filtered_midas(:, 1:2) (filtered_midas(:,3) - max(filtered_midas(:,3)))];
filtered_midas = filtered_midas(filtered_midas(:, 3) >= -4, :);
filtered_midas = [(filtered_midas(:,1) - ((max(filtered_midas(:,1)) + min(filtered_midas(:,1)))/2)) ...
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
