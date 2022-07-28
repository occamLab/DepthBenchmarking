% set paths for trial folder
USER = "HccdFYqmqETaJltQbAe19bnyk2e2";
TRIAL = "F231CF41-FBCE-4C5B-8E88-BD05FEDB7591";
TRIAL_PATH = "/Users/occamlab/Documents/DepthData/depth_benchmarking/" + USER + "/" + TRIAL;

% get all of the image folders within the trial path
subfolders = dir(TRIAL_PATH);
s = size(subfolders);

for i=1:s
    name = TRIAL_PATH + "/" + subfolders(i).name;
    if isfile(name + "/midas_point_cloud.csv") && isfile(name + "/lidar_depth.csv") 
        % load data and make point clouds
        lidar_data = load(name + "/lidar_depth.csv");
        lidar_depth = lidar_data(:, 1:3);
        midas_depth = load(name + "/midas_point_cloud.csv");
        lidar_point_cloud = pointCloud(lidar_depth);
        midas_point_cloud = pointCloud(midas_depth);
        fname = name + '/framemetadata.json';
        fid = fopen(fname);
        raw = fread(fid,inf);
        str = char(raw');
        fclose(fid);
        val = jsondecode(str);
        pose = reshape(getfield(val, "pose"), [4,4]);
        % create a figure with two point clouds
        f = figure();
        set(gcf, "InvertHardCopy", "off")
        pcshowpair(lidar_point_cloud, midas_point_cloud)
        title("Comparison of Lidar and Midas")
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        legend("\color{white} Lidar", "\color{white} Midas")

        % save figure
        saveas(gcf, name + "/point_clouds_" + subfolders(i).name + ".png");
        saveas(gcf, TRIAL_PATH + "/data/point_clouds_" + subfolders(i).name + ".png");
        close(f)
        %filter data
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
        for j = 2:1:numel(binLeftEdge)-1 
            leftCount = hist(j-1);
            centerCount = hist(j);
            rightCount = hist(j+1);
            if centerCount > leftCount && centerCount > rightCount && centerCount > threshhold
                localMaxes = [localMaxes,-binLeftEdge(j)];
            end 
        end 
        if numel(localMaxes) > 5
            maxSize = int2str(numel(localMaxes));
            txt = maxSize + ":" + num2str(localMaxes(1:5));
        else
            txt = num2str(localMaxes);
        end
        subtitle(txt)
        saveas(gcf, name + "/histogram_" + subfolders(i).name + ".png");
        saveas(gcf, TRIAL_PATH + "/data/histogram_" + subfolders(i).name + ".png");
        close(w)
    end
    % show progress
    disp("Proccessed " + i + "/" + s(1))
end

