% set paths for trial folder
USER = "HccdFYqmqETaJltQbAe19bnyk2e2";
TRIAL = "54104457-93AF-47AC-8A6A-602C507CA0F9";
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

    end
    % show progress
    disp("Proccessed " + i + "/" + s(1))
end
