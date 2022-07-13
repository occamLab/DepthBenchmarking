% set paths for trial folder
USER = "HccdFYqmqETaJltQbAe19bnyk2e2";
TRIAL = "3F2B13D6-443F-4B5B-9CCD-2DB0AB887296";
TRIAL_PATH = "/Users/angrocki/Desktop/DepthData/depth_benchmarking/" + USER + "/" + TRIAL;

% get all of the image folders within the trial path
subfolders = dir(TRIAL_PATH);
s = size(subfolders);

for i=1:s
    name = TRIAL_PATH + "/" + subfolders(i).name;
    if isfile(name + "/midas_point_cloud.csv") && isfile(name + "/lidar_depth.csv") 
        % load data and make point clouds
        lidar_depth = load(name + "/lidar_depth.csv");
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




