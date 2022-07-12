USER = "HccdFYqmqETaJltQbAe19bnyk2e2";
TRIAL = "3F2B13D6-443F-4B5B-9CCD-2DB0AB887296";
TRIAL_PATH = "/Users/angrocki/Desktop/DepthData/depth_benchmarking/" + USER + "/" + TRIAL;

subfolders = dir(TRIAL_PATH);

for i=1:size(subfolders)
    name = "/" + subfolders(i).name;
    if isfile(TRIAL_PATH + name + "/midas_point_cloud.csv") && isfile(TRIAL_PATH + name + "/lidar_depth.csv") 
        lidar_depth = load("lidar_depth.csv");
        midas_depth = load('midas_point_cloud.csv');
        lidar_point_cloud = pointCloud(lidar_depth);
        midas_point_cloud = pointCloud(midas_depth);
        
        f = figure();
        pcshowpair(lidar_point_cloud, midas_point_cloud)
        title("Comparison of Lidar and Midas")
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        legend("\color{white} Lidar", "\color{white} Midas")
        % save figure to image folder and data folder
    end
end


