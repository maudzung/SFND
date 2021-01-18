// PCL lib Functions for processing point clouds 
#include <unordered_set>
// #include <algorithm>
// #include <random>

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered {new pcl::PointCloud<PointT>};
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region {new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indicies;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.f, 1.f));
    roof.setMax(Eigen::Vector4f(2.6f, 1.7f, -0.4f, 1.f));
    roof.setInputCloud(cloud_region);
    roof.filter(indicies);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int p_idx : indicies) {
        inliers->indices.push_back(p_idx);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obst_cloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT>());

    for (auto idx : inliers->indices) {
        plane_cloud->points.push_back(cloud->points[idx]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obst_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	float A, B, C, D, x1, y1, z1, x2, y2, z2, x3, y3, z3;
	std::unordered_set<int> inliers;
	for (int iter = 0; iter < maxIterations; iter++) {
	// Randomly sample subset and fit line
		while (inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size());
		}

        // std::sample(cloud->points.begin(), cloud->points.end(), std::back_inserter(inliers), 5, std::mt19937{std::random_device{}()});

		auto it = inliers.begin();
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
		z1 = cloud->points[*it].z;

		it++;
		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;
		z2 = cloud->points[*it].z;

		it++;
		x3 = cloud->points[*it].x;
		y3 = cloud->points[*it].y;
		z3 = cloud->points[*it].z;

		A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		D = -(A * x1 + B * y1 + C * z1);

		for (int p_idx = 0; p_idx < cloud->points.size(); p_idx++) {
			if (inliers.count(p_idx) > 0) continue;
            auto p_x = cloud->points[p_idx].x;
            auto p_y = cloud->points[p_idx].y;
            auto p_z = cloud->points[p_idx].z;
			float dist = std::abs(A * p_x + B * p_y + C * p_z + D) / std::sqrt(A * A + B * B + C * C);
			if (dist < distanceThreshold) inliers.insert(p_idx);
		}
		if (inliers.size() > inliersResult.size()) inliersResult = inliers;
		inliers.clear();
	}

	if (inliersResult.empty()) {
        std::cerr << "No point in inliers" << std::endl;
	}

	typename pcl::PointIndices::Ptr cloudInliers (new pcl::PointIndices());
	for (int point : inliersResult) {
		cloudInliers->indices.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(cloudInliers, cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::find_nearest(int p_idx, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[p_idx] = true;
	cluster.push_back(p_idx);
	for (int idx : tree->search(points[p_idx], distanceTol)) {
		if (!processed[idx]) {
			find_nearest(idx, points, cluster, processed, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed (points.size(), false);
	for (int p_idx = 0; p_idx < points.size(); p_idx++) {
		if (!processed[p_idx]) {
			std::vector<int> cluster;
			find_nearest(p_idx, points, cluster, processed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}

	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<std::vector<float>> points;
	KdTree* tree = new KdTree;
	for (int p_idx = 0; p_idx < cloud->points.size(); p_idx++) {
		auto point = cloud->points[p_idx];
		tree->insert({point.x, point.y, point.z}, p_idx);
		points.push_back({point.x, point.y, point.z});
	}
	std::vector<std::vector<int>> indices = euclideanCluster(points, tree, clusterTolerance);

	for (std::vector<int> v : indices) {
		typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
		if (v.size() >= minSize && v.size() <= maxSize) {
			for (int p_idx : v) {
				cluster->points.push_back(cloud->points[p_idx]);
			}
			clusters.push_back(cluster);
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}