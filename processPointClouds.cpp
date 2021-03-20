// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());

    // Voxel grid. Create volume pixel of side filterRes
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filteredCloud);

    // Region of Interest. We keep only the zone that is interesting for us : the road environment
    pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(filteredCloud);
    typename pcl::PointCloud<PointT>::Ptr filteredCloud2 (new pcl::PointCloud<PointT>());
    boxFilter.filter(*filteredCloud2);

    // The next code is used to remove the detected points of the ego vehicle roof
    // Parameters for setMin and setMax are the sames that used by Udacity since I did not reached any better result
    // by changing the parameters
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1., 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(filteredCloud2);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (const auto element: indices) 
    {
        inliers->indices.push_back(element);
    }
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filteredCloud2);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion; 

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    // Iterate through the inliners
    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers from the original cloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);

    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    
    // TODO:: Fill in this function to find inliers for the cloud.
    while (maxIterations--)
	{
		// Set for the inliners of current iteration
		std::unordered_set<int> inliers;
		// Select 3 random points from the cloud :
		// Module operation is to limit the above value the random value can take to the max number of points in the cloud
		// Using a set and method insert helps us to avoid to have twice the same value, since it would not be added
		while (inliers.size() < 3)
        {
            inliers.insert(rand()%(cloud->points.size()));
        }
		// Coordinates of 3 points.
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Parameters of plane equation
		float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float d = -(a*x1+b*y1+c*z1);

		// Iterate through the points of the cloud to calculate their distances to the fitted line (plane in 3D case)
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// To avoid the points already taken as inliers
			if (inliers.count(index)>0)
			{
                continue;
            }

			float x4, y4, z4;
			x4 = cloud->points[index].x;
			y4 = cloud->points[index].y;
			z4 = cloud->points[index].z;

			float dist = ((fabs(a*x4+b*y4+c*z4+d))/(sqrt(a*a+b*b+c*c)));
			if (dist <= distanceThreshold)
            {
                inliers.insert(index);
            }
		}

		if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
	}

    // To store the inliers point indices format
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    for (const auto& element: inliersResult)
    {
        inliers2->indices.push_back(element);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmenResult = SeparateClouds(inliers2,cloud);
    return segmenResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int ind, std::vector<int> &cluster, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree,  float distanceTol, std::vector<bool> &processed){
		processed[ind]= true;
		cluster.push_back(ind);
        std::vector<float> pt; 
        pt.push_back(cloud->points[ind].x);
        pt.push_back(cloud->points[ind].y);
        pt.push_back(cloud->points[ind].z);
		std::vector<int> nearby_points= tree->search(pt,distanceTol);
		for (auto index: nearby_points){
			if(!processed[index])
            {
				proximity(index, cluster, cloud, tree, distanceTol, processed);
			}
		}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersVector;

	std::vector<std::vector<int>> clusters_indices;
    KdTree* tree = new KdTree;
	// Vector to allocate a single cluster
	std::vector<bool> processedPoints(cloud->points.size());
	std::fill (processedPoints.begin(),processedPoints.end(),false);
	for(int i=0; i<cloud->points.size(); i++)
    {
		// If a point is already processed, nothing to be done for it
		if(processedPoints[i])
		{
            continue;
        }
		// Vector to allocate the indices of processed points
		std::vector<float> point{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point,i);
	}

    for(int i=0; i<cloud->points.size(); i++)
    {
		if(processedPoints[i])
        {
            continue;
        }
		std::vector<int> cluster;
		proximity(i, cluster, cloud, tree, clusterTolerance, processedPoints);
        if((cluster.size() >= minSize) &&  (cluster.size() <= maxSize))
        { 
            clusters_indices.push_back(cluster);
        }
	}

    for(const auto& element: clusters_indices)
    {    
        typename pcl::PointCloud<PointT>::Ptr cluster(new typename pcl::PointCloud<PointT>);
        
        for( int i : element)
        {
            cluster->points.push_back(cloud->points[i]);
        }
        // Define some parameters of the cluster and add it to the vector of clusters
        cluster->width= cluster->points.size();
        cluster->height= 1;
        cluster->is_dense = true;
        clustersVector.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " milliseconds and found " << clustersVector.size() << " clusters" << std::endl;

    return clustersVector;
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    BoxQ box;
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    // Final transform
    box.bboxQuaternion =  eigenVectorsPCA; //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    box.cube_length = std::abs(maxPoint.x-minPoint.x);
    box.cube_width = std::abs(maxPoint.y-minPoint.y);
    box.cube_height = std::abs(maxPoint.z-minPoint.z);
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