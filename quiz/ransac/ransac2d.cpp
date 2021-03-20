/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
 
  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// It will contain the best set of inliners once the processing is done for all iterations.
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--)
	{
		// Set for the inliners of current iteration
		std::unordered_set<int> inliers;
		// Select 2 random points from the cloud :
		// Modulo is to limit the above value the random value can take to the max number of points in the cloud
		// Using a set and method insert helps us to avoid to have twice the same value, since it would not be added
		while (inliers.size() < 2)
			inliers.insert(rand()%(cloud->points.size()));
		// Coordinates of 2 points.
		float x1, x2, y1, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		// Parameters of line equation.
		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		// Iterate through the points of the cloud to calculate their distances to the line
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// To avoid the points already taken as inliers
			if (inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x3, y3;
			x3 = point.x;
			y3 = point.y;

			float d = ((fabs(a*x3+b*y3+c))/(sqrt(a*a+b*b)));
			if (d <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}
*/
// Used for planar RANSAC
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// It will contain the best set of inliners once the processing is done for all iterations.
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--)
	{
		// Set for the inliners of current iteration
		std::unordered_set<int> inliers;
		// Select 3 random points from the cloud :
		// Modulo is to limit the above value the random value can take to the max number of points in the cloud
		// Using a set and method insert helps us to avoid to have twice the same value, since it would not be added
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));
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

		// Parameters of plane equation.
		float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float d = -(a*x1+b*y1+c*z1);

		// Iterate through the points of the cloud to calculate their distances to the line
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// To avoid the points already taken as inliers
			if (inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x4, y4, z4;
			x4 = point.x;
			y4 = point.y;
			z4 = point.z;

			float dist = ((fabs(a*x4+b*y4+c*z4+d))/(sqrt(a*a+b*b+c*c)));
			if (dist <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.2);
	std::unordered_set<int> inliers = RansacPlane(cloud, 1000, 0.18);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
