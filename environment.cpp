/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    // Create beams of lidar with scan function
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = lidar->scan();
    // Set in sumulator the beamrays of lidar
    // lidar->position is defined in lidar class, generated with data passed to create the lidar previously
    //renderRays(viewer, lidar->position, pointcloud);

    // Show the PCD instead of the rays environment
    //renderPointCloud(viewer, pointcloud, "pointCloud");
 
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* myPCL = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pairResult = myPCL->SegmentPlane(pointcloud, 1000, 0.18);

    renderPointCloud(viewer, pairResult.first, "obsCloud", Color(1,1,1));
    renderPointCloud(viewer, pairResult.second, "planeCloud", Color(0,1,0));

    // Clustering code
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = myPCL->Clustering(pairResult.first, 4, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,1,0)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
    {
        std::cout << "cloud size: ";
        myPCL->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        Box box = myPCL->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId%colors.size()]);
        /*BoxQ boxq = myPCL->BoundingBoxQ(cluster);
        renderBox(viewer, boxq, clusterId, colors[clusterId%colors.size()]);*/

        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // Filtering
    // Filtering resolution is 0.45
    // View for X-axis - 15 meters behind ego car and 30 meters in front of it is more than enough for the purpose we want
    // View for Y-axis - 6 meters to the right the ego car and 6.5 to detect all cars and avoid the walls in the detection - not interesting if we want just the road
    // View for Z-axis - 100 meters above the ego car (for higher vehicles and signals) and 2.5 under it, so we do not have false positive obstacle detections under the ground plane
    // I consider the max height of the used ego vehicle to be between 1.5 and 2.5 meters, so it is enough
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.45f , Eigen::Vector4f ( -15., -6, -2.5, 1), Eigen::Vector4f ( 30., 7, 50., 1));
    
    // Segmentation code: divide between road plane and obstacles
    // Separate ground and obstacles
    // 100 iterations are done
    // 0.18 meters has been found as an optimal distanceTolerance. With 0.2 I had slighty worse results and used time was pretty much the same
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.18);
    renderPointCloud(viewer,segmentCloud.first,"obsCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"groundCloud",Color(0,1,0));

    //Clustering
    // ClusterTolerance is 0.6 meters to properly perform clustering with nearest clusters
    // minSize is 6 to count as a cluster the post
    // maxSize is 250 to have always clusters for all detected vehicles, since some cluster have more than 200 points due to previous hyperparameters for other functions
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusteredCloud = pointProcessorI->Clustering(segmentCloud.first, 0.6, 5, 180);
    int clusterIdentifier = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusteredCloud)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterIdentifier),colors[clusterIdentifier]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterIdentifier);
        ++clusterIdentifier;
        // BoxQ is not adding valuable information since we are running in a flat street and all obstacles are equally oriented
        // I used it, but is makes a little slower the viewer and much more confusing to follow with the aye each obstacle
    } 
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // In order to stream all the PCD files, I need a vector to read all the files
    // The ProcessPointClouds is useful to avoid to generate a new one inside cityBlock for each frame
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    // Iterator for the stream of PCD files
    auto iterator = stream.begin();
    // Input PointCloud for the entry data 
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // Load first PCD file
    inputCloudI = pointProcessorI->loadPcd((*iterator).string());
    // Run obstacle detection process
    cityBlock(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped ())
    {
        // Clear viewer to display the information of next frame
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load next PCD file and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*iterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        iterator++;
        // If the stream arrives to the last frame, go back to the first one to restart the cyle of frames
        if(iterator == stream.end())
        {
            iterator = stream.begin();
        }

        viewer->spinOnce ();
    } 
}