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

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
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

    std::unique_ptr<Lidar> lidarSensor = std::unique_ptr<Lidar>(new Lidar(cars, 0.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr scannedCloud = lidarSensor->scan();
    //renderRays(viewer, lidarSensor->position, scannedCloud);
    //renderPointCloud(viewer, scannedCloud, "lidarPoints");

    ProcessPointClouds<pcl::PointXYZ> pcProcessor;
    auto planeFilterPair = pcProcessor.SegmentPlane(scannedCloud, 100, 0.2);
    renderPointCloud(viewer, planeFilterPair.first, "PlanePoints", Color(0, 1, 0));
//    renderPointCloud(viewer, planeFilterPair.second, "ObstaclePoints", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pcProcessor.Clustering(planeFilterPair.second, 1.0,
            3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 0, 1), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pcProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pcProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> pcProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pcProcessor.loadPcd(
        "../src/sensors/data/pcd/data_1/0000000000.pcd");

    auto filteredCloud = pcProcessor.FilterCloud(inputCloud, 0.2f, Eigen::Vector4f(-10.f, -5.f, -3.f, 1.f),
            Eigen::Vector4f(15.f, 8.f, 1.f, 1.f));

    renderPointCloud(viewer, filteredCloud, "inputCloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
      case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
      case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
      case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
      case FPS: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
    {
        viewer->addCoordinateSystem(1.0);
    }
}


int main()
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
