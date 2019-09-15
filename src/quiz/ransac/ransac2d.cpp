/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include "line.h"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++)
    {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--)
    {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
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
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

size_t randomInRange(const size_t lowerBound, const size_t upperBound)
{
    if (lowerBound < upperBound)
    {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<size_t> dist(lowerBound, upperBound);
        return dist(generator);
    }
    return lowerBound;
}

template<class ModelT>
std::unordered_set<int> modelInliers(
    const ModelT& model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float distanceTol)
{
    std::unordered_set<int> inliers;
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (model.distanceToPoint(cloud->points[i]) <= distanceTol)
        {
            inliers.insert(i);
        }
    }
    return inliers;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    using namespace udacity_ransac;
    std::unordered_set<int> inliersResult;

    const size_t cloudSize = cloud->size();
    if (cloudSize >= 2)
    {
        for (int i = 0; i < maxIterations; ++i)
        {
            size_t indexP1 = randomInRange(0, cloudSize - 1);
            size_t indexP2 = randomInRange(0, cloudSize - 1);
            while (indexP1 == indexP2)
            {
                indexP2 = randomInRange(0, cloudSize - 1);
            }
            try
            {
                Line3D model(cloud->points[indexP1], cloud->points[indexP2]);
                auto inliersTmp = modelInliers(model, cloud, distanceTol);
                if (inliersTmp.size() > inliersResult.size())
                {
                    inliersResult = inliersTmp;
                }
            }
            catch (std::exception&)
            {
                continue;
            }
        }
    }

    return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    using namespace udacity_ransac;
    std::unordered_set<int> inliersResult;

    const size_t cloudSize = cloud->size();
    if (cloudSize >= 3)
    {
        for (int i = 0; i < maxIterations; ++i)
        {
            size_t indexP1 = randomInRange(0, cloudSize - 1);
            size_t indexP2 = randomInRange(0, cloudSize - 1);
            size_t indexP3 = randomInRange(0, cloudSize - 1);
            while (indexP2 == indexP1)
            {
                indexP2 = randomInRange(0, cloudSize - 1);
            }
            while (indexP3 == indexP1 || indexP3 == indexP2)
            {
                indexP3 = randomInRange(0, cloudSize - 1);
            }
            try
            {
                Plane<pcl::PointXYZ> model(cloud->points[indexP1], cloud->points[indexP2], cloud->points[indexP3]);
                auto inliersTmp = modelInliers(model, cloud, distanceTol);
                if (inliersTmp.size() > inliersResult.size())
                {
                    inliersResult = inliersTmp;
                }
            }
            catch (std::exception&)
            {
                continue;
            }
        }
    }

    return inliersResult;
}

int main()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    //std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
        {
            cloudInliers->points.push_back(point);
        }
        else
        {
            cloudOutliers->points.push_back(point);
        }
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size())
    {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    }
    else
    {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

}
