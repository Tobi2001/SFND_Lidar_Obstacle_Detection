// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "quiz/ransac/plane_impl.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudRemainders(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloudFiltered);

    extract.setNegative(true);
    extract.filter(*cloudRemainders);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudFiltered,
        cloudRemainders);
    return segResult;
}

template<typename PointT>
size_t ProcessPointClouds<PointT>::randomInRange(const size_t lowerBound, const size_t upperBound)
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

template<typename PointT>
template<typename ModelT>
std::unordered_set<int> ProcessPointClouds<PointT>::modelInliers(
    const ModelT& model,
    typename pcl::PointCloud<PointT>::Ptr cloud,
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

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int maxIterations, float distanceTol)
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
                Plane<PointT> model(cloud->points[indexP1], cloud->points[indexP2], cloud->points[indexP3]);
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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
/*
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
*/

    std::unordered_set<int> inlierIndices = RansacPlane(cloud, maxIterations, distanceThreshold);
    if (inlierIndices.empty())
    {
        std::cout << "Could not estimate plane parameters for given cloud" << std::endl;
    }
    for (const auto& index : inlierIndices)
    {
        inliers->indices.push_back(index);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(
        inliers, cloud);
    return segResult;
}

template<class PointT>
void ProcessPointClouds<PointT>::proximity(
    int pointIndex, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT, 3>* tree,
    float distanceTol, std::vector<int>& cluster, std::unordered_set<int>& processedPoints)
{
    processedPoints.insert(pointIndex);
    cluster.push_back(pointIndex);
    std::vector<int> nearbyPoints = tree->search(cloud->points[pointIndex], distanceTol);
    for (int nearbyPoint : nearbyPoints)
    {
        if (processedPoints.find(nearbyPoint) == processedPoints.end())
        {
            proximity(nearbyPoint, cloud, tree, distanceTol, cluster, processedPoints);
        }
    }
}

template<class PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT, 3>* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processedPoints;
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (processedPoints.find(i) == processedPoints.end())
        {
            std::vector<int> cluster;
            proximity(i, cloud, tree, distanceTol, cluster, processedPoints);
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//    tree->setInputCloud(cloud);

//    std::vector<pcl::PointIndices> clusterIndices;
//    pcl::EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance(clusterTolerance);
//    ec.setMinClusterSize(minSize);
//    ec.setMaxClusterSize(maxSize);
//    ec.setSearchMethod(tree);
//    ec.setInputCloud(cloud);
//    ec.extract(clusterIndices);

//    for (const auto& cluster : clusterIndices)
//    {
//        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
//        for (std::vector<int>::const_iterator pit = cluster.indices.begin(); pit != cluster.indices.end(); ++pit)
//        {
//            cloudCluster->points.push_back(cloud->points[*pit]);
//        }
//        cloudCluster->width = cloudCluster->points.size();
//        cloudCluster->height = 1;
//        cloudCluster->is_dense = true;
//        clusters.push_back(cloudCluster);
//    }

    KdTree<PointT, 3>* tree = new KdTree<PointT, 3>;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        tree->insert(cloud->points[i], i);
    }

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(cloud, tree, clusterTolerance);
    for (const auto& cluster : clusterIndices)
    {
        if (cluster.size() < minSize || cluster.size() > maxSize)
        {
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (auto index : cluster)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() <<
        " clusters" << std::endl;

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
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
        boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
