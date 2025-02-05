// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "quiz/cluster/kdtree.h"

template<typename PointT>
class ProcessPointClouds
{
public:
    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(
        pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    // get random value in range [lowerBound, upperBound]
    size_t randomInRange(const size_t lowerBound, const size_t upperBound);

    // ransac model inliers
    template<typename ModelT>
    std::unordered_set<int> modelInliers(
        const ModelT& model, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol);

    // ransac with plane model
    std::unordered_set<int> RansacPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    void proximity(
        int pointIndex, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT, 3>& tree,
        float distanceTol, std::vector<int>& cluster, std::unordered_set<int>& processedPoints);

    std::vector<std::vector<int>> euclideanCluster(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        KdTree<PointT, 3>& tree,
        float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};
#endif /* PROCESSPOINTCLOUDS_H_ */
