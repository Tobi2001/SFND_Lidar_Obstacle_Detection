/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
template<class PointT>
struct Node
{
    PointT point;
    int id;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;

    Node(PointT arr, int setId)
      : point(arr), id(setId)
    {}
};

template<class PointT, int DimensionsT = 3>
struct KdTree
{
    std::unique_ptr<Node<PointT>> root;

    KdTree()
    {}

    float pointComponent(const PointT& point, int componentIndex);

    void insertHelper(std::unique_ptr<Node<PointT>>& currNode, std::unique_ptr<Node<PointT>>& newNode, int level)
    {
        if (!currNode)
        {
            currNode = std::move(newNode);
        }
        else if (pointComponent(newNode->point, level) < pointComponent(currNode->point, level))
        {
            insertHelper(currNode->left, newNode, ++level % DimensionsT);
        }
        else
        {
            insertHelper(currNode->right, newNode, ++level % DimensionsT);
        }
    }

    void insert(PointT point, int id)
    {
        std::unique_ptr<Node<PointT>> newNode = std::unique_ptr<Node<PointT>>(new Node<PointT>(point, id));
        insertHelper(root, newNode, 0);

    }

    float distance(const PointT& p1, const PointT& p2)
    {
        float sum = 0.f;
        for (int i = 0; i < DimensionsT; ++i)
        {
            float diff = pointComponent(p1, i) - pointComponent(p2, i);
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    void searchHelper(
        std::unique_ptr<Node<PointT>>& currNode, const PointT& target, const float distanceTol, const int level,
        std::vector<int>& ids)
    {
        if (currNode)
        {
            bool inBox = true;
            for (int i = 0; i < DimensionsT; ++i)
            {
                if (std::abs(pointComponent(currNode->point, i) - pointComponent(target, i)) > distanceTol)
                {
                    inBox = false;
                    break;
                }
            }
            if (inBox && distance(currNode->point, target) <= distanceTol)
            {
                ids.push_back(currNode->id);
            }

            int nextLevel = (level + 1) % DimensionsT;
            if (pointComponent(target, level) - distanceTol < pointComponent(currNode->point, level))
            {
                //check left child
                searchHelper(currNode->left, target, distanceTol, nextLevel, ids);
            }
            if (pointComponent(target, level) + distanceTol >= pointComponent(currNode->point, level))
            {
                //check right child
                searchHelper(currNode->right, target, distanceTol, nextLevel, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, target, distanceTol, 0, ids);
        return ids;
    }
};


template<>
inline float KdTree<std::vector<float>, 2>::pointComponent(const std::vector<float>& point, int componentIndex)
{
    return point.at(componentIndex);
}

template<>
inline float KdTree<pcl::PointXYZ, 3>::pointComponent(const pcl::PointXYZ& point, int componentIndex)
{
    switch (componentIndex)
    {
      case 0: return point.x;
      case 1: return point.y;
      case 2: return point.z;
      default:
          throw std::runtime_error("Invalid component index when accessing 3D-point");
    }
}

template<>
inline float KdTree<pcl::PointXYZI, 3>::pointComponent(const pcl::PointXYZI& point, int componentIndex)
{
    switch (componentIndex)
    {
      case 0: return point.x;
      case 1: return point.y;
      case 2: return point.z;
      default:
          throw std::runtime_error("Invalid component index when accessing 3D-point");
    }
}
