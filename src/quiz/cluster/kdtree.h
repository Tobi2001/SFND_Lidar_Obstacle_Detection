/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;

    Node(std::vector<float> arr, int setId)
      : point(arr), id(setId)
    {}
};

struct KdTree
{
    constexpr static int DimensionsT = 2;
    std::unique_ptr<Node> root;

    KdTree()
    {}

    void insertHelper(std::unique_ptr<Node>& currNode, std::unique_ptr<Node>& newNode, int level)
    {
        if (!currNode)
        {
            currNode = std::move(newNode);
        }
        else if (newNode->point[level] < currNode->point[level])
        {
            insertHelper(currNode->left, newNode, ++level % DimensionsT);
        }
        else
        {
            insertHelper(currNode->right, newNode, ++level % DimensionsT);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        std::unique_ptr<Node> newNode = std::unique_ptr<Node>(new Node(point, id));
        insertHelper(root, newNode, 0);

    }

    float distance(const std::vector<float>& p1, const std::vector<float>& p2)
    {
        float sum = 0.f;
        for (int i = 0; i < p1.size(); ++i)
        {
            float diff = p1[i] - p2[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    void searchHelper(
        std::unique_ptr<Node>& currNode, const std::vector<float>& target, const float distanceTol, const int level,
        std::vector<int>& ids)
    {
        if (currNode)
        {
            bool inBox = true;
            for (int i = 0; i < DimensionsT; ++i)
            {
                if (std::abs(currNode->point[i] - target[i]) > distanceTol)
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
            if (target[level] - distanceTol < currNode->point[level])
            {
                //check left child
                searchHelper(currNode->left, target, distanceTol, nextLevel, ids);
            }
            if (target[level] + distanceTol >= currNode->point[level])
            {
                //check right child
                searchHelper(currNode->right, target, distanceTol, nextLevel, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        assert(target.size() == DimensionsT);
        searchHelper(root, target, distanceTol, 0, ids);
        return ids;
    }


};
