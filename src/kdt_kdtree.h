#ifndef KDT_KDTREE_H
#define KDT_KDTREE_H

#include <kdt_limitedpriorityqueue.h>

#include <vector>
#include <stack>
#include <cmath>

namespace kdt {

typedef std::size_t Index; // Used for referncing nodes in a tree
static const Index None = std::numeric_limits<Index>::max();
static const double MaxDistance = std::numeric_limits<double>::max();

template <typename PointType> struct KdTreeNode {
    Index leftIndex;
    Index rightIndex;
    PointType point;

    // Create a node with point specified by the 'p' and not children
    KdTreeNode(const PointType& p) 
        : leftIndex(None)
        , rightIndex(None)
        , point(p) {}
};

template <typename PointType> class KdTree {
public:
    // Create an empty kd-tree with the number of dimensions specified by the
    // 'numDimensions' parameter
    KdTree(unsigned int numDimensions) : d_numDimensions(numDimensions) {}
    ~KdTree() {}

    // Add a point specified by the 'point' to the existing kd-tree
    void AddPoint(const PointType& point);

    // Return the vector that contains indexes of 'numPoints' nearest points to
    // the point specified by the 'point' sorted by distance. If the tree
    // contains less than 'numPoints' points, return indexes of all points of
    // the tree. If the tree is empty, return an empty vector.
    std::vector<Index> FindNearestPoints(const PointType& point,
                                         unsigned int numPoints);

    std::vector<Index> FindNearestPointsLinear(const PointType& point,
                                               unsigned int numPoints);

private:
    typedef KdTreeNode<PointType> Node;
    typedef unsigned int Level;

private:
    double DistanceSq(const PointType& a, const PointType& b) {
        double distanceSq = 0.0;
        for (unsigned i = 0; i < a.size(); i++) {
            double diff = a[i] - b[i];
            distanceSq += diff * diff;
        }
        return distanceSq;
    }

private:
    std::vector<Node> d_nodes;
    Index d_root = None;
    unsigned int d_numDimensions;
};


template<typename PointType>
inline void KdTree<PointType>::AddPoint(const PointType & point)
{
    // TODO: check dimensions

    Index newNodeIndex = d_nodes.size();
    d_nodes.push_back(Node(point));

    if (None == d_root) {
        // It's the first node of the tree
        d_root = newNodeIndex;
        return;
    }

    Index curNodeIndex = d_root;
    unsigned int level = 0;
    while (true) {
        unsigned int dimension = level % d_numDimensions;
        Node& curNode = d_nodes[curNodeIndex];
        if (point[dimension] < curNode.point[dimension]) {
            // left
            if (None == curNode.leftIndex) {
                curNode.leftIndex = newNodeIndex;
                break;
            } else {
                curNodeIndex = curNode.leftIndex;
            }
        } else {
            // right
            if (None == curNode.rightIndex) {
                curNode.rightIndex = newNodeIndex;
                break;
            } else {
                curNodeIndex = curNode.rightIndex;
            }
        }
        level++;
    }
}

template<typename PointType>
inline std::vector<Index> KdTree<PointType>::FindNearestPoints(
                                                       const PointType & point,
                                                       unsigned int numPoints)
{
    LimitedDistanceQueue<Index> limitedQueue(numPoints);

    struct StackFrame {
        Level level;
        Index subtreeRootIndex;
        double minSubtreeDistance;
    };
    std::stack<StackFrame> stack;
    
    StackFrame initialFrame;
    initialFrame.level = 0;
    initialFrame.subtreeRootIndex = d_root;
    initialFrame.minSubtreeDistance = 0;
    stack.push(initialFrame);

    int visited = 0;
    while (!stack.empty()) {
        StackFrame curFrame = stack.top();
        stack.pop();
        
        if (None == curFrame.subtreeRootIndex) {
            continue;
        }

        if (curFrame.minSubtreeDistance > limitedQueue.MaxDistance()
            && limitedQueue.IsFull()) {
            // Current subtree cannot have points closer than the farthest
            // already found point and there is enough points found already.
            continue;
        }

        Node& curNode = d_nodes[curFrame.subtreeRootIndex];
        limitedQueue.Push(curFrame.subtreeRootIndex, 
                          DistanceSq(point, curNode.point));

        unsigned int level = curFrame.level;
        unsigned int dimension = level % d_numDimensions;

        StackFrame newFrame;
        newFrame.level = level + 1;
        double distanceToBorder 
            = abs(point[dimension] - curNode.point[dimension]);
        if (point[dimension] < curNode.point[dimension]) {
            // left
            newFrame.subtreeRootIndex = curNode.leftIndex;
            newFrame.minSubtreeDistance = 0;
            stack.push(newFrame);

            newFrame.subtreeRootIndex = curNode.rightIndex;
            newFrame.minSubtreeDistance = distanceToBorder * distanceToBorder;
            stack.push(newFrame);
        } else {
            // right
            newFrame.subtreeRootIndex = curNode.rightIndex;
            newFrame.minSubtreeDistance = 0;
            stack.push(newFrame);

            newFrame.subtreeRootIndex = curNode.leftIndex;
            newFrame.minSubtreeDistance = distanceToBorder * distanceToBorder;
            stack.push(newFrame);
        }
        visited++;
    }
    std::cout << "visited: " << visited << "\n";

    return limitedQueue.Items();
}

template<typename PointType>
inline std::vector<Index> KdTree<PointType>::FindNearestPointsLinear(
                                                       const PointType & point,
                                                       unsigned int numPoints)
{
    // Linear search for testing purpose
    LimitedDistanceQueue<Index> limitedQueue(numPoints);
    for (Index i = 0; i < d_nodes.size(); i++) {
        limitedQueue.Push(i, DistanceSq(point, d_nodes[i].point));
    }
    return limitedQueue.Items();
}

} //namespace kdt

#endif