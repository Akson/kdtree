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
    double Distance(const PointType& a, const PointType& b) {
        double distance = 0.0;
        for (unsigned i = 0; i < a.size(); i++) {
            double diff = a[i] - b[i];
            distance += diff * diff;
        }
        return distance;
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
    StackFrame frame;
    frame.level = 0;
    frame.subtreeRootIndex = d_root;
    frame.minSubtreeDistance = MaxDistance;
    stack.push(frame);
    while (!stack.empty()) {
        StackFrame frame = stack.top();
        stack.pop();
        
        if (None == frame.subtreeRootIndex) {
            continue;
        }
        if (frame.minSubtreeDistance > limitedQueue.MaxDistance() 
            && limitedQueue.IsFull()) {
            continue;
        }

        Node& curNode = d_nodes[frame.subtreeRootIndex];
        limitedQueue.Push(frame.subtreeRootIndex, 
                          Distance(point, curNode.point));

        unsigned int level = frame.level;
        unsigned int dimension = level % d_numDimensions;

        frame.level = level + 1;
        double distanceToBorder 
            = abs(point[dimension] - curNode.point[dimension]);
        if (point[dimension] < curNode.point[dimension]) {
            // left
            frame.subtreeRootIndex = curNode.leftIndex;
            frame.minSubtreeDistance = 0;
            stack.push(frame);

            frame.subtreeRootIndex = curNode.rightIndex;
            frame.minSubtreeDistance = distanceToBorder;
            stack.push(frame);
        } else {
            // right
            frame.subtreeRootIndex = curNode.rightIndex;
            frame.minSubtreeDistance = 0;
            stack.push(frame);

            frame.subtreeRootIndex = curNode.leftIndex;
            frame.minSubtreeDistance = distanceToBorder;
            stack.push(frame);
        }
    }

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
        limitedQueue.Push(i, Distance(point, d_nodes[i].point));
    }
    return limitedQueue.Items();
}

} //namespace kdt

#endif