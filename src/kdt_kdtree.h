#ifndef KDT_KDTREE_H
#define KDT_KDTREE_H

#include <kdt_limitedpriorityqueue.h>

#include <vector>

namespace kdt {

typedef std::size_t Index; // Used for referncing nodes in a tree
static const Index None = std::numeric_limits<Index>::max();

template <typename PointType> struct KdTreeNode {
    Index left;
    Index right;
    PointType point;

    // Create a node with point specified by the 'p' and not children
    KdTreeNode(const PointType& p) : left(None), right(None), point(p) {}
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

private:
    typedef KdTreeNode<PointType> Node;

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
    unsigned int d_numDimensions;
};


template<typename PointType>
inline void KdTree<PointType>::AddPoint(const PointType & point)
{
    // TODO: check dimensions
    d_nodes.push_back(Node(point));
}

template<typename PointType>
inline std::vector<Index> KdTree<PointType>::FindNearestPoints(
                                                       const PointType & point,
                                                       unsigned int numPoints)
{
    LimitedDistanceQueue<Index> limitedQueue(numPoints);
    for (Index i = 0; i < d_nodes.size(); i++) {
        limitedQueue.Push(i, Distance(point, d_nodes[i].point));
    }
    return limitedQueue.Items();
}

} //namespace kdt

#endif