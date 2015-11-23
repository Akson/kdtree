#ifndef KDT_KDTREE_H
#define KDT_KDTREE_H

#include <kdt_limitedpriorityqueue.h>

#include <vector>
#include <stack>
#include <algorithm>
#include <limits>

namespace kdt {

typedef std::size_t NodeIndex; // Used for referncing nodes in a tree
static const NodeIndex None = std::numeric_limits<NodeIndex>::max();
static const double MaxDistance = std::numeric_limits<double>::max();

template <typename PointType> class KdTree {
public: // PUBLIC METHODS

    // Create an empty kd-tree with the number of dimensions specified by the
    // optional parameter 'numDimensions'. If the number of dimensions is not
    // specified during construction, the tree should be created from a binary
    // representation using the 'ReadFromStream' or from a list of points using
    // the 'CreateFromPoints'.
    KdTree(unsigned int numDimensions = 0) : d_numDimensions(numDimensions) {}

    ~KdTree() {}

    // Replace the current tree by the new tree create from points stored in 
    // the vector specified by the 'points' with number of dimesions specified
    // by the 'numDimensions'. If the 'point' vector is empty, create an empty
    // tree.
    void CreateFromPoints(const std::vector<PointType>& points,
                          unsigned int numDimensions);

    // Add a point specified by the 'point' to the existing kd-tree.
    // The behavior is undefined unless the number of  dimension of the added
    // point is equal to number of dimensions of the tree.
    void AddPoint(const PointType& point);

    // Write the tree data in binary representation to the 'stream'.
    void WriteToStream(std::ostream& stream) const;

    // Read the tree's binary representation from the 'stream'.
    // The behavior is undefined unless the stream contains valid kd-tree
    // binary representation.
    void ReadFromStream(std::istream& stream);

    // Return the vector that contains 'numPoints' nearest points to
    // the point specified by the 'point' sorted by distance.
    // If the tree contains less than 'numPoints' points, return all points of
    // the tree. If the tree is empty, return an empty vector.
    std::vector<PointType> FindNearestPoints(const PointType& point,
                                             unsigned int numPoints) const;

    std::vector<PointType> FindNearestPointsBBF(const PointType& point,
                                                unsigned int numPoints) const;

    std::vector<PointType> FindNearestPointsLinear(
        const PointType& point,
        unsigned int numPoints) const;
    
private: // PRIVATE TYPES

    struct Node {
        NodeIndex leftIndex;
        NodeIndex rightIndex;
        PointType point;
    };
    typedef unsigned int Level;
    typedef std::vector<NodeIndex> IndexVector;

private: // PRIVATE DATA

    std::vector<Node> d_nodes;
    NodeIndex d_root = None;
    unsigned int d_numDimensions;

private: // PRIVATE METHODS
    NodeIndex ConstructSubtreeRecursively(
        const std::vector<PointType>& points,
        IndexVector::iterator startIt,
        IndexVector::iterator endIt,
        Level curLevel);
};

template<typename PointType>
NodeIndex KdTree<PointType>::ConstructSubtreeRecursively(
    const std::vector<PointType>& points, 
    IndexVector::iterator startIt, 
    IndexVector::iterator endIt, 
    Level curLevel)
{
    if (startIt >= endIt) {
        return None;
    }

    unsigned int curDimension = curLevel % d_numDimensions;
    auto comparator = [&](NodeIndex a, NodeIndex b) {
        return points[a][curDimension] < points[b][curDimension];
    };
    std::sort(startIt, endIt, comparator);

    unsigned int pivotOffset = (endIt - startIt) / 2;
    IndexVector::iterator pivotIt = startIt + pivotOffset;

    Node newNode;
    newNode.point = points[*pivotIt];
    newNode.leftIndex
        = (pivotOffset == 0)
        ? None
        : ConstructSubtreeRecursively(
            points,
            startIt,
            pivotIt,
            curLevel + 1);
    newNode.rightIndex
        = ConstructSubtreeRecursively(
            points,
            pivotIt + 1,
            endIt,
            curLevel + 1);
    NodeIndex newNodeIndex = d_nodes.size();
    d_nodes.push_back(newNode);
    return newNodeIndex;
}

template<typename PointType>
void KdTree<PointType>::CreateFromPoints(
    const std::vector<PointType>& points,
    unsigned int numDimensions)
{
    d_numDimensions = numDimensions;
    d_nodes.clear();
    d_nodes.reserve(points.size());

    std::vector<NodeIndex> indexes(points.size());
    for (int i = 0; i < points.size(); i++) {
        indexes[i] = i;
    }

    d_root = ConstructSubtreeRecursively(
        points, 
        indexes.begin(), 
        indexes.end(), 
        0);
}

template<typename PointType>
void KdTree<PointType>::WriteToStream(std::ostream& stream) const {
    unsigned int numNodes = d_nodes.size();
    stream.write(reinterpret_cast<const char*>(&numNodes), sizeof numNodes);
    stream.write(reinterpret_cast<const char*>(&d_numDimensions), 
                 sizeof d_numDimensions);
    stream.write(reinterpret_cast<const char*>(&d_root), sizeof d_root);
    
    for (const auto& node : d_nodes) {
        node.point.WritePointToStream(stream, d_numDimensions);
        stream.write(reinterpret_cast<const char*>(&node.leftIndex),
                     sizeof node.leftIndex);
        stream.write(reinterpret_cast<const char*>(&node.rightIndex),
                     sizeof node.rightIndex);
    }
}

template<typename PointType>
void KdTree<PointType>::ReadFromStream(std::istream& stream) {
    unsigned int numNodes;
    stream.read(reinterpret_cast<char*>(&numNodes), sizeof numNodes);
    stream.read(reinterpret_cast<char*>(&d_numDimensions), 
                sizeof d_numDimensions);
    stream.read(reinterpret_cast<char*>(&d_root), sizeof d_root);
    
    d_nodes.clear();
    d_nodes.resize(numNodes);
    for (unsigned int i = 0; i < numNodes; i++) {
        Node& node = d_nodes[i];
        node.point.ReadPointFromStream(stream, d_numDimensions);
        stream.read(reinterpret_cast<char*>(&node.leftIndex), 
                    sizeof node.leftIndex);
        stream.read(reinterpret_cast<char*>(&node.rightIndex),
                    sizeof node.rightIndex);
    }
}

template<typename PointType>
void KdTree<PointType>::AddPoint(const PointType & point)
{
    NodeIndex newNodeIndex = d_nodes.size();
    Node newNode;
    newNode.leftIndex = None;
    newNode.rightIndex = None;
    newNode.point = point;
    d_nodes.push_back(newNode);

    if (None == d_root) {
        // It's the first node of the tree
        d_root = newNodeIndex;
        return;
    }

    NodeIndex curNodeIndex = d_root;
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
    return;
}

template<typename PointType>
std::vector<PointType> KdTree<PointType>::FindNearestPoints(
    const PointType & point,
    unsigned int numPoints) const
{
    LimitedDistanceQueue<NodeIndex> limitedQueue(numPoints);

    struct Bin {
        Level level;
        NodeIndex nodeIndex;
        double minBinDistanceSq;
    };
    std::stack<Bin> backtrackingStack;
    
    NodeIndex curNodeIndex = d_root;
    Level curLevel = 0;
    while (!backtrackingStack.empty() || None != curNodeIndex) {
        if (None == curNodeIndex) {
            Bin curBin = backtrackingStack.top();
            backtrackingStack.pop();

            curLevel = curBin.level;
            curNodeIndex = curBin.nodeIndex;

            if (curBin.minBinDistanceSq > limitedQueue.MaxDistance()
                && limitedQueue.IsFull()) {
                // Current subtree cannot have points closer than the farthest
                // already found point and there are enough points found.
                curNodeIndex = None;
                continue;
            }
        }
        
        const Node& curNode = d_nodes[curNodeIndex];
        limitedQueue.Push(curNodeIndex, point.Distance(curNode.point));

        unsigned int dimension = curLevel % d_numDimensions;
        double distanceToBorder
            = abs(point[dimension] - curNode.point[dimension]);
        double distanceToBorderSq = distanceToBorder * distanceToBorder;

        NodeIndex otherNodeIndex = None;
        if (point[dimension] < curNode.point[dimension]) {
            // left
            curNodeIndex = curNode.leftIndex;
            otherNodeIndex = curNode.rightIndex;
        } else {
            // right
            curNodeIndex = curNode.rightIndex;
            otherNodeIndex = curNode.leftIndex;
        }

        if (None != otherNodeIndex) {
            Bin backTrackBin;
            backTrackBin.nodeIndex = otherNodeIndex;
            backTrackBin.level = curLevel + 1;
            backTrackBin.minBinDistanceSq = distanceToBorderSq;
            backtrackingStack.push(backTrackBin);
        }

        curLevel++;
    }

    auto nodeIndexes = limitedQueue.Items();
    std::vector<PointType> result;
    result.reserve(nodeIndexes.size());
    for (auto& index : nodeIndexes) {
        result.push_back(d_nodes[index].point);
    }
    return result;
}

template<typename PointType>
std::vector<PointType> KdTree<PointType>::FindNearestPointsBBF(
    const PointType & point,
    unsigned int numPoints) const
{
    LimitedDistanceQueue<NodeIndex> limitedQueue(numPoints);

    struct Bin {
        Level level;
        NodeIndex nodeIndex;
        double minBinDistanceSq;
    };
    LimitedDistanceQueue<Bin> backtrackingQueue(
        std::numeric_limits<unsigned int>::max());

    NodeIndex curNodeIndex = d_root;
    Level curLevel = 0;
    while (!backtrackingQueue.Empty() || None != curNodeIndex) {
        if (None == curNodeIndex) {
            Bin curBin = backtrackingQueue.PopFront();

            curLevel = curBin.level;
            curNodeIndex = curBin.nodeIndex;

            if (curBin.minBinDistanceSq > limitedQueue.MaxDistance()
                && limitedQueue.IsFull()) {
                // Current subtree cannot have points closer than the farthest
                // already found point and there are enough points found.
                curNodeIndex = None;
                continue;
            }
        }
        
        const Node& curNode = d_nodes[curNodeIndex];
        limitedQueue.Push(curNodeIndex, point.Distance(curNode.point));

        unsigned int dimension = curLevel % d_numDimensions;
        double distanceToBorder
            = abs(point[dimension] - curNode.point[dimension]);
        double distanceToBorderSq = distanceToBorder * distanceToBorder;

        NodeIndex otherNodeIndex = None;
        if (point[dimension] < curNode.point[dimension]) {
            // left
            curNodeIndex = curNode.leftIndex;
            otherNodeIndex = curNode.rightIndex;
        } else {
            // right
            curNodeIndex = curNode.rightIndex;
            otherNodeIndex = curNode.leftIndex;
        }

        if (None != otherNodeIndex) {
            Bin backTrackBin;
            backTrackBin.nodeIndex = otherNodeIndex;
            backTrackBin.level = curLevel + 1;
            backTrackBin.minBinDistanceSq = distanceToBorderSq;
            backtrackingQueue.Push(backTrackBin, distanceToBorderSq);
        }

        curLevel++;
    }

    auto nodeIndexes = limitedQueue.Items();
    std::vector<PointType> result;
    result.reserve(nodeIndexes.size());
    for (auto& index : nodeIndexes) {
        result.push_back(d_nodes[index].point);
    }
    return result;
}

template<typename PointType>
std::vector<PointType> KdTree<PointType>::FindNearestPointsLinear(
    const PointType & point,
    unsigned int numPoints) const
{
    // Linear search for testing purpose
    LimitedDistanceQueue<NodeIndex> limitedQueue(numPoints);
    for (NodeIndex i = 0; i < d_nodes.size(); i++) {
        limitedQueue.Push(i, point.Distance(d_nodes[i].point));
    }

    auto nodeIndexes = limitedQueue.Items();
    std::vector<PointType> result;
    result.reserve(nodeIndexes.size());
    for (auto& index : nodeIndexes) {
        result.push_back(d_nodes[index].point);
    }
    return result;
}

} //namespace kdt

#endif