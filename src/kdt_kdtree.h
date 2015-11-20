#include <vector>

namespace kdt {

template <typename PointType> class KdTree {
public:
    // Create an empty kd-tree with the number of dimensions specified by the
    // 'nDimensions' parameter
    KdTree(unsigned int nDimensions) {}
    ~KdTree() {}

    // Add a point specified by the 'point' to the existing kd-tree
    void AddPoint(const PointType& point);

    // Return the vector that contains 'numPoints' nearest points to the point
    // specified by the 'point'. If the tree contains less than 'numPoints'
    // points, return all points of the tree.
    std::vector<PointType> FindNearestPoints(const PointType& point,
                                             unsigned int numPoints);

private:
    std::vector<PointType> d_points;
};


template<typename PointType>
inline void KdTree<PointType>::AddPoint(const PointType & point)
{
    d_points.push_back(point);
}

template<typename PointType>
inline std::vector<PointType> KdTree<PointType>::FindNearestPoints(const PointType & point, unsigned int numPoints)
{
    return d_points;
}

} //namespace kdt
