#include <gtest/gtest.h>

#include <kdt_kdtree.h>
#include <kdt_point.h>

typedef kdt::VectorPointWithUserData<double, uint32_t> Point;
typedef kdt::KdTree<Point> KdTree;

TEST(testKdTree, emptyTreeReturnsEmptyResult)
{
    Point queryPoint;
    KdTree tree(1);

    auto result = tree.FindNearestPoints(queryPoint, 3);

    ASSERT_TRUE(result.empty());
}