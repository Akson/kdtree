#include <iostream>
#include <array>
#include <vector>
#include "kdt_kdtree.h"

using namespace kdt;
typedef std::array<double, 3> PointA3;
typedef std::vector<double> PointV;


void main() {
	std::cout << "builder";
    KdTree<PointA3> kdTreeA3(3);
    KdTree<PointV> kdTreeV(3);

    PointA3 p1 = { 1,2,3 };
    kdTreeA3.AddPoint(p1);

    auto nearestPoints = kdTreeA3.FindNearestPoints(p1, 1);
}