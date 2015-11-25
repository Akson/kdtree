#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <ctime>
#include <chrono>
#include <algorithm>

#include <kdt_kdtree.h>
#include <kdt_point.h>

const unsigned int nearestPointsNumber = 3;
const unsigned int numQueryPoints = 100;

template<size_t NumDimensions, typename Point> void Test(uint32_t numPoints) {
    typedef kdt::KdTree<Point> KdTree;

    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point endTime;
    double deltaSeconds;

    unsigned int numDimensions = NumDimensions;

    std::cout << "Generating " << numPoints << " with " << numDimensions
        << " dimensions\n";
    std::vector<Point> points(numPoints);
    std::srand(static_cast<unsigned int>(std::time(0)));
    for (uint32_t iPoint = 0; iPoint < numPoints; iPoint++) {
        points[iPoint].resize(numDimensions);
        for (uint32_t iDim = 0; iDim < numDimensions; iDim++) {
            double v = double(std::rand()) / RAND_MAX;
            points[iPoint][iDim] = v;
        }
        points[iPoint].userData = iPoint;
    }

    // Construct kd-tree
    std::cout << "Constructing kd-tree with " << numDimensions
        << " dimensions\n";
    KdTree tree(numDimensions);
    startTime = std::chrono::high_resolution_clock::now();
    tree.CreateFromPoints(points, numDimensions);
    endTime = std::chrono::high_resolution_clock::now();
    deltaSeconds
        = std::chrono::duration_cast<std::chrono::duration<double>>(
            endTime - startTime).count() * 1000;
    std::cout << "Done in " << deltaSeconds << " ms.\n";

    // Searching for closest points
    std::vector<double> queryTime[3];
    std::cout << "Searching for " << nearestPointsNumber 
        << " closest points " << numQueryPoints << " times\n";
    for (int iQueryPoint = 0;
        iQueryPoint < numQueryPoints;
            iQueryPoint++) {

        Point queryPoint;
        queryPoint.resize(numDimensions);
        for (uint32_t iDim = 0; iDim < numDimensions; iDim++) {
            double v = double(std::rand()) / RAND_MAX;
            queryPoint[iDim] = v;
        }

        Point p1 = queryPoint;
        startTime = std::chrono::high_resolution_clock::now();
        auto nearestPointsIndexes
            = tree.FindNearestPointsLinear(queryPoint, nearestPointsNumber);
        endTime = std::chrono::high_resolution_clock::now();
        deltaSeconds
            = std::chrono::duration_cast<std::chrono::duration<double>>(
                endTime - startTime).count() * 1000;
        queryTime[0].push_back(deltaSeconds);

        startTime = std::chrono::high_resolution_clock::now();
        nearestPointsIndexes
            = tree.FindNearestPoints(queryPoint, nearestPointsNumber);
        endTime = std::chrono::high_resolution_clock::now();
        deltaSeconds
            = std::chrono::duration_cast<std::chrono::duration<double>>(
                endTime - startTime).count() * 1000;
        queryTime[1].push_back(deltaSeconds);

        startTime = std::chrono::high_resolution_clock::now();
        nearestPointsIndexes
            = tree.FindNearestPointsBBF(queryPoint, nearestPointsNumber);
        endTime = std::chrono::high_resolution_clock::now();
        deltaSeconds
            = std::chrono::duration_cast<std::chrono::duration<double>>(
                endTime - startTime).count() * 1000;
        queryTime[2].push_back(deltaSeconds);
    }

    for (int iAlgorithm = 0; iAlgorithm < 3; iAlgorithm++) {
        std::sort(queryTime[iAlgorithm].begin(), queryTime[iAlgorithm].end());
    }
    
    std::cout.precision(6);
    std::cout << "\n" << std::fixed;
    std::vector<double>& results = queryTime[0];
    std::cout << "Linear "
        << results[results.size() / 2] << " | "
        << results[0] << " | "
        << results[results.size() - 1] << " ms (med|min|max)\n";
    results = queryTime[1];
    std::cout << "Normal "
        << results[results.size() / 2] << " | "
        << results[0] << " | "
        << results[results.size() - 1] << " ms (med|min|max)\n";
    results = queryTime[2];
    std::cout << "Bbf    "
        << results[results.size() / 2] << " | "
        << results[0] << " | "
        << results[results.size() - 1] << " ms (med|min|max)\n";
    std::cout << "\n";
}

template<size_t NumDimensions> 
void TestMultiplePointImplementations(uint32_t numPoints) {
    typedef kdt::ArrayPointWithUserData<double, NumDimensions, uint32_t> PointA;
    typedef kdt::VectorPointWithUserData<double, uint32_t> PointV;

    std::cout << "Dimensions: " << NumDimensions << "\n\n";
    std::cout << "Testing std::array based point\n";
    Test<NumDimensions, PointA>(numPoints);
    std::cout << "Testing std::vector based point\n";
    Test<NumDimensions, PointV>(numPoints);

    std::cout << "\n";
}

void TestMultipleDimensions(uint32_t numPoints) {
    TestMultiplePointImplementations<1>(numPoints);
    TestMultiplePointImplementations<2>(numPoints);
    TestMultiplePointImplementations<3>(numPoints);
    TestMultiplePointImplementations<16>(numPoints);
    TestMultiplePointImplementations<128>(numPoints);
}

int main(int argc, char * argv[]) {
    if (argc != 2) {
        std::cout
            << "This application takes performance measurements of the\n"
            << "kdtree implementation. It creates a kdtree with uniformly\n"
            << "distributed points and run 100 queries with random points.\n"
            << "It tests 2 type of points:\n"
            << "- based on std::vector\n"
            << "- based on std::array\n"
            << "It tests 3 search algorithms:\n"
            << "- linear search\n"
            << "- normal search\n"
            << "- best bin first search\n"
            << "It tests trees with multiple dimensions: 1, 2, 3, 16, 128\n"
            << "\n"
            << "Usage:\n"
            << "kdtbenchmark <number of points in the tree>\n"
            << "\n"
            << "Example:\n"
            << "kdtbenchmark 1000\n"
            << "\n";
        return 0;
    }

    std::string numPointsStr(argv[1]);
    uint32_t numPoints = 0;
    try {
        numPoints = std::stoul(numPointsStr);
    }
    catch (...) {
        std::cout << "Failed understand number of points: "
            << numPointsStr << "\n";
        return -1;
    }

    TestMultipleDimensions(numPoints);
 
    std::cout << "\nDone\n";
    std::cin.ignore();
    return 0;
}