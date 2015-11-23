#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<sstream>
#include"kdt_kdtree.h"
#include"kdt_point.h"

typedef kdt::VectorPointWithUserData<double, int> Point;
typedef kdt::KdTree<Point> KdTree;


int main(int argc, char * argv[]) {
    if (argc != 3) {
        std::cout
            << "This application reads the kdtree from the binary file and\n"
            << "the list of query points from the text file. Then it\n"
            << "and reports the top 3 nearest neighbors for each query point\n"
            << "\n"
            << "Usage:\n"
            << "main_kdtreequery <tree file name> <query points file name>\n"
            << "\n"
            << "Example:\n"
            << "kdtreebuilder sample_data.kdt query_points.txt\n"
            << "\n";
        return 0;
    }

    // Open input query file
    std::string inputQueryFileName(argv[2]);
    std::ifstream inputQueryFile;
    inputQueryFile.open(inputQueryFileName);
    if (!inputQueryFile.is_open()) {
        std::cout << "Failed to open the input query file: "
            << inputQueryFileName << "\n";
        return -1;
    }

    // Read query points from input file
    std::cout << "Reading query points from: " << inputQueryFileName << "\n";
    std::vector<Point> queryPoints;
    for (std::string inputLine; std::getline(inputQueryFile, inputLine); ) {
        std::istringstream inputLineStream(inputLine);
        Point point;
        double currentCoordinate;
        while (inputLineStream >> currentCoordinate) {
            point.push_back(currentCoordinate);
        }
        point.userData = queryPoints.size();
        queryPoints.push_back(point);
    }
    inputQueryFile.close();

    if (queryPoints.empty()) {
        std::cout << "No points read from the input file\n";
        return 0;
    }
    std::cout << "Read " << queryPoints.size() << " query points\n";

    // Open input tree file
    std::string inputTreeFileName(argv[1]);
    std::ifstream inputTreeFile;
    inputTreeFile.open(inputTreeFileName, std::ios::binary);
    if (!inputTreeFile.is_open()) {
        std::cout << "Failed to open the input tree file: "
            << inputTreeFileName << "\n";
        return -1;
    }

    // Reading kd-tree from file
    std::cout << "Reading the tree from the input file: "
        << inputTreeFileName << "\n";
    KdTree tree(0);
    tree.ReadFromStream(inputTreeFile);
    inputTreeFile.close();

    // Searching for closest points
    const unsigned int nearestPointsNumber = 3;
    std::cout << "Searching for "<< nearestPointsNumber << " closest points";
    for (int iQueryPoint = 0;
        iQueryPoint < queryPoints.size(); 
        iQueryPoint++) {
        Point queryPoint = queryPoints[iQueryPoint];
        std::cout << "Query point " << iQueryPoint;

        std::cout << "\nLinear search:  ";
        Point p1 = queryPoint;
        auto nearestPointsIndexes
            = tree.FindNearestPointsLinear(p1, nearestPointsNumber);
        for (auto point : nearestPointsIndexes) {
            std::cout << point.userData << ", ";
        }

        std::cout << "\nNormal search:  ";
        nearestPointsIndexes 
            = tree.FindNearestPoints(p1, nearestPointsNumber);
        for (auto point : nearestPointsIndexes) {
            std::cout << point.userData << ", ";
        }

        std::cout << "\nBest bin first: ";
        nearestPointsIndexes 
            = tree.FindNearestPointsBBF(p1, nearestPointsNumber);
        for (auto point : nearestPointsIndexes) {
            std::cout << point.userData << ", ";
        }

        std::cout << "\n\n";
    }

    std::cout << "DONE\n";
    std::cin.ignore();

    return 0;
}