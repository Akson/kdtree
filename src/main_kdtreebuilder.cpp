#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<sstream>
#include"kdt_kdtree.h"

typedef std::vector<double> Point;
typedef kdt::KdTree<Point> KdTree;

int main(int argc, char * argv[]) {
    if (argc != 3) {
        std::cout
            << "This application takes a list of points from the input text\n"
            << "file, constructs a kd-tree and stores it in the output file.\n"
            << "\n"
            << "Usage:\n"
            << "kdtreebuilder <input file name> <output file name>\n"
            << "\n"
            << "Example:\n"
            << "kdtreebuilder sample_data.txt sample_data.kdt\n"
            << "\n";
        return 0;
    }

    // Open input file
    std::string inputFileName(argv[1]);
    std::ifstream inputFile;
    inputFile.open(inputFileName);
    if (!inputFile.is_open()) {
        std::cout << "Failed to open input file: " << inputFileName << "\n";
        return -1;
    }

    // Open output file before computing to make sure results can be saved
    std::string outputFileName(argv[2]);
    std::ofstream outputFile;
    outputFile.open(outputFileName, std::ios::binary);
    if (!outputFile.is_open()) {
        std::cout << "Failed to open output file: " << outputFileName << "\n";
        return -1;
    }

    // Read points from input file
    std::cout << "Reading points from: " << inputFileName << "\n";
    std::vector<Point> points;
    /*for (std::string inputLine; std::getline(inputFile, inputLine); ) {
        std::istringstream inputLineStream(inputLine);
        Point point;
        double currentCoordinate;
        while (inputLineStream  >> currentCoordinate) {
            point.push_back(currentCoordinate);
        }
        points.push_back(point);
    }
    inputFile.close();
    if (points.empty()) {
        std::cout << "No points read from the input file\n";
        return 0;
    }
    std::cout << "Read " << points.size() << " points\n";*/

    int dimensions = 3;
    for (int i = 0; i < 10000; i++) {
        Point point;
        for (int j = 0; j < dimensions; j++) {
            double r = (double) std::rand() / RAND_MAX;
            point.push_back(r);
        }
        points.push_back(point);
    }

    // Construct kd-tree
    unsigned int numDimensions = points[0].size();
    std::cout << "Constructing kd-tree with " << numDimensions 
        << " dimensions\n";
    KdTree tree(numDimensions);
    for (auto point : points) {
        tree.AddPoint(point);
    }

    // Write kd-tree to file
    tree.WriteToStream(outputFile);
    outputFile.close();

    KdTree tree1(0);
    std::ifstream ioutputFile;
    ioutputFile.open(outputFileName, std::ios::binary);
    tree1.ReadFromStream(ioutputFile);
    ioutputFile.close();

    // Test
    int k = 10;
    for (int i = 0; i < 10; i++) {
        Point point;
        for (int j = 0; j < dimensions; j++) {
            double r = (double)std::rand() / RAND_MAX;
            //double r = j / 10.0;
            point.push_back(r);
        }

        std::cout << "Testing linear:\n";
        Point p1 = point;
        auto nearestPointsIndexes = tree.FindNearestPointsLinear(p1, k);
        auto nearestPointsIndexes1 = tree1.FindNearestPointsLinear(p1, k);
        for (auto index : nearestPointsIndexes) {
            std::cout << index << ", ";
        }
        std::cout << "\n";
        for (auto index : nearestPointsIndexes1) {
            std::cout << index << ", ";
        }

        std::cout << "\nTesting effective:\n";
        nearestPointsIndexes = tree.FindNearestPoints(p1, k);
        nearestPointsIndexes1 = tree1.FindNearestPoints(p1, k);
        for (auto index : nearestPointsIndexes) {
            std::cout << index << ", ";
        }

        std::cout << "\nTesting BBF:\n";
        nearestPointsIndexes = tree.FindNearestPointsBBF(p1, k);
        nearestPointsIndexes1 = tree1.FindNearestPointsBBF(p1, k);
        for (auto index : nearestPointsIndexes) {
            std::cout << index << ", ";
        }

        std::cout << "\n\n";
    }
    std::cin.ignore();

    return 0;
}