#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<sstream>
#include"kdt_kdtree.h"
#include"kdt_point.h"

typedef kdt::VectorPointWithUserData<double, uint32_t> Point;
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
    for (std::string inputLine; std::getline(inputFile, inputLine); ) {
        std::istringstream inputLineStream(inputLine);
        Point point;
        double currentCoordinate;
        while (inputLineStream  >> currentCoordinate) {
            point.push_back(currentCoordinate);
        }
        point.userData = points.size();
        points.push_back(point);
    }
    inputFile.close();
    if (points.empty()) {
        std::cout << "No points read from the input file\n";
        return 0;
    }
    std::cout << "Read " << points.size() << " points\n";

    // Construct kd-tree
    unsigned int numDimensions = points[0].size();
    std::cout << "Constructing kd-tree with " << numDimensions 
        << " dimensions\n";
    KdTree tree(numDimensions);
    tree.CreateFromPoints(points, numDimensions);

    // Write kd-tree to file
    std::cout << "Write kd-tree to file: " << outputFileName << "\n";
    tree.WriteToStream(outputFile);
    outputFile.close();

    std::cout << "\nDone\n";
    return 0;
}