#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cstdint>

int main(int argc, char * argv[]) {
    if (argc != 4) {
        std::cout
            << "This application generates random n-dimensions points and\n"
            << "saves them to a file that can be used by the kdtreebuilder\n"
            << "or kdtreequery.\n"
            << "\n"
            << "Usage:\n"
            << "kdtpointsgenerator <number points> <number dimensions> <output file name>\n"
            << "\n"
            << "Example:\n"
            << "kdtpointsgenerator 1000 128 points.txt\n"
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

    std::string numDimensionsStr(argv[2]);
    uint32_t numDimensions = 0;
    try {
        numDimensions = std::stoul(numDimensionsStr);
    }
    catch (...) {
        std::cout << "Failed understand number of dimensions: "
            << numDimensionsStr << "\n";
        return -1;
    }

    std::string outputFileName(argv[3]);
    std::ofstream outputFile;
    outputFile.open(outputFileName);
    if (!outputFile.is_open()) {
        std::cout << "Failed to open output file: " << outputFileName << "\n";
        return -1;
    }

    std::srand(static_cast<unsigned int>(std::time(0)));
    for (uint32_t iPoint = 0; iPoint < numPoints; iPoint++) {
        for (uint32_t iDim = 0; iDim < numDimensions; iDim++) {
            double v = double(std::rand()) / RAND_MAX;
            outputFile << v << " ";
        }
        outputFile << "\n";
    }

    std::cout << "DONE\n";

    return 0;
}