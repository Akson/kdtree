# kdtree
Implementation of the k-d tree

-------------------------------------------------------------------------------
How to build on Windows with MSVS:
-------------------------------------------------------------------------------

Requirements:
- Visual Studio 2015 (2013 should work as well but was not tested)

Steps:
- Open the 'kdtree\msvs\kdtree.sln'
- Build all projects
- Executables will be built in the 'kdtree\msvs\bin\'

-------------------------------------------------------------------------------
How to build on Linux with GCC and CMake:
-------------------------------------------------------------------------------

Requirements:
- g++ 4.8.4 (older versions that support C++11 should work but was not tested)
- cmake 2.8.12.2 (2.8.9+ should work but was not tested)

Steps:
- cd kdtree/cmake
- cmake .
- make
- Executables will be built in the 'kdtree/cmake'

-------------------------------------------------------------------------------
How to test:
-------------------------------------------------------------------------------
Warning: 
- On windows executables will have names like 'kdtreequery_32_Release.exe'.

Steps:
- Go to the folder with executables
- kdtpointsgenerator 1000 5 tree_points.txt
- kdtpointsgenerator 10 5 query_points.txt
- kdtreebuilder tree_points.txt tree_points.kdt
- kdtreequery tree_points.kdt query_points.txt
- kdtbenchmark 10000