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
