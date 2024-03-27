# v3.8.6
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

This code has been modified for ````Ubuntu 22.04````.

### Notice
support robot: Go1

not support robot: Laikago, B1, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Dependencies
* [Unitree](https://www.unitree.com/download)
```bash
Legged_sport    >= v1.36.0
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```
* [Boost](https://www.boost.org/users/history/version_1_71_0.html) (only version 1.71 tested)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [g++](https://gcc.gnu.org/) (version 8.3.0 or higher)

### Boost Install Instructions
This instruction is about where to place the downloaded boost library. There is no need to install it. 

Download the file named
````
boost_1_71_0.tar.bz2
````
In the directory where you want to put the Boost installation, execute
````
tar --bzip2 -xf <your_path_to/boost_1_71_0.tar.bz2>
````
You will now see a folder named ````boost_1_71_0````. Copy this folder to the root directory of ````unitree_legged_sdk````. All that is left is to build.

### Build
```bash
mkdir build
cd build
cmake ..
make
```

If you want to build the python wrapper, then replace the cmake line with:
```bash
cmake -DPYTHON_BUILD=TRUE ..
```

If can not find pybind11 headers, then add
```bash
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/pybind11/include)
```
at line 14 in python_wrapper/CMakeLists.txt.

If can not find msgpack.hpp, then
```bash
sudo apt install libmsgpack*
```

### Run

#### Cpp
Run examples with 'sudo' for memory locking.

#### Python
##### arm
change `sys.path.append('../lib/python/amd64')` to `sys.path.append('../lib/python/arm64')`
