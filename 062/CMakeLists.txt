# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( dense_monocular )

# 定义编译的模式和编译选项
set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )

# Eigen
include_directories( "/usr/include/eigen3" )

# 寻找 opencv 库并添加它的头文件
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# 寻找 Sophus 库并添加它的头文件
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# 添加可执行程序
add_executable( dense_mapping dense_mapping.cpp )

# 与 opencv 和 Sophus 链接
target_link_libraries( dense_mapping ${OpenCV_LIBS} ${Sophus_LIBRARIES} )
