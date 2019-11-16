# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( pointcloud_mapping )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找 opencv 库并添加它的头文件
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 添加 Eigen 头文件
include_directories( "/usr/include/eigen3/" )

# 寻找 pcl 库并添加它的头文件 
find_package( PCL REQUIRED COMPONENT common io filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# 添加可执行程序
add_executable( pointcloud_mapping pointcloud_mapping.cpp )

# 与 opencv 和 pcl 链接
target_link_libraries( pointcloud_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} )
