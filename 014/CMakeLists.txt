# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( geometry )

set(CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加一个可执行程序
add_executable( eigenGeometry eigenGeometry.cpp )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# 添加一个可执行程序
add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )
