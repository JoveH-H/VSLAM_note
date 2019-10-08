# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( g2o_curve_fitting )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O库并添加它的头文件
find_package( G2O REQUIRED )
include_directories( 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

# 寻找OpenCV库并添加它的头文件
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

# 添加一个可执行程序
add_executable( curve_fitting g2o_curve_fitting.cpp )

# 与G2O和OpenCV链接
target_link_libraries( curve_fitting ${OpenCV_LIBS} g2o_core g2o_stuff )
