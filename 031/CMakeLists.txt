# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( feature_extraction )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 寻找OpenCV库并添加它的头文件
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_LIBS} )

# 添加一个可执行程序
add_executable( feature_extraction feature_extraction.cpp  )

# 与OpenCV链接
target_link_libraries( feature_extraction ${OpenCV_LIBS} )
