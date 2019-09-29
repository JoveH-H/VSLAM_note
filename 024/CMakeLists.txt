# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( imageBasics )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )

# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 添加一个可执行程序
add_executable( imageBasics imageBasics.cpp )

# 链接OpenCV库
target_link_libraries( imageBasics ${OpenCV_LIBS} )
