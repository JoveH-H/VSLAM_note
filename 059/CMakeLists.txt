# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( loop_closure )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 寻找 opencv 库并添加它的头文件
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# dbow3 是一个简单的库，假设安装在默认目录
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

# 添加可执行程序
add_executable( loop_closure loop_closure.cpp )
add_executable( gen_vocab gen_vocab_large.cpp )

# 与 opencv 和 dbow3 链接
target_link_libraries( loop_closure ${OpenCV_LIBS} ${DBoW3_LIBS} )
target_link_libraries( gen_vocab ${OpenCV_LIBS} ${DBoW3_LIBS} )
