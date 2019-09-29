# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( joinMap )

# 定义编译的模式
set( CMAKE_BUILD_TYPE Release )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 添加Eigen头文件 
include_directories( "/usr/include/eigen3/" )

# 寻找PCL库
find_package( PCL REQUIRED COMPONENT common io )
# 添加头文件
include_directories( ${PCL_INCLUDE_DIRS} )
# 添加编译选项
add_definitions( ${PCL_DEFINITIONS} )

# 添加一个可执行程序
add_executable( joinMap joinMap.cpp )
# 将库文件链接到可执行程序上
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

