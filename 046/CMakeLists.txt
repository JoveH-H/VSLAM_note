# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project ( myslam )

# 定义编译的模式和编译选项
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

# 添加cmake模块路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 设置可执行文件的输出目录
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

# 设置库文件的输出目录
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### 依赖关系 ######################
# 寻找OpenCV库并添加它的头文件
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 寻找Sophus库并添加它的头文件
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )


# 寻找G2O库并添加它的头文件
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} ) 

# 添加eigen3库的头文件
include_directories( "/usr/include/eigen3" )

# 添加库
set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)

############### 源码和测试文件 ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
