# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( pose_graph )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找 G2O 库并添加它的头文件
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# 寻找 sophus 库并添加它的头文件
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# 寻找 Eigen 库并添加它的头文件
find_package(Eigen3  REQUIRED)
include_directories( "/usr/include/eigen3" )

# 寻找 Cholmod 库并添加它的头文件
find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

# 寻找 Ceres 库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# 添加一个可执行程序
add_executable( pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp )

# 与 G2O 和 Cholmod 链接
target_link_libraries( pose_graph_g2o_SE3
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
)

# 添加一个可执行程序
add_executable( pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp )

# 与 G2O，Sophus 和 Cholmod 链接
target_link_libraries( pose_graph_g2o_lie
    g2o_core g2o_stuff 
    ${CHOLMOD_LIBRARIES}
    ${Sophus_LIBRARIES}
)

