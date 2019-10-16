# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( pose_estimation_3d2d )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找OpenCV库并添加它的头文件
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_LIBS} )

# 寻找G2O库并添加它的头文件
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# 寻找CSparse库并添加它的头文件
find_package( CSparse REQUIRED )
include_directories( ${CSPARSE_INCLUDE_DIR} )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加一个可执行程序
add_executable( pose_estimation_3d2d pose_estimation_3d2d.cpp )

# 与G2O和OpenCV链接
target_link_libraries( pose_estimation_3d2d 
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)
