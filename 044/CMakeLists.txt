# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( directMethod )

# 定义编译的模式和编译选项
set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找OpenCV库并添加它的头文件
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 寻找G2O库并添加它的头文件
find_package( G2O )
include_directories( ${G2O_INCLUDE_DIRS} ) 

# 添加eigen3库的头文件
include_directories( "/usr/include/eigen3" )

# 添加G2O库
set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

# 添加一个可执行程序
add_executable( direct_sparse direct_sparse.cpp )
target_link_libraries( direct_sparse ${OpenCV_LIBS} ${G2O_LIBS} )

# 添加一个可执行程序
add_executable( direct_semidense direct_semidense.cpp )
target_link_libraries( direct_semidense ${OpenCV_LIBS} ${G2O_LIBS} )
