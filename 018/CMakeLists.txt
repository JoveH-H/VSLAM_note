# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( useSophus )

# 为使用 sophus，需要使用find_package命令找到它
find_package( Sophus REQUIRED )

# 添加Eigen头文件
include_directories( ${Sophus_INCLUDE_DIRS} )

# 添加一个可执行程序
add_executable( useSophus_SO3 useSophus_SO3.cpp )
add_executable( useSophus_SE3 useSophus_SE3.cpp )

# 将库文件链接到可执行程序上
target_link_libraries( useSophus_SO3 ${Sophus_LIBRARIES} )
target_link_libraries( useSophus_SE3 ${Sophus_LIBRARIES} )

