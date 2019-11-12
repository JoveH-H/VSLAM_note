# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project(ceres_customBundle)

# 寻找库
find_package(Ceres REQUIRED)
Find_Package(Eigen3 REQUIRED)

include(CheckCXXCompilerFlag)

# 定义编译的模式和编译选项
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# 添加头文件
include_directories(${CERES_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR} 
                    ${PROJECT_SOURCE_DIR}/common
                    ${PROJECT_SOURCE_DIR}/common/tools 
                    ${PROJECT_SOURCE_DIR}/common/flags)

# 生成链接文件，且添加到工程中
add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

# 添加一个可执行程序
add_executable(${PROJECT_NAME} ceresBundle.cpp)

# 将目标文件与库文件进行链接
target_link_libraries(${PROJECT_NAME} BALProblem ParseCmd ${CERES_LIBRARIES} )
