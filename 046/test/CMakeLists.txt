# 添加一个可执行程序
add_executable( run_vo run_vo.cpp )

# 将文件链接到可执行程序上
target_link_libraries( run_vo myslam )
