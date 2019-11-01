# 将源文件生成链接文件，并添加到工程
add_library( myslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)

# 将库文件链接到可执行程序上
target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
