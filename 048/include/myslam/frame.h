#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
    class MapPoint;
    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long                  id_;            // 帧的id
        double                         time_stamp_;    // 记录的时间
        SE3                            T_c_w_;         // 从世界到相机的转换
        Camera::Ptr                    camera_;        // 针孔/RGBD相机模型
        Mat                            color_, depth_; // 颜色和深度图像

        bool                           is_key_frame_;  // 是否关键帧

    public: // 数据成员
        Frame();
        Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(), Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
        ~Frame();

        // 创建 Frame
        static Frame::Ptr createFrame();

        // 寻找给定点对应的深度
        double findDepth(const cv::KeyPoint& kp);

        // 获取相机光心
        Vector3d getCamCenter() const;

        void setPose( const SE3& T_c_w );

        // 判断某个点是否在视野内
        bool isInFrame(const Vector3d& pt_world);
    };
}

#endif // FRAME_H
