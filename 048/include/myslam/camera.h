#ifndef CAMERA_H // 防止头文件重复引用
#define CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{
    // 针孔/RGBD 相机模型
    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;
        float   fx_, fy_, cx_, cy_, depth_scale_;  // 内参

        Camera();   // 定义 Camera 的指针类型
        Camera(float fx, float fy, float cx, float cy, float depth_scale = 0) :
            fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale)
        {}

        // 坐标变换:世界，相机，像素
        Vector3d world2camera(const Vector3d& p_w, const SE3& T_c_w);
        Vector3d camera2world(const Vector3d& p_c, const SE3& T_c_w);
        Vector2d camera2pixel(const Vector3d& p_c);
        Vector3d pixel2camera(const Vector2d& p_p, double depth = 1);
        Vector3d pixel2world(const Vector2d& p_p, const SE3& T_c_w, double depth = 1);
        Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w);
    };
}
#endif // CAMERA_H
