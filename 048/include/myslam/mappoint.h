#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    class Frame;
    class MapPoint
    {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long      id_;           // ID
        static unsigned long factory_id_;    
        bool        good_;                // 是否是好点
        Vector3d    pos_;                 // 世界坐标系上的坐标
        Vector3d    norm_;                // 观察方向法线
        Mat         descriptor_;          // 描述符匹配

        list<Frame*>    observed_frames_; // 观察时间

        int         matched_times_;       // 匹配时间
        int         visible_times_;       // 一帧时间

        MapPoint();
        MapPoint(
            unsigned long id,
            const Vector3d& position,
            const Vector3d& norm,
            Frame* frame = nullptr,
            const Mat& descriptor = Mat()
        );

        inline cv::Point3f getPositionCV() const {
            return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
        }

        // 建立MapPoint
        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr createMapPoint(
            const Vector3d& pos_world,
            const Vector3d& norm_,
            const Mat& descriptor,
            Frame* frame);
    };
}

#endif // MAPPOINT_H

