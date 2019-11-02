#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
    class Map
    {
    public:
        typedef shared_ptr<Map> Ptr;
        unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // 所有路标点
        unordered_map<unsigned long, Frame::Ptr >     keyframes_;         // 所有关键帧

        Map() {}

        void insertMapPoint(MapPoint::Ptr map_point);                     // 插入路标点
        void insertKeyFrame(Frame::Ptr frame);                            // 插入关键帧
    };
}

#endif // MAP_H
