#ifndef MAPPOINT_H
#define MAPPOINT_H

namespace myslam
{
    class Frame;
    class MapPoint
    {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long      id_;         // ID

        bool        good_;              // 是否是好点

        Vector3d    pos_;               // 世界坐标系上的坐标
        Vector3d    norm_;              // 观察方向法线
        Mat         descriptor_;        // 描述符匹配
        int         observed_times_;    // 观察时间
        int         correct_times_;     // 正确时间

        int         matched_times_;     // 匹配时间

        MapPoint();
        MapPoint(long id, Vector3d position, Vector3d norm);

        // 建立MapPoint
        static MapPoint::Ptr createMapPoint();
    };
}

#endif // MAPPOINT_H
