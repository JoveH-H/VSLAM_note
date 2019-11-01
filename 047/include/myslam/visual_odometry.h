#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
    class VisualOdometry
    {
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING = -1,
            OK = 0,
            LOST
        };

        VOState     state_;     // 当前 VO 状态 
        Map::Ptr    map_;       // 映射所有帧和映射点
        Frame::Ptr  ref_;       // 参考坐标系
        Frame::Ptr  curr_;      // 当前帧

        cv::Ptr<cv::ORB> orb_;  // ORB 检测和计算器
        vector<cv::Point3f>     pts_3d_ref_;        // 参考坐标系中的三维点
        vector<cv::KeyPoint>    keypoints_curr_;    // 当前帧中的关键点
        Mat                     descriptors_curr_;  // 当前帧描述符
        Mat                     descriptors_ref_;   // 参考系描述符
        vector<cv::DMatch>      feature_matches_;   // 特征匹配

        cv::FlannBasedMatcher   matcher_flann_;     // flann matcher

        SE3 T_c_r_estimated_;    // 当前帧的估计位姿
        int num_inliers_;        // pnp中输入点的数量
        int num_lost_;           // 丢失的数量

        // 参数
        int num_of_features_;   // 特征数
        double scale_factor_;   // 图像比例因子
        int level_pyramid_;     // 图像金字塔尺度
        float match_ratio_;     // 良好匹配率
        int max_num_lost_;      // 连续丢失的最大次数
        int min_inliers_;       // 最小内点数

        double key_frame_min_rot;   // 两个关键帧的最小旋转
        double key_frame_min_trans; // 两个关键帧的最小平移

        double  map_point_erase_ratio_; // 地图点删除比例

    public: // 函数
        VisualOdometry();
        ~VisualOdometry();

        bool addFrame(Frame::Ptr frame);      // 添加帧

    protected:
        // 内部操作
        void extractKeyPoints();      // 提取关键点 
        void computeDescriptors();    // 计算描述子
        void featureMatching();       // 在上一帧的特征点3D坐标和当前的特征点2D坐标匹配
        void poseEstimationPnP();     // 姿势估计
        void setRef3DPoints();        // 设置参考帧的3D点
        void addKeyFrame();           // 添加关键帧
        bool checkEstimatedPose();    // 检查估计姿势
        bool checkKeyFrame();         // 检查关键帧

    };
}

#endif // VISUALODOMETRY_H
