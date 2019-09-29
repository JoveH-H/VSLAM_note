#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // 格式化字符串
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // 读取彩色和深度图像对和位姿信息，并把位姿从四元数与平移向量转换为变换矩阵
    vector<cv::Mat> colorImgs, depthImgs;   // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;    // 相机位姿

    ifstream fin("./pose.txt");
    if (!fin)
    {
        cerr << "请在有pose.txt的目录下运行此程序" << endl;
        return 1;
    }
    
    // 设置外参变换矩阵T（4*4）
    for (int i = 0; i < 5; i++)
    {
        // boost::format 格式化字符串  拼接出图片文件名
        boost::format fmt("./%s/%d.%s");                                              // 图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1)); // 使用-1读取原始图像

        // 读取位姿数据
        double data[7] = { 0 };
        for (auto& d : data)
            fin >> d;

        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);   // 四元数
        Eigen::Isometry3d T(q);                                     // 变换矩阵T初始化旋转部分
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2])); // T初始化平移向量部分
        poses.push_back(T);
    }

    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;  // 像素坐标系与成像平面之间的原点平移
    double cy = 253.5;
    double fx = 518.0;  // 焦距
    double fy = 519.0;
    double depthScale = 1000.0;

    cout << "正在将图像转换为点云..." << endl;

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < 5; i++)
    {
        cout << "转换图像中: " << i + 1 << endl;
        // 颜色、深度、位姿T
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        // 已知像素坐标，遍历所有像素（u,v）
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue;                             // 为0表示没有测量到
                //像素坐标(u,v,d)计算相机坐标系下坐标 point
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;

                // 相机位姿T计算在世界坐标系下坐标 pointWorld
                Eigen::Vector3d pointWorld = T * point;

                // pcl点 pointT ，x,y,z,b,g,r
                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];

                // push_back(p)放进去一个个点p，构成了点云pointCloud
                pointCloud->points.push_back(p);
            }
    }

    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;

    // 拼接点云，点云是指针形式
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
    return 0;
}
