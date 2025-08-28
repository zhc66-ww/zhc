#ifndef REALSENSE_HPP
#define REALSENSE_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <iostream>
#include <unistd.h>
#include <memory>
#include <string>

#include "yolo.hpp"

#define COUT_RED_START       std :: cout << "\033[1;31m";
#define COUT_GREEN_START     std :: cout << "\033[1;32m";
#define COUT_YELLOW_START    std :: cout << "\033[1;33m";
#define COUT_BLUE_START      std :: cout << "\033[1;34m";
#define COUT_PURPLE_START    std :: cout << "\033[1;35m";
#define COUT_CYAN_START      std :: cout << "\033[1;36m";
#define COUT_WHITE_START     std :: cout << "\033[1;37m";
#define COUT_COLOR_END       std :: cout << "\033[0m";

#define MIN_DISTANCE 2.0;

class my_realsense
{
    private:
        rs2 :: pipeline pipe;   //RealSense 管道，处理从相机获取的帧数据
        rs2 :: config cfg;     //配置对象
        rs2 :: pipeline_profile profile;    //存储管道的配置文件
        rs2 :: frameset frameset;   //存储捕获的多个图像
        cv :: Mat image_rs_color , image_rs_depth;  //彩色图、深度图
        cv :: Mat image_rs_infrared_left , image_rs_infrared_right;     //左右红外图像
        cv :: Mat mask;     //掩码图像，可对特定区域做处理

        int frame_count = 0;    //捕获的帧数
        
        void Configuration_Default();   //配置相机模式为默认模式

        void Configuration_Infrared_Only();     //配置相机模式为仅红外模式

        void Configuration_RGBD();   //配置相机模式为RGBD模式

    public: 
        rs2_intrinsics intrinsics_depth;    //深度图像内参
        rs2_intrinsics intrinsics_color;    //彩色图像内参
        rs2_intrinsics intrinsics_infrared;     //红外图像内参

        static my_realsense Create_Default();   //调用Configuration_Default()，并返回一个类的实例

        static my_realsense Create_Infrared_Only();   //调用Configuration_Infrared_Only()，并返回一个类的实例

        static my_realsense Create_RGBD();   // 调用Configuration

        void Configuration();   //配置相机
    
        void Image_to_Cv(cv :: Mat &image_cv_color , cv :: Mat &image_cv_depth);    //将彩色图和深度图转换为OpenCV模式

        void Color_to_Cv(cv :: Mat &image_cv_color);    //将彩色图转换为OpenCV模式

        void Infrared_to_Cv(cv :: Mat &image_cv_infrared_left , cv :: Mat &image_cv_infrared_right);    //将左右红外图转换为OpenCV模式

        void Color_With_Mask(cv :: Mat &image_cv_color , yolo :: BoxArray objs);    //彩图＋YOLO检测框

        void Depth_With_Mask(cv :: Mat &image_cv_depth , yolo :: BoxArray objs);    //深度图＋YOLO检测框

        void Value_Depth_to_Pcl(pcl :: PointCloud<pcl :: PointXYZ> &cloud);     //将深度图转换成点云

        void Value_Mask_to_Pcl(pcl :: PointCloud<pcl :: PointXYZ> &cloud);      //将掩码图转换成点云

        void Save_Image(int amount , std :: string output_dir);     //保存图像

        my_realsense() = default;

        ~my_realsense() = default;

};

#endif // REALSENSE_HPP