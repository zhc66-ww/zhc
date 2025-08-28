#ifndef AZUREKINECT_HPP
#define AZUREKINECT_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <k4a/k4a.hpp>

#include <iostream>     //引入输入输出库
#include <unistd.h>     //能提供系统和调用函数
#include <memory>       //提供智能指针
#include <string>       //提供字符串类，方便处理

#include "yolo.hpp"

#define COUT_RED_START      std :: cout << "\033[1;31m"     //定义一个宏，在终端输出红色文字 （\033是ASCII的ESC，[1;31m是控制码，1是加粗，31是红色；std::cout << "\033[1;31m" 就会让之后输出的文字变为红色（加粗））
#define COUT_GREEN_START    std :: cout << "\033[1;32m"     //定义一个宏，在终端输出绿色文字
#define COUT_YELLOW_START   std :: cout << "\033[1;33m"     //定义一个宏，在终端输出黄色文字
#define COUT_BLUE_START     std :: cout << "\033[1;34m"     //定义一个宏，在终端输出蓝色文字
#define COUT_PURPLE_START   std :: cout << "\033[1;35m"     //定义一个宏，在终端输出紫色文字
#define COUT_CYAN_START     std :: cout << "\033[1;36m"     //定义一个宏，在终端输出青色文字
#define COUT_WHITE_START    std :: cout << "\033[1;37m"     //定义一个宏，在终端输出白色文字
#define COUT_COLOR_END      std :: cout << "\033[0m"        //重置宏，恢复正常颜色和样式

#define MIN_DISTANCE 2.0    //MIN_DISTANCE = 2.0 

class my_k4a
{
    private:
        k4a :: device device;   //k4a::device ：找 k4a 命名空间里的 device 类， device等同于SDK中的device类，能打开和关闭相机
        k4a_device_configuration_t config;  //配置设备的分辨率、帧率等
        k4a :: capture capture;     //一次抓取的图像
        uint32_t device_count;   //设备数量
        k4a :: image image_k4a_color , image_k4a_depth , image_k4a_infrared;    //彩色、深度和红外图像（图像数据）
        k4a :: image image_k4a_depth_to_color;  //深度图转到彩色图坐标系
        k4a_calibration_camera_t depth_intrinsics;  //相机深度内参
        k4a_calibration_camera_t color_intrinsics;  //相机彩色内参
        k4a :: calibration k4a_Calibration;    //相机标定信息
        k4a :: transformation k4a_Transformation;    //深度与彩色图的空间变换
        int frame_count = 0;   //记录抓取帧数
    
    public:
        bool Open();    //打开相机

        void Installed_Count();     //检测设备数量

        void Configuration();   //设置相机参数

        void Image_to_Cv(cv :: Mat &image_cv_color , cv :: Mat &image_cv_depth);    //将捕获到的彩色图像和深度图像转换成OpenCV的Mat格式

        void Color_to_Cv(cv :: Mat &image_cv_color);    //只转换彩色图像

        void Depth_to_Cv(cv :: Mat &image_cv_depth);    //只转换深度图像
    
        void Color_With_Mask(cv :: Mat &image_cv_color , yolo :: BoxArray &objs);   //在彩色图像上叠加YOLO检测到的边框（mask）

        void Depth_With_Mask(cv :: Mat &image_cv_depth , yolo :: BoxArray &objs);   //在深度图像上  

        void Value_Mask_to_Pcl(pcl :: PointCloud<pcl :: PointXYZ> &cloud , yolo :: BoxArray &objs);     //将YOLO探测到的区域的深度值转换成点云

        void Value_Depth_to_Pcl(pcl :: PointCloud<pcl :: PointXYZ> &cloud );    //将深度图转换成点云

        void Save_Image(int amount , std :: string output_dir);     //将图像保存到磁盘

        my_k4a()    //构造函数
        {
            Installed_Count();      //检查设备

            if(Open())              //打开相机
            {
                Configuration();    //设置参数       
            }
        }

        ~my_k4a()    //析构函数   释放资源，避免占内存
        {
            image_k4a_depth.reset();

            image_k4a_color.reset();

            capture.reset();

            device.close();
        }
};

#endif