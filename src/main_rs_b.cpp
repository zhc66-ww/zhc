#include "realsense.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int main()
{
    try
    {
        // 1) 初始化（默认：彩色+深度）
        my_realsense myrealsense1 = my_realsense::Create_RGBD();

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0); // 显示坐标系
        viewer->initCameraParameters();
        
        //设置点云的初始显示参数
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");  // 设置点云点大小为3
        while (true)
        {
            // 2) 抓取一帧彩色与深度（仅用于显示；点云不依赖这一步）
            cv::Mat color, depth;
            myrealsense1.Image_to_Cv(color, depth); // 注意：此函数内部做了对齐，仅用于可视化

            if (!color.empty())
                cv::imshow("Color", color);
            if (!depth.empty())
                cv::imshow("Depth (8-bit)", depth);

            // 3) 生成点云（函数内部会自己获取一帧深度，不受上一步对齐影响）
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            myrealsense1.Value_Depth_to_Pcl(*cloud);

            if (!cloud->empty())
            {
                if (pcl::io::savePCDFileBinary("frame_cloud.pcd", *cloud) == 0)
                {
                    //删除之前的点云
                    viewer->removePointCloud("cloud");

                    //添加新的点云
                    viewer->addPointCloud(cloud, "cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                }
                pcl::io::savePCDFileBinary("frame_cloud.pcd", *cloud);
                std::cout <<"Save point cloud:frame_cloud.pcd (" <<cloud->size()<<" points)" << std::endl;
                    
            }
            else
            {
                std::cerr << "Point cloud is empty.\n";
            }

            // 4) 显示窗口等待一会儿

            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) // q or Esc
            {
                break;
            }
            viewer->spinOnce(10); // 刷新显示
            // ==== 如需红外图保存，换成红外配置：====
            // RealSense ir = RealSense::Create_Infrared_Only();
            // for (int i = 0; i < 5; ++i) {
            //     ir.Save_Image(5, "./out");   // 多次调用直到 amount 达到
            //     usleep(50000);
            // }
        }
        return EXIT_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Std exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}