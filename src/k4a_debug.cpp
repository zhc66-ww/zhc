#include "azurekinect.hpp"
#include "myinfer.hpp"


int main(int argc, char const *argv[])
{
    my_k4a k4a;
    Yolo yolo;
    std::string engine_v8_seg = "/home/right/RIGHT-Infer/workspace/Basket&Target/best.engine";
    yolo::BoxArray objs;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cv::Mat image_color, image_depth;
    int index = 0;

    while(1)
    {   
        k4a.Color_to_Cv(image_color);
        yolo.Yolov8_Seg_Enable(engine_v8_seg);

        yolo.Single_Inference(image_color, objs);

        // if(index < 500)
        // {
        //     pcl::io::savePLYFileASCII("/home/right/RIGHT-Infer/datasets/ply/test/basket" + std::to_string(index++) + ".ply", cloud);
        //     std::cout << "Basket" << index << " Save Success!" << std::endl; 
        //     usleep(1000);
        // }
        // else
        // {
        //     std::cout << "Pointcloud Save Over!" << std::endl; 
        // }
        k4a.Color_With_Mask(image_color, objs);
        // k4a.Depth_With_Mask(image_depth, objs);     

        // cv::imshow("Seg Depth Image", image_depth);
        cv::imshow("Seg Color Image", image_color);
        if (cv::waitKey(1) == 27) break;

    }
    return 0;
}