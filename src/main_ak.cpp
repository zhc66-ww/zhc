#include "azurekinect.hpp"

#include <iostream>

#include <opencv2/opencv.hpp>

int main() 
{
    my_k4a k4aviewer;

    cv :: Mat color_img , depth_img;

    while(true)
    {
        k4aviewer.Image_to_Cv(color_img , depth_img);

        cv :: imshow("Color", color_img);

        cv :: Mat depth8;      //OpenCV的矩阵

        cv :: normalize(depth_img, depth8, 0, 255, cv :: NORM_MINMAX);      //归一化

        depth8.convertTo(depth8, CV_8U);

        cv :: imshow("Depth", depth8);
         
        k4aviewer.Value_Depth_to_Pcl(cloud);

        std::cout << "点云大小：" << cloud.size() << std::endl;

        k4aviewer.Save_Image(10000 , "output/");

        if(cv :: waitKey(1) == 'q') break;

    }

    return 0;

}

