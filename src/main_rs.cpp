#include <realsense.hpp>
#include <opencv2/opencv.hpp>
#include "realsense.hpp"

int main()
{
    my_realsense rs = my_realsense::Create_RGBD();



    pcl :: PointCloud<pcl :: PointXYZ> :: Ptr cloud (new pcl :: PointCloud<pcl :: PointXYZ>);

    while(true)
    {
        cv :: Mat color_img , depth_img;

        rs.Image_to_Cv(color_img, depth_img);

        cv :: imshow("Color Image" , color_img);

        cv :: Mat depth8;

        cv :: normalize(depth_img , depth8 , 0 , 255 , cv :: NORM_MINMAX);

        depth8.convertTo(depth8 , CV_8U);

        cv :: imshow("Depth Image" , depth8);

        cloud -> clear();

        rs.Value_Depth_to_Pcl(*cloud);

        pcl::io::savePLYFileBinary("cloud.ply", *cloud);

        cv :: waitKey(10);
    }

    return 0;

}