
#include <ros/ros.h>
#include <darknet_det/image_detector.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "darknet_image_det");
    ros::NodeHandle nh, private_nh("~");
    darknet_det::ImageDetector det(nh, private_nh);
    ros::spin(); 
    return 0;
}
