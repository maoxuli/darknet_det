#ifndef IMAGE_DETECTOR_H__
#define IMAGE_DETECTOR_H__

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

#include <darknet_det/ros_parameter.hpp>
#include <darknet_det/darknet_detector.h>

#include <darknet_det/BoundingBox.h>
#include <darknet_det/BoundingBoxes.h>

namespace darknet_det {

class ImageDetector 
{
public:
    ImageDetector(const ros::NodeHandle& nh = ros::NodeHandle(), 
                  const ros::NodeHandle& private_nh = ros::NodeHandle("~"));

private:
    void ImageCallback(const sensor_msgs::Image::ConstPtr& im);

    void PublishDetection(const std::vector<darknet::BoundingBox>& bbs, 
                          const std_msgs::Header& header);

#ifdef VISUALIZE_DETECTION_ 
    void PublishOverlay(const std::vector<darknet::BoundingBox>& bbs, 
                        cv::Mat& im, const std_msgs::Header& header);
#endif 

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh; 

    ros::Subscriber _image_sub;
    ros::Publisher _det_pub;

#ifdef FORWARD_IMAGE_
    ros::Publisher _image_pub;
#endif 

#ifdef VISUALIZE_DETECTION_
    ros::Publisher _overlay_pub;
#endif 

    std::shared_ptr<darknet::Detector> _det;
    std::vector<std::string> _class_names; 
};

} // namespace darknet_det 

#endif // #ifndef IMAGE_DETECTOR_H__
