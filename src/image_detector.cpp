#include <darknet_det/image_detector.h>
#include <cv_bridge/cv_bridge.h>

namespace darknet_det {

ImageDetector::ImageDetector(const ros::NodeHandle& nh, 
                             const ros::NodeHandle& private_nh)
: _nh(nh)
, _private_nh(private_nh)
{
    // create darknet detector
    int gpu_idx = 0;
    std::string cfg_file = "yolov3.cfg";
    std::string weights_file = "yolov3.weights";
    ros::LoadParam(_private_nh, "gpu_index", gpu_idx); // <0 for no gpu
    ros::LoadParam(_private_nh, "cfg_file", cfg_file);
    ros::LoadParam(_private_nh, "weights_file", weights_file);
    _det.reset(new darknet::Detector(cfg_file, weights_file, gpu_idx));
    assert(_det);
    ROS_INFO("Darknet detector initialized.");

    // detection thresholds 
    double probability_th = 0.5;
    double hier_th = 0.5;
    double nms_th = 0.45;
    ros::LoadParam(_private_nh, "probability_threshold", probability_th);
    ros::LoadParam(_private_nh, "hier_threshold", hier_th);
    ros::LoadParam(_private_nh, "nms_threshold", nms_th);
    _det->SetThresholds(probability_th, hier_th, nms_th); 

    // object classes to be detected 
    // empty indicates all classes 
    // for coco data set, id 0 is for 'person'
    std::vector<int> class_ids; 
    //class_ids.push_back(0); 
    ros::LoadParam(_private_nh, "class_ids", class_ids);
    _det->SetClasses(class_ids); 

    // class names are only used for visualization 
    std::string names_file; 
    ros::LoadParam(_private_nh, "names_file", names_file);
    if (!names_file.empty())
    {
        std::ifstream ifs(names_file);
        std::string line;
        while (std::getline(ifs, line)) 
        {
            if (!line.empty()) _class_names.push_back(line);
        }
        ROS_INFO_STREAM("Loaded class names: " << _class_names.size()); 
    } 

    std::string image_topic = "image";
    _image_sub = _nh.subscribe(image_topic, 3, &ImageDetector::ImageCallback, this);
    assert(_image_sub);
    ROS_INFO_STREAM("Subscribe image topic: " << image_topic);
     
    std::string det_topic = "det";
    _det_pub = _nh.advertise<darknet_det::BoundingBoxes>(det_topic, 3);
    assert(_det_pub);
    ROS_INFO_STREAM("Advertised detection topic: " << det_topic);

#ifdef FORWARD_IMAGE_ 
    std::string det_image_topic = "det_image";
    _image_pub = _nh.advertise<sensor_msgs::Image>(det_image_topic, 3);
    assert(_image_pub);
    ROS_INFO_STREAM("Advertised detection image topic: " << det_image_topic);
#endif 

#ifdef VISUALIZE_DETECTION_ 
    std::string overlay_topic = "det_overlay";
    _overlay_pub = _nh.advertise<sensor_msgs::Image>(overlay_topic, 3);
    assert(_overlay_pub);
    ROS_INFO_STREAM("Advertised detection overlay topic: " << overlay_topic);
#endif 
}

void ImageDetector::ImageCallback(const sensor_msgs::Image::ConstPtr& im)
{
    try
    {
        cv_bridge::CvImageConstPtr cvi_ptr = 
        cv_bridge::toCvShare(im, sensor_msgs::image_encodings::BGR8); 
        std::vector<darknet::BoundingBox> bbs = _det->Detect(cvi_ptr->image);
        PublishDetection(bbs, im->header);

    #ifdef FORWARD_IMAGE_ 
        if(_image_pub.getNumSubscribers() > 0)
        {
            assert(_image_pub); 
            _image_pub.publish(im);
        }
    #endif 

    #ifdef VISUALIZE_DETECTION_ 
        if(_overlay_pub.getNumSubscribers() > 0)
        {
            cv::Mat overlay = cvi_ptr->image.clone();
            PublishOverlay(bbs, overlay, im->header);
        }
    #endif 
    }
    catch(const ros::Exception& ex)
    {
        ROS_WARN("ROS excpetion: %s", ex.what());
    }
    catch(const cv::Exception& ex)
    {
        ROS_WARN("OpenCV exception: %s", ex.what());
    }
    catch(const std::exception& ex)
    {
        ROS_WARN("std exception: %s", ex.what()); 
    }
    catch(...)
    {
        ROS_WARN("Unknown exception");
    }
}

void ImageDetector::PublishDetection(const std::vector<darknet::BoundingBox>& bbs, 
                                     const std_msgs::Header& header)
{
    darknet_det::BoundingBoxes det; 
    det.header = header; 
    for (auto it = bbs.begin(); it != bbs.end(); ++it)
    {
        darknet_det::BoundingBox b; 
        b.x = it->x;
        b.y = it->y; 
        b.w = it->w; 
        b.h = it->h; 
        b.probability = it->probability;
        b.class_id = it->class_id; 
        det.boxes.push_back(b); 
    }
    assert(_det_pub); 
    _det_pub.publish(det); 
}

#ifdef VISUALIZE_DETECTION_ 
void ImageDetector::PublishOverlay(const std::vector<darknet::BoundingBox>& bbs, 
                                   cv::Mat& im, const std_msgs::Header& header)
{
    for(auto it = bbs.begin(); it != bbs.end(); ++it)
    {
        cv::Rect rc((it->x - it->w/2.0) * im.cols, (it->y - it->h/2.0) * im.rows, 
                    it->w * im.cols, it->h * im.rows);
        cv::rectangle(im, rc, cv::Scalar(255, 0, 0), 2, 8, 0);
        std::string label = cv::format("%.2f",it->probability);
        if (it->class_id < _class_names.size()) label = _class_names[it->class_id] + " " + label;
        cv::putText(im, label, rc.tl() + cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, cv::Scalar(255, 0, 0));
    }

    cv_bridge::CvImage cvi;
    cvi.header = header;
    cvi.image = im;
    cvi.encoding = sensor_msgs::image_encodings::BGR8;
    assert(_overlay_pub); 
    _overlay_pub.publish(cvi.toImageMsg());
}
#endif 

} // namespace darknet_det 
