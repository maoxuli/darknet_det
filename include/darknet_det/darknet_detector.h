#ifndef DARKNET_DETECTOR_H__
#define DARKNET_DETECTOR_H__

// IMPORTANT: Refer to Makefile of Darknet, define below macros accordingly 
// before darknet.h is included. Sugggest to define in CMakeLists.txt.
// #define GPU
// #define CUDNN 
// #define OPENCV 
// #define OPENMP
#include <darknet.h>
#include <opencv2/opencv.hpp>

namespace darknet {

// Bounding box for detected objects 
// center point, with, height, normalized to image size
struct BoundingBox
{
    double probability;
    double x; 
    double y;
    double w;
    double h; 
    int class_id; 

    BoundingBox(double _x = 0, double _y = 0, double _w = 0, double _h = 0)
    : probability(0), x(_x), y(_y), w(_w), h(_h), class_id(0) 
    { 
    }
};

// Utility function from Darknet 
extern "C" image mat_to_image(cv::Mat m);

class Detector
{
public:
    // Using gpu_idx < 0 to indicate GPU is not used  
    Detector(const std::string& cfg, const std::string& weights, int gpu_idx = 0);
    ~Detector();

    void SetThresholds(double probability, double hier = 0.5, double nms = 0.45); 

    // Set object classes to be detected  
    // Empty list indicates all classes 
    void SetClasses(const std::vector<int>& class_ids); 

    std::vector<BoundingBox> Detect(const cv::Mat& m); 

private:
    network* _net;

    // detection thresholds  
    double _probability_th; 
    double _hier_th; 
    double _nms_th; 

    // target classes to be detected 
    // empty for all claases 
    std::vector<int> _class_ids; 
};

} // namespace darknet

#endif // #ifndef DARKNET_DETECTOR_H__ 
