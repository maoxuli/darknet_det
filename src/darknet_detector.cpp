#include <darknet_det/darknet_detector.h>

namespace darknet {

Detector::Detector(const std::string& cfg, const std::string& weights, int gpu_idx)
: _net(nullptr)
, _probability_th(0.5) 
, _hier_th(0.5)
, _nms_th(0.45)
{
    if (gpu_idx >= 0)
    {
        printf("darknet using GPU: %d\r\n", gpu_idx);  
        cuda_set_device(gpu_idx);
    }

    printf("darknet loading network: %s, %s\r\n", cfg.c_str(), weights.c_str());
    _net = load_network(const_cast<char*>(cfg.c_str()), const_cast<char*>(weights.c_str()), 0);
    assert(_net); 
    set_batch_network(_net, 1);
    srand(time(0));
    
    printf("darknet loaded network (batch:channel:width:height): %d:%d:%d:%d\r\n", 
            _net->batch, _net->c, _net->w, _net->h); 
}

Detector::~Detector()
{
    if (_net) free_network(_net); 
}

void Detector::SetThresholds(double probability, double hier, double nms)
{
    _probability_th = probability; 
    _hier_th = hier; 
    _nms_th = nms; 
}

// Set object classes to be detected  
// Empty list indicates all classes 
void Detector::SetClasses(const std::vector<int>& class_ids)
{
    _class_ids = class_ids; 
}

std::vector<BoundingBox> Detector::Detect(const cv::Mat& m)
{
    assert(_net); 
    assert(_net->batch >= 1); 

    image im = mat_to_image(m); 
    image resized = letterbox_image(im, _net->w, _net->h); 

    network_predict(_net, resized.data); 

    layer l = _net->layers[_net->n - 1]; 
    int classes = l.classes;

    int nboxes = 0;
    detection* det = get_network_boxes(_net, im.w, im.h, _probability_th, _hier_th, 0, 1, &nboxes);
    do_nms_sort(det, nboxes, classes, _nms_th);

    std::vector<BoundingBox> bbs; 
    for (int i = 0; i < nboxes; i++)
    {
        int iclass = max_index(det[i].prob, classes);
        if (_class_ids.empty() || 
           std::find(_class_ids.begin(), _class_ids.end(), iclass) != _class_ids.end())
        {
            float probability = det[i].prob[iclass];
            if (probability >= _probability_th)
            {
                box b = det[i].bbox;
                BoundingBox bb(b.x, b.y, b.w, b.h);
                bb.probability = probability;
                bb.class_id = iclass;
                bbs.push_back(bb);
            }
        }
    }

    free_detections(det, nboxes);
    free_image(resized);
    free_image(im);
    return bbs; 
}  

} // namespace darknet 
