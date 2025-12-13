/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT (wrapper for Ultralytics)                   */
/*    FILE: YOLOInference.h                                 */
/*    DATE: December 2024                                   */
/************************************************************/

#ifndef YOLO_INFERENCE_HEADER
#define YOLO_INFERENCE_HEADER

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace yolo {

struct Detection {
    cv::Rect box;
    float confidence;
    int class_id;
    std::string className;
};

class Inference {
public:
    Inference(const std::string& model_path, 
             const cv::Size& model_input_shape,
             float model_confidence_threshold,
             float model_NMS_threshold);
    
    ~Inference();
    
    void RunInference(cv::Mat& frame);
    std::vector<Detection> GetDetections(const cv::Mat& frame);

private:
    void* m_impl;  // Pointer to Ultralytics Inference class
};

} // namespace yolo

#endif
