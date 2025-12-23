/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT (using Ultralytics preprocessing)           */
/*    FILE: YOLOInference.cpp                               */
/*    DATE: December 2024                                   */
/************************************************************/

#include "YOLOInference.h"
#include <openvino/openvino.hpp>
#include <iostream>

namespace yolo {

class UltralyticsInference {
public:
    UltralyticsInference(const std::string& modelPath,
                        const cv::Size& modelShape,
                        float confThreshold,
                        float iouThreshold);
    
    std::vector<Detection> runInference(const cv::Mat& input);

private:
    ov::InferRequest inferRequest;
    ov::CompiledModel compiledModel;
    
    cv::Size modelInputShape;
    float confidenceThreshold;
    float iouThreshold;
    
    std::vector<std::string> classNames;
    
    cv::Mat letterbox(const cv::Mat& source);
};

UltralyticsInference::UltralyticsInference(const std::string& modelPath,
                                          const cv::Size& modelShape,
                                          float confThreshold,
                                          float iouThreshold)
    : modelInputShape(modelShape)
    , confidenceThreshold(confThreshold)
    , iouThreshold(iouThreshold)
{
    std::cout << "DEBUG: Loading YOLO with conf=" << confThreshold << ", nms=" << iouThreshold << std::endl;
    
    // COCO class names
    classNames = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
        "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
        "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };
    
    // Load model with preprocessing (EXACTLY like Ultralytics)
    ov::Core core;
    std::shared_ptr<ov::Model> model = core.read_model(modelPath);
    
    // Configure preprocessing
    ov::preprocess::PrePostProcessor ppp(model);
    
    // Input preprocessing
    ppp.input().tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    
    ppp.input().preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255.0f, 255.0f, 255.0f});
    
    ppp.input().model().set_layout("NCHW");
    
    // Output
    ppp.output().tensor().set_element_type(ov::element::f32);
    
    model = ppp.build();
    
    compiledModel = core.compile_model(model, "CPU");
    inferRequest = compiledModel.create_infer_request();
    
    std::cout << "DEBUG: YOLO model loaded successfully" << std::endl;
}

cv::Mat UltralyticsInference::letterbox(const cv::Mat& source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = std::max(col, row);
    
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    
    return result;
}

std::vector<Detection> UltralyticsInference::runInference(const cv::Mat& input)
{
    // Letterbox resize (like Ultralytics does)
    cv::Mat modelInput = letterbox(input);

    // Calculate scale factors BEFORE resize (considering letterbox padding)
    int max_size = std::max(input.cols, input.rows);
    float scale = (float)modelInputShape.width / max_size;
    float x_offset = 0;  // No offset in X for letterbox
    float y_offset = 0;  // No offset in Y for letterbox

    cv::resize(modelInput, modelInput, modelInputShape);

    // Create input tensor
    ov::Tensor input_tensor = ov::Tensor(compiledModel.input().get_element_type(),
                                         compiledModel.input().get_shape(),
                                         modelInput.data);

    inferRequest.set_input_tensor(input_tensor);
    inferRequest.infer();

    // Get output
    const ov::Tensor& output_tensor = inferRequest.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    float* detections = output_tensor.data<float>();

    std::cout << "DEBUG: Output shape: [" << output_shape[0] << ", "
              << output_shape[1] << ", " << output_shape[2] << "]" << std::endl;

    // Parse detections
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    const int dimensions = output_shape[1];  // Usually 84 (4 box + 80 classes)
    const int rows = output_shape[2];        // Number of detections
    
    int detection_count = 0;
    
    for (int i = 0; i < rows; ++i) {
        float* detection = &detections[i];
        
        // Get class scores (skip first 4 box coordinates)
        float* classes_scores = &detection[4 * rows];
        
        // Find best class
        float max_score = 0.0f;
        int best_class = 0;
        
        for (int c = 0; c < 80; c++) {
            float score = classes_scores[c * rows];
            if (score > max_score) {
                max_score = score;
                best_class = c;
            }
        }
        
        if (max_score > confidenceThreshold) {
            detection_count++;

            float x = detection[0 * rows];
            float y = detection[1 * rows];
            float w = detection[2 * rows];
            float h = detection[3 * rows];

            // Convert from model coordinates (640x640) to original image coordinates
            // considering letterbox padding
            int left = int((x - 0.5 * w - x_offset) / scale);
            int top = int((y - 0.5 * h - y_offset) / scale);
            int width = int(w / scale);
            int height = int(h / scale);

            // Clamp to image boundaries
            left = std::max(0, std::min(left, input.cols - 1));
            top = std::max(0, std::min(top, input.rows - 1));
            width = std::min(width, input.cols - left);
            height = std::min(height, input.rows - top);

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(max_score);
            class_ids.push_back(best_class);
        }
    }
    
    std::cout << "DEBUG: Before NMS: " << detection_count << " detections" << std::endl;
    
    // Apply NMS
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold, iouThreshold, nms_result);
    
    std::cout << "DEBUG: After NMS: " << nms_result.size() << " detections" << std::endl;
    
    // Build final detections
    std::vector<Detection> result;
    for (int idx : nms_result) {
        Detection det;
        det.box = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        det.className = (class_ids[idx] < classNames.size()) ? 
                       classNames[class_ids[idx]] : "unknown";
        result.push_back(det);
        
        std::cout << "DEBUG: Detection - Class: " << det.class_id 
                  << " (" << det.className << "), Conf: " << det.confidence << std::endl;
    }
    
    return result;
}

// Wrapper implementation
Inference::Inference(const std::string& model_path,
                    const cv::Size& model_input_shape,
                    float model_confidence_threshold,
                    float model_NMS_threshold)
{
    m_impl = new UltralyticsInference(model_path, model_input_shape,
                                     model_confidence_threshold, model_NMS_threshold);
}

Inference::~Inference()
{
    delete static_cast<UltralyticsInference*>(m_impl);
}

void Inference::RunInference(cv::Mat& frame)
{
    auto detections = GetDetections(frame);
    
    for (const auto& det : detections) {
        cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
        
        std::string label = det.className + " " + 
                          std::to_string(det.confidence).substr(0, 4);
        
        cv::putText(frame, label,
                   cv::Point(det.box.x, det.box.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5,
                   cv::Scalar(0, 255, 0), 2);
    }
}

std::vector<Detection> Inference::GetDetections(const cv::Mat& frame)
{
    UltralyticsInference* impl = static_cast<UltralyticsInference*>(m_impl);
    return impl->runInference(frame);
}

} // namespace yolo
