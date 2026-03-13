/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT (using Ultralytics preprocessing)           */
/*    FILE: YOLOInference.cpp                               */
/*    DATE: December 2024                                   */
/************************************************************/

#include "YOLOInference.h"
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace yolo {

class UltralyticsInference {
public:
    UltralyticsInference(const std::string& modelPath,
                         const cv::Size& modelShape,
                         float confThreshold,
                         float iouThreshold);

    std::vector<Detection> runInference(const cv::Mat& input);

private:
    struct LetterboxInfo {
        cv::Mat image;
        float scale;
        int pad_x;
        int pad_y;
    };

    ov::InferRequest inferRequest;
    ov::CompiledModel compiledModel;

    cv::Size modelInputShape;
    float confidenceThreshold;
    float iouThreshold;

    std::vector<std::string> classNames;

    LetterboxInfo letterbox(const cv::Mat& source);
};

UltralyticsInference::UltralyticsInference(const std::string& modelPath,
                                           const cv::Size& modelShape,
                                           float confThreshold,
                                           float iouThreshold)
    : modelInputShape(modelShape),
      confidenceThreshold(confThreshold),
      iouThreshold(iouThreshold)
{
    std::cout << "DEBUG: Loading YOLO with conf=" << confThreshold
              << ", nms=" << iouThreshold << std::endl;

    classNames = {
        "boat", "tree", "vegetation"
    };

    ov::Core core;
    std::shared_ptr<ov::Model> model = core.read_model(modelPath);

    ov::preprocess::PrePostProcessor ppp(model);

    ppp.input().tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);

    ppp.input().preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255.0f, 255.0f, 255.0f});

    ppp.input().model().set_layout("NCHW");
    ppp.output().tensor().set_element_type(ov::element::f32);

    model = ppp.build();

    compiledModel = core.compile_model(model, "CPU");
    inferRequest = compiledModel.create_infer_request();

    std::cout << "DEBUG: modelPath = " << modelPath << std::endl;
    std::cout << "DEBUG: classNames.size() = " << classNames.size() << std::endl;

    std::cout << "DEBUG: input shape = [";
    {
        ov::Shape s = compiledModel.input().get_shape();
        for (size_t i = 0; i < s.size(); ++i) {
            std::cout << s[i] << (i + 1 < s.size() ? ", " : "");
        }
    }
    std::cout << "]" << std::endl;

    std::cout << "DEBUG: output shape = [";
    {
        ov::Shape s = compiledModel.output().get_shape();
        for (size_t i = 0; i < s.size(); ++i) {
            std::cout << s[i] << (i + 1 < s.size() ? ", " : "");
        }
    }
    std::cout << "]" << std::endl;

    std::cout << "DEBUG: YOLO model loaded successfully" << std::endl;
}

UltralyticsInference::LetterboxInfo UltralyticsInference::letterbox(const cv::Mat& source)
{
    LetterboxInfo result;

    const int src_w = source.cols;
    const int src_h = source.rows;
    const int dst_w = modelInputShape.width;
    const int dst_h = modelInputShape.height;

    float scale = std::min(
        static_cast<float>(dst_w) / static_cast<float>(src_w),
        static_cast<float>(dst_h) / static_cast<float>(src_h)
    );

    int new_w = static_cast<int>(std::round(src_w * scale));
    int new_h = static_cast<int>(std::round(src_h * scale));

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(new_w, new_h));

    int pad_x = (dst_w - new_w) / 2;
    int pad_y = (dst_h - new_h) / 2;

    cv::Mat output(dst_h, dst_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(output(cv::Rect(pad_x, pad_y, new_w, new_h)));

    result.image = output;
    result.scale = scale;
    result.pad_x = pad_x;
    result.pad_y = pad_y;

    return result;
}

std::vector<Detection> UltralyticsInference::runInference(const cv::Mat& input)
{
    if (input.empty()) {
        std::cerr << "ERROR: Input image is empty" << std::endl;
        return {};
    }

    LetterboxInfo lb = letterbox(input);
    cv::Mat modelInput = lb.image;

    ov::Tensor input_tensor(
        compiledModel.input().get_element_type(),
        compiledModel.input().get_shape(),
        modelInput.data
    );

    inferRequest.set_input_tensor(input_tensor);
    inferRequest.infer();

    const ov::Tensor& output_tensor = inferRequest.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    const float* detections = output_tensor.data<const float>();

    std::cout << "DEBUG: Output shape: [";
    for (size_t i = 0; i < output_shape.size(); ++i) {
        std::cout << output_shape[i];
        if (i + 1 < output_shape.size()) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;

    if (output_shape.size() != 3) {
        std::cerr << "ERROR: Unexpected output rank: " << output_shape.size() << std::endl;
        return {};
    }

    const int dimensions = static_cast<int>(output_shape[1]);
    const int rows = static_cast<int>(output_shape[2]);
    const int num_classes = dimensions - 4;

    std::cout << "DEBUG: dimensions=" << dimensions
              << ", rows=" << rows
              << ", num_classes=" << num_classes << std::endl;

    if (num_classes <= 0) {
        std::cerr << "ERROR: Invalid number of classes inferred from model output" << std::endl;
        return {};
    }

    // Critical fix: stop immediately if model classes do not match expected classes
    if (num_classes != static_cast<int>(classNames.size())) {
        std::cerr << "ERROR: Model num_classes (" << num_classes
                  << ") != classNames.size() (" << classNames.size() << ")" << std::endl;
        std::cerr << "ERROR: Loaded model does not match configured class list." << std::endl;
        std::cerr << "ERROR: Expected classes: ";
        for (size_t i = 0; i < classNames.size(); ++i) {
            std::cerr << classNames[i] << (i + 1 < classNames.size() ? ", " : "");
        }
        std::cerr << std::endl;
        return {};
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    int detection_count = 0;

    for (int i = 0; i < rows; ++i) {
        float x = detections[0 * rows + i];
        float y = detections[1 * rows + i];
        float w = detections[2 * rows + i];
        float h = detections[3 * rows + i];

        float max_score = 0.0f;
        int best_class = -1;

        for (int c = 0; c < num_classes; ++c) {
            float score = detections[(4 + c) * rows + i];
            if (score > max_score) {
                max_score = score;
                best_class = c;
            }
        }

        if (best_class < 0 || max_score < confidenceThreshold) {
            continue;
        }

        detection_count++;

        float x0 = (x - 0.5f * w - static_cast<float>(lb.pad_x)) / lb.scale;
        float y0 = (y - 0.5f * h - static_cast<float>(lb.pad_y)) / lb.scale;
        float x1 = (x + 0.5f * w - static_cast<float>(lb.pad_x)) / lb.scale;
        float y1 = (y + 0.5f * h - static_cast<float>(lb.pad_y)) / lb.scale;

        int left = static_cast<int>(std::round(x0));
        int top = static_cast<int>(std::round(y0));
        int right = static_cast<int>(std::round(x1));
        int bottom = static_cast<int>(std::round(y1));

        left = std::max(0, std::min(left, input.cols - 1));
        top = std::max(0, std::min(top, input.rows - 1));
        right = std::max(0, std::min(right, input.cols - 1));
        bottom = std::max(0, std::min(bottom, input.rows - 1));

        int width = right - left;
        int height = bottom - top;

        if (width <= 0 || height <= 0) {
            continue;
        }

        boxes.emplace_back(left, top, width, height);
        confidences.push_back(max_score);
        class_ids.push_back(best_class);
    }

    std::cout << "DEBUG: Before NMS: " << detection_count << " detections" << std::endl;

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold, iouThreshold, nms_result);

    std::cout << "DEBUG: After NMS: " << nms_result.size() << " detections" << std::endl;

    std::vector<Detection> result;
    for (int idx : nms_result) {
        Detection det;
        det.box = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        det.className = classNames[det.class_id];

        result.push_back(det);

        std::cout << "DEBUG: Detection - Class: " << det.class_id
                  << " (" << det.className << "), Conf: " << det.confidence << std::endl;
    }

    return result;
}

Inference::Inference(const std::string& model_path,
                     const cv::Size& model_input_shape,
                     float model_confidence_threshold,
                     float model_NMS_threshold)
{
    m_impl = new UltralyticsInference(model_path,
                                      model_input_shape,
                                      model_confidence_threshold,
                                      model_NMS_threshold);
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

        int text_y = std::max(det.box.y - 5, 15);

        cv::putText(frame,
                    label,
                    cv::Point(det.box.x, text_y),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    2);
    }
}

std::vector<Detection> Inference::GetDetections(const cv::Mat& frame)
{
    UltralyticsInference* impl = static_cast<UltralyticsInference*>(m_impl);
    return impl->runInference(frame);
}

} // namespace yolo