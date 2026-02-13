/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Multivisiondetector_ref.cpp                     */
/*    DATE: 2025                                            */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Multivisiondetector_ref.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include "YOLOInference.h"

using namespace std;

//---------------------------------------------------------
// Constructor

Multivisiondetector_ref::Multivisiondetector_ref()
{
    // Initialize own ship state
    m_own_x = 0.0;
    m_own_y = 0.0;
    m_own_heading = 0.0;

    // Initialize camera calibration (defaults from reference)
    m_camera_height = 2.13;
    m_average_boat_height = 2.5;
    m_horizontal_fov = 87.0;
    m_vertical_fov = 46.0;
    m_distance_calibration_factor = 0.5;

    // Initialize detection parameters
    m_confidence_threshold = 0.25f;
    m_nms_threshold = 0.45f;
    m_target_class = 8;  // COCO class for boat

    // Initialize exclusion zone (default: center bottom)
    m_exclusion_x1 = 0.3;
    m_exclusion_y1 = 0.8;
    m_exclusion_x2 = 0.65;
    m_exclusion_y2 = 1.0;

    // Initialize statistics
    m_frame_count = 0;
    m_detection_count = 0;
    m_published_count = 0;
    m_last_fps_time = 0.0;
    m_current_fps = 0.0;

    // Thread control
    m_running = false;
    m_yolo = nullptr;
}

//---------------------------------------------------------
// Destructor

Multivisiondetector_ref::~Multivisiondetector_ref()
{
    // Stop vision thread
    m_running = false;
    if (m_vision_thread.joinable()) {
        m_vision_thread.join();
    }

    // Cleanup camera
    if (m_camera.isOpened()) {
        m_camera.release();
    }

    // Cleanup YOLO
    if (m_yolo != nullptr) {
        delete m_yolo;
    }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Multivisiondetector_ref::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);

    MOOSMSG_LIST::iterator p;
    for(p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();

        if(key == "NAV_X") {
            m_own_x = msg.GetDouble();
        }
        else if(key == "NAV_Y") {
            m_own_y = msg.GetDouble();
        }
        else if(key == "NAV_HEADING") {
            m_own_heading = msg.GetDouble();
        }
    }

    return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Multivisiondetector_ref::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool Multivisiondetector_ref::Iterate()
{
    AppCastingMOOSApp::Iterate();

    // Process detections from queue
    std::lock_guard<std::mutex> lock(m_queue_mutex);

    while (!m_detection_queue.empty()) {
        VisionDetection det = m_detection_queue.front();
        m_detection_queue.pop();

        publishDetection(det);
    }

    AppCastingMOOSApp::PostReport();
    return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool Multivisiondetector_ref::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();

    reportEvent("===========================================");
    reportEvent("pMultivisiondetector_ref Starting");
    reportEvent("===========================================");

    // ========================================================
    // METHOD 1: Try environment variables first (recommended)
    // ========================================================
    const char* env_camera = getenv("CAMERA_URL");
    const char* env_model = getenv("YOLO_MODEL");
    
    if(env_camera) {
        m_rtsp_url = env_camera;
        reportEvent("Using camera URL from environment: " + m_rtsp_url);
    }
    
    if(env_model) {
        m_yolo_model_path = env_model;
        reportEvent("Using YOLO model from environment: " + m_yolo_model_path);
    }

    // ========================================================
    // METHOD 2: Read from simple config file if env not set
    // ========================================================
    if(m_rtsp_url.empty()) {
        // Try reading from ~/.camera_config
        ifstream cam_file(string(getenv("HOME")) + "/.camera_config");
        if(cam_file.is_open()) {
            getline(cam_file, m_rtsp_url);
            cam_file.close();
            reportEvent("Using camera URL from ~/.camera_config: " + m_rtsp_url);
        }
    }
    
    if(m_yolo_model_path.empty()) {
        ifstream model_file(string(getenv("HOME")) + "/.yolo_model");
        if(model_file.is_open()) {
            getline(model_file, m_yolo_model_path);
            model_file.close();
            reportEvent("Using YOLO model from ~/.yolo_model: " + m_yolo_model_path);
        }
    }

    // ========================================================
    // METHOD 3: Read simple parameters from mission file
    // ========================================================
    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
        reportConfigWarning("No config block found for " + GetAppName());

    STRING_LIST::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
        string orig  = *p;
        string line  = *p;
        string param = tolower(biteStringX(line, '='));
        string value = line;

        bool handled = false;
        
        // Only read simple numeric/boolean parameters from mission file
        if(param == "camera_height") {
            m_camera_height = atof(value.c_str());
            handled = true;
        }
        else if(param == "average_boat_height" || param == "object_height") {
            m_average_boat_height = atof(value.c_str());
            handled = true;
        }
        else if(param == "horizontal_fov") {
            m_horizontal_fov = atof(value.c_str());
            handled = true;
        }
        else if(param == "vertical_fov") {
            m_vertical_fov = atof(value.c_str());
            handled = true;
        }
        else if(param == "distance_calibration_factor" || param == "distance_cal_factor") {
            m_distance_calibration_factor = atof(value.c_str());
            handled = true;
        }
        else if(param == "confidence_threshold" || param == "confidence") {
            m_confidence_threshold = atof(value.c_str());
            handled = true;
        }
        else if(param == "nms_threshold") {
            m_nms_threshold = atof(value.c_str());
            handled = true;
        }
        else if(param == "target_class") {
            m_target_class = atoi(value.c_str());
            handled = true;
        }
        else if(param == "apptick" || param == "commstick") {
            // Ignore standard MOOS parameters
            handled = true;
        }

        if(!handled)
            reportUnhandledConfigWarning(orig);
    }

    // ========================================================
    // METHOD 4: Fallback to hardcoded values
    // ========================================================
    if(m_rtsp_url.empty()) {
        m_rtsp_url = "rtsp://douglas:Brazil2025!~@192.168.1.65:554/Streaming/Channels/101";
        reportEvent("WARNING: Using hardcoded camera URL");
    }
    
    if(m_yolo_model_path.empty()) {
        m_yolo_model_path = "/home/douglas/Documents/yolo/yolo11n_openvino_model/yolo11n.xml";
        reportEvent("WARNING: Using hardcoded YOLO model path");
    }

    // ========================================================
    // Final configuration report
    // ========================================================
    reportEvent("===========================================");
    reportEvent("FINAL CONFIGURATION:");
    reportEvent("Camera URL: " + m_rtsp_url);
    reportEvent("YOLO Model: " + m_yolo_model_path);
    reportEvent("Camera Height: " + to_string(m_camera_height) + " m");
    reportEvent("Object Height: " + to_string(m_average_boat_height) + " m");
    reportEvent("Calibration Factor: " + to_string(m_distance_calibration_factor));
    reportEvent("===========================================");

    // Initialize YOLO
    try {
        reportEvent("Loading YOLO model...");
        m_yolo = new yolo::Inference(
            m_yolo_model_path,
            cv::Size(640, 640),
            m_confidence_threshold,
            m_nms_threshold
        );
        reportEvent("YOLO loaded successfully");
    }
    catch (const std::exception& e) {
        reportRunWarning(string("YOLO load failed: ") + e.what());
        return false;
    }

    // Open camera
    reportEvent("Opening camera...");
    m_camera.open(m_rtsp_url);
    m_camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!m_camera.isOpened()) {
        reportRunWarning("Failed to open camera");
        return false;
    }

    // Start vision thread
    m_running = true;
    m_vision_thread = std::thread(&Multivisiondetector_ref::visionLoop, this);

    reportEvent("Vision detector started successfully");
    
    registerVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Multivisiondetector_ref::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_HEADING", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool Multivisiondetector_ref::buildReport()
{
    m_msgs << "================================" << endl;
    m_msgs << "pMultivisiondetector_ref Status" << endl;
    m_msgs << "================================" << endl;

    m_msgs << "Camera URL:    " << m_rtsp_url << endl;
    m_msgs << "Model Path:    " << m_yolo_model_path << endl;
    m_msgs << endl;

    m_msgs << "Own Ship State:" << endl;
    m_msgs << "  X:           " << m_own_x << " m" << endl;
    m_msgs << "  Y:           " << m_own_y << " m" << endl;
    m_msgs << "  Heading:     " << m_own_heading << " deg" << endl;
    m_msgs << endl;

    m_msgs << "Statistics:" << endl;
    m_msgs << "  FPS:         " << fixed << setprecision(1) << m_current_fps << endl;
    m_msgs << "  Detections:  " << m_detection_count << endl;
    m_msgs << "  Published:   " << m_published_count << endl;
    m_msgs << endl;

    m_msgs << "Detection Queue: " << m_detection_queue.size() << endl;

    return true;
}

// Continue in next part...

//---------------------------------------------------------
// Procedure: visionLoop() - Vision processing thread

void Multivisiondetector_ref::visionLoop()
{
    reportEvent("Vision thread running");

    int frame_width = (int)m_camera.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = (int)m_camera.get(cv::CAP_PROP_FRAME_HEIGHT);

    reportEvent("Camera resolution: " +
                to_string(frame_width) + "x" + to_string(frame_height));

    // Always create display window
    cv::namedWindow("Vision Detector", cv::WINDOW_NORMAL);
    cv::resizeWindow("Vision Detector", 1280, 720);
    reportEvent("Video display window created");

    while (m_running) {
        cv::Mat frame;
        m_camera >> frame;

        if (frame.empty()) {
            reportRunWarning("Empty frame, reconnecting...");
            m_camera.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            m_camera.open(m_rtsp_url);
            continue;
        }

        m_frame_count++;

        // Calculate FPS
        double current_time = MOOSTime();
        if (current_time - m_last_fps_time >= 1.0) {
            m_current_fps = (double)m_frame_count / (current_time - m_last_fps_time);
            m_frame_count = 0;
            m_last_fps_time = current_time;
        }

        // Create display frame
        cv::Mat display_frame = frame.clone();

        // Get YOLO detections
        std::vector<yolo::Detection> yolo_detections = m_yolo->GetDetections(frame);

        // Draw ALL detections on display frame
        for (const auto& yolo_det : yolo_detections) {
            cv::Scalar box_color;
            if (yolo_det.class_id == m_target_class) {
                box_color = cv::Scalar(0, 255, 0);  // Green for boats
            } else {
                box_color = cv::Scalar(128, 128, 128);  // Gray for others
            }

            cv::rectangle(display_frame, yolo_det.box, box_color, 2);

            // Draw label with distance and bearing
            int bbox_center_x = yolo_det.box.x + yolo_det.box.width / 2;
            double distance = estimateDistance(yolo_det.box.height, frame_height);
            double bearing = calculateBearing(bbox_center_x, frame_width);
            
            std::string label = yolo_det.className + " " +
                               std::to_string((int)(yolo_det.confidence * 100)) + "%";
            if(distance > 0) {
                label += " | " + std::to_string((int)distance) + "m " + 
                         std::to_string((int)bearing) + "°";
            }
            
            int baseline;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::rectangle(display_frame,
                         cv::Point(yolo_det.box.x, yolo_det.box.y - label_size.height - 5),
                         cv::Point(yolo_det.box.x + label_size.width, yolo_det.box.y),
                         box_color, -1);
            cv::putText(display_frame, label,
                       cv::Point(yolo_det.box.x, yolo_det.box.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }

        // Draw exclusion zone
        int ex_x1 = (int)(m_exclusion_x1 * frame_width);
        int ex_y1 = (int)(m_exclusion_y1 * frame_height);
        int ex_x2 = (int)(m_exclusion_x2 * frame_width);
        int ex_y2 = (int)(m_exclusion_y2 * frame_height);
        cv::rectangle(display_frame,
                     cv::Point(ex_x1, ex_y1),
                     cv::Point(ex_x2, ex_y2),
                     cv::Scalar(0, 0, 255), 2);
        cv::putText(display_frame, "EXCLUSION ZONE",
                   cv::Point(ex_x1 + 5, ex_y1 + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

        // Draw FPS counter
        std::string fps_text = "FPS: " + std::to_string((int)m_current_fps);
        cv::putText(display_frame, fps_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        // Draw detection count
        std::string det_text = "Detections: " + std::to_string(m_detection_count);
        cv::putText(display_frame, det_text, cv::Point(10, 70),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        // Process each detection for MOOS
        for (const auto& yolo_det : yolo_detections) {
            // Filter for target class (boat = 8 in COCO)
            if (yolo_det.class_id != m_target_class) {
                continue;
            }

            // Check exclusion zone
            if (isInExclusionZone(yolo_det.box.x, yolo_det.box.y,
                                 yolo_det.box.x + yolo_det.box.width,
                                 yolo_det.box.y + yolo_det.box.height,
                                 frame_width, frame_height)) {
                continue;
            }

            // Estimate distance from bbox height
            double distance = estimateDistance(yolo_det.box.height, frame_height);
            if (distance < 0) {
                continue;
            }

            // Calculate bearing
            int bbox_center_x = yolo_det.box.x + yolo_det.box.width / 2;
            double bearing = calculateBearing(bbox_center_x, frame_width);

            // Convert to world coordinates
            double bearing_rad = bearing * M_PI / 180.0;
            double target_x = m_own_x + distance * sin(bearing_rad);
            double target_y = m_own_y + distance * cos(bearing_rad);

            // Create VisionDetection
            VisionDetection det;
            det.x = target_x;
            det.y = target_y;
            det.confidence = yolo_det.confidence;
            det.bearing = bearing;
            det.range = distance;
            det.classname = yolo_det.className;
            det.timestamp = current_time;
            det.bbox_x1 = yolo_det.box.x;
            det.bbox_y1 = yolo_det.box.y;
            det.bbox_x2 = yolo_det.box.x + yolo_det.box.width;
            det.bbox_y2 = yolo_det.box.y + yolo_det.box.height;

            // Add to queue (thread-safe)
            {
                std::lock_guard<std::mutex> lock(m_queue_mutex);
                m_detection_queue.push(det);
            }

            m_detection_count++;
        }

        // Always display the frame
        cv::imshow("Vision Detector", display_frame);

        // Handle keyboard input (ESC to quit)
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC key
            m_running = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 FPS
    }

    cv::destroyWindow("Vision Detector");
    reportEvent("Vision thread stopped");
}

//---------------------------------------------------------
// Procedure: estimateDistance()

double Multivisiondetector_ref::estimateDistance(int bbox_height_pixels, int image_height)
{
    if (bbox_height_pixels < 10) {
        return -1.0;  // Invalid
    }

    double vertical_fov_rad = m_vertical_fov * M_PI / 180.0;
    double focal_length_pixels = (image_height / 2.0) / tan(vertical_fov_rad / 2.0);
    double distance = (m_average_boat_height * focal_length_pixels) / bbox_height_pixels;
    distance *= m_distance_calibration_factor;

    return distance;
}

//---------------------------------------------------------
// Procedure: calculateBearing()

double Multivisiondetector_ref::calculateBearing(int bbox_center_x, int image_width)
{
    double image_center_x = image_width / 2.0;
    double offset_x = bbox_center_x - image_center_x;
    double max_angle = m_horizontal_fov / 2.0;
    double normalized_offset = offset_x / (image_width / 2.0);
    double relative_bearing = normalized_offset * max_angle;

    // Convert to true bearing
    double true_bearing = m_own_heading + relative_bearing;

    // Normalize to 0-360
    while (true_bearing < 0) true_bearing += 360.0;
    while (true_bearing >= 360.0) true_bearing -= 360.0;

    return true_bearing;
}

//---------------------------------------------------------
// Procedure: isInExclusionZone()

bool Multivisiondetector_ref::isInExclusionZone(int x1, int y1, int x2, int y2,
                                       int img_width, int img_height)
{
    int ex_x1 = (int)(m_exclusion_x1 * img_width);
    int ex_y1 = (int)(m_exclusion_y1 * img_height);
    int ex_x2 = (int)(m_exclusion_x2 * img_width);
    int ex_y2 = (int)(m_exclusion_y2 * img_height);

    int bbox_center_x = (x1 + x2) / 2;
    int bbox_center_y = (y1 + y2) / 2;

    return (bbox_center_x >= ex_x1 && bbox_center_x <= ex_x2 &&
            bbox_center_y >= ex_y1 && bbox_center_y <= ex_y2);
}

//---------------------------------------------------------
// Procedure: publishDetection()

void Multivisiondetector_ref::publishDetection(const VisionDetection& det)
{
    // Build VISION_REPORT string
    ostringstream oss;
    oss << fixed << setprecision(2);
    oss << "class=" << det.classname << ",";
    oss << "confidence=" << setprecision(3) << det.confidence << ",";
    oss << "bearing_deg=" << setprecision(1) << det.bearing << ",";
    oss << "range_m=" << setprecision(1) << det.range << ",";
    oss << "time=" << setprecision(3) << det.timestamp;

    Notify("VISION_REPORT", oss.str());
    m_published_count++;
}
