/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VisionDetector_ref.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "VisionDetector_ref.h"
#include <cmath>
#include <sstream>
#include <iomanip>
// Include YOLO inference
#include "YOLOInference.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

VisionDetector_ref::VisionDetector_ref()
{
    // Initialize own ship state
    m_own_x = 0.0;
    m_own_y = 0.0;
    m_own_heading = 0.0;


    // Initialize camera calibration (defaults)
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

VisionDetector_ref::~VisionDetector_ref()
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

bool VisionDetector_ref::OnNewMail(MOOSMSG_LIST &NewMail)
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

bool VisionDetector_ref::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool VisionDetector_ref::Iterate()
{
    AppCastingMOOSApp::Iterate();

    std::cout << "DEBUG: Iterate() called, queue size: " << m_detection_queue.size() << std::endl;

    // Process detections from queue
    std::lock_guard<std::mutex> lock(m_queue_mutex);

    int published_this_iter = 0;

    while (!m_detection_queue.empty()) {
        VisionDetection det = m_detection_queue.front();
        m_detection_queue.pop();

        std::cout << "DEBUG: Publishing detection to MOOS - x=" << det.x
                  << ", y=" << det.y << ", range=" << det.range << std::endl;

        publishDetection(det);
        published_this_iter++;
    }

    if (published_this_iter > 0) {
        std::cout << "DEBUG: Published " << published_this_iter << " detections this iteration" << std::endl;
    }

    AppCastingMOOSApp::PostReport();
    return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool VisionDetector_ref::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();

    std::cout << "DEBUG: OnStartUp() called" << std::endl;

    //Hard-coded for testing
    m_rtsp_url =
        "rtsp://douglas:Brazil2025!~@192.168.143.14:554/Streaming/Channels/101";

    m_yolo_model_path =
        "/home/douglas/Documents/yolo/yolo11n_openvino_model/yolo11n.xml";

    // Sanity check
    if (m_rtsp_url.empty()) {
        reportRunWarning("rtsp_url is empty (hardcoded)");
        return false;
    }

    if (m_yolo_model_path.empty()) {
        reportRunWarning("yolo_model_path is empty (hardcoded)");
        return false;
    }

    try {
        reportEvent("Loading YOLO model");
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

    reportEvent("Opening camera");
    m_camera.open(m_rtsp_url);
    m_camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!m_camera.isOpened()) {
        reportRunWarning("Failed to open camera");
        return false;
    }

    m_running = true;
    m_vision_thread = std::thread(&VisionDetector_ref::visionLoop, this);

    registerVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: registerVariables()

void VisionDetector_ref::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_HEADING", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool VisionDetector_ref::buildReport()
{
    m_msgs << "================================" << endl;
    m_msgs << "pVisionDetector Status" << endl;
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

//---------------------------------------------------------
// Procedure: visionLoop()
//            Vision processing thread

void VisionDetector_ref::visionLoop()
{
    reportEvent("Vision thread running");

    int frame_width = (int)m_camera.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = (int)m_camera.get(cv::CAP_PROP_FRAME_HEIGHT);

    reportEvent("Camera resolution: " +
                to_string(frame_width) + "x" + to_string(frame_height));

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

        // Debug: Every 30 frames
        if (m_frame_count % 30 == 0) {
            std::cout << "DEBUG: Processing frame " << m_frame_count
                      << ", size: " << frame.cols << "x" << frame.rows << std::endl;
        }

        // ========================================
        // GET DETECTIONS (not just draw!)
        // ========================================
        std::vector<yolo::Detection> yolo_detections = m_yolo->GetDetections(frame);

        std::cout << "DEBUG: GetDetections returned " << yolo_detections.size()
                  << " detections" << std::endl;

        // Process each detection
        for (const auto& yolo_det : yolo_detections) {

            // Debug: Show ALL detections first
            std::cout << "DEBUG: Detection - Class: " << yolo_det.class_id
                      << " (" << yolo_det.className << "), Confidence: "
                      << yolo_det.confidence << std::endl;

            // Filter for target class (boat = 8 in COCO)
            if (yolo_det.class_id != m_target_class) {
                continue;
            }

            // Check exclusion zone
            if (isInExclusionZone(yolo_det.box.x, yolo_det.box.y,
                                 yolo_det.box.x + yolo_det.box.width,
                                 yolo_det.box.y + yolo_det.box.height,
                                 frame_width, frame_height)) {
                std::cout << "DEBUG: Boat in exclusion zone - skipped" << std::endl;
                continue;
            }

            // Estimate distance from bbox height
            double distance = estimateDistance(yolo_det.box.height, frame_height);
            if (distance < 0) {
                std::cout << "DEBUG: Invalid distance - skipped" << std::endl;
                continue;
            }

            // Calculate bearing
            int bbox_center_x = yolo_det.box.x + yolo_det.box.width / 2;
            double bearing = calculateBearing(bbox_center_x, frame_width);

            // Convert to world coordinates
            double bearing_rad = bearing * M_PI / 180.0;
            double target_x = m_own_x + distance * sin(bearing_rad);
            double target_y = m_own_y + distance * cos(bearing_rad);

            std::cout << "DEBUG: BOAT DETECTED! Distance: " << distance
                      << "m, Bearing: " << bearing << "°" << std::endl;

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

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 FPS
    }

    reportEvent("Vision thread stopped");
}

//---------------------------------------------------------
// Procedure: estimateDistance()

double VisionDetector_ref::estimateDistance(int bbox_height_pixels, int image_height)
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

double VisionDetector_ref::calculateBearing(int bbox_center_x, int image_width)
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

bool VisionDetector_ref::isInExclusionZone(int x1, int y1, int x2, int y2,
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

void VisionDetector_ref::publishDetection(const VisionDetection& det)
{
    // Build VISION_REPORT string
    ostringstream oss;
    oss << fixed << setprecision(2);
    oss << "x=" << det.x << ",";
    oss << "y=" << det.y << ",";
    oss << "confidence=" << det.confidence << ",";
    oss << "bearing=" << det.bearing << ",";
    oss << "range=" << det.range << ",";
    oss << "class=" << det.classname << ",";
    oss << "timestamp=" << setprecision(3) << det.timestamp;

    std::cout << "DEBUG: VISION_REPORT = " << oss.str() << std::endl;

    Notify("VISION_REPORT", oss.str());

   // ========================================
    // ALSO PUBLISH NODE_REPORT (for pMarineViewer)
    // ========================================
    ostringstream node_oss;
    node_oss << fixed << setprecision(2);
    node_oss << "NAME=vision_" << m_published_count << ",";
    node_oss << "TYPE=kayak,";
    node_oss << "X=" << det.x << ",";
    node_oss << "Y=" << det.y << ",";
    node_oss << "SPD=0.0,";
    node_oss << "HDG=" << det.bearing << ",";
    node_oss << "TIME=" << setprecision(3) << det.timestamp;

    // Publish NODE_REPORT (pMarineViewer will display this)
    Notify("NODE_REPORT", node_oss.str());

    m_published_count++;

    std::cout << "DEBUG: Notify() called, published_count now: " << m_published_count << std::endl;
}
