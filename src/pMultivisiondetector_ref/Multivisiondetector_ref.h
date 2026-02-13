/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Multivisiondetector_ref.h                       */
/*    DATE: 2025                                            */
/*                                                          */
/* Clean vision detector - based on working reference       */
/* Uses environment variables to avoid config parsing      */
/************************************************************/

#ifndef Multivisiondetector_ref_HEADER
#define Multivisiondetector_ref_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

// Forward declaration for YOLO
namespace yolo {
    class Inference;
    struct Detection;
}

//================================================================
// Vision Detection Structure
//================================================================
struct VisionDetection {
    double x;                // World X coordinate
    double y;                // World Y coordinate
    double confidence;       // Detection confidence
    double bearing;          // Bearing (degrees)
    double range;            // Estimated range (meters)
    std::string classname;   // Object class
    double timestamp;        // MOOSTIME
    
    // Bounding box
    int bbox_x1, bbox_y1, bbox_x2, bbox_y2;
};

//================================================================
// Main Application Class
//================================================================
class Multivisiondetector_ref : public AppCastingMOOSApp
{
 public:
   Multivisiondetector_ref();
   ~Multivisiondetector_ref();

 protected: // Standard MOOSApp functions
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // AppCastingMOOSApp function
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration
   std::string m_rtsp_url;
   std::string m_yolo_model_path;

 private: // State variables
   // Ownship state
   double m_own_x;
   double m_own_y;
   double m_own_heading;
   
   // Camera calibration
   double m_camera_height;
   double m_average_boat_height;
   double m_horizontal_fov;
   double m_vertical_fov;
   double m_distance_calibration_factor;
   
   // Detection parameters
   float m_confidence_threshold;
   float m_nms_threshold;
   int m_target_class;
   
   // Exclusion zone (normalized 0-1)
   double m_exclusion_x1, m_exclusion_y1;
   double m_exclusion_x2, m_exclusion_y2;
   
   // Statistics
   int m_frame_count;
   int m_detection_count;
   int m_published_count;
   double m_last_fps_time;
   double m_current_fps;
   
   // Vision thread
   std::thread m_vision_thread;
   std::atomic<bool> m_running;
   
   // Detection queue (thread-safe)
   std::queue<VisionDetection> m_detection_queue;
   std::mutex m_queue_mutex;
   
   // Camera and YOLO
   cv::VideoCapture m_camera;
   yolo::Inference* m_yolo;

 private: // Vision processing
   void visionLoop();
   double estimateDistance(int bbox_height_pixels, int image_height);
   double calculateBearing(int bbox_center_x, int image_width);
   bool isInExclusionZone(int x1, int y1, int x2, int y2, int img_width, int img_height);
   void publishDetection(const VisionDetection& det);
};

#endif
