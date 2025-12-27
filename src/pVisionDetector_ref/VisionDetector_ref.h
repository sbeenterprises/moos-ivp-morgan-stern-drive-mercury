/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VisionDetector_ref.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef VisionDetector_ref_HEADER
#define VisionDetector_ref_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

// Forward declarations
namespace yolo {
    class Inference;
}

struct VisionDetection {
    double x;              // World X coordinate (meters)
    double y;              // World Y coordinate (meters)
    double confidence;     // YOLO confidence (0-1)
    double bearing;        // True bearing (0-360 degrees)
    double range;          // Distance (meters)
    std::string classname; // Detection class
    double timestamp;      // MOOSTime

    // Raw bbox for debugging
    int bbox_x1, bbox_y1, bbox_x2, bbox_y2;
};

class VisionDetector_ref : public AppCastingMOOSApp
{
 public:
   VisionDetector_ref();
   ~VisionDetector_ref();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
   std::string m_rtsp_url;
   std::string m_yolo_model_path;

   // Camera calibration parameters
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

 private: // State variables
   // Own ship state
   double m_own_x;
   double m_own_y;
   double m_own_heading;

   // Statistics
   unsigned long m_frame_count;
   unsigned long m_detection_count;
   unsigned long m_published_count;
   double m_last_fps_time;
   double m_current_fps;

   // Vision processing thread
   void visionLoop();
   std::thread m_vision_thread;
   std::atomic<bool> m_running;

   // Detection queue (thread-safe)
   std::queue<VisionDetection> m_detection_queue;
   std::mutex m_queue_mutex;

   // YOLO inference
   yolo::Inference* m_yolo;

   // Camera
   cv::VideoCapture m_camera;

   // Helper functions
   double estimateDistance(int bbox_height_pixels, int image_height);
   double calculateBearing(int bbox_center_x, int image_width);
   bool isInExclusionZone(int x1, int y1, int x2, int y2,
                         int img_width, int img_height);
   void publishDetection(const VisionDetection& det);
};

#endif 
