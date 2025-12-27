// This replaces the visionLoop() function in VisionDetector.cpp

void VisionDetector::visionLoop()
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
        
        // Run YOLO inference - get structured detections
        std::vector<yolo::Detection> yolo_detections = m_yolo->GetDetections(frame);
        
        // Process each detection
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
                continue;  // Invalid distance
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
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 FPS
    }
    
    reportEvent("Vision thread stopped");
}
