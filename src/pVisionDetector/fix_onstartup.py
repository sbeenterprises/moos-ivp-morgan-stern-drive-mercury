#!/usr/bin/env python3
import re

# Read the file
with open('VisionDetector.cpp', 'r') as f:
    content = f.read()

# Find and replace the entire OnStartUp function
old_pattern = r'bool VisionDetector::OnStartUp\(\).*?^}'
new_function = '''bool VisionDetector::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();
    
    std::cout << "DEBUG: OnStartUp() called" << std::endl;

    // Read configuration
    STRING_LIST sParams;
    m_MissionReader.GetConfiguration(GetAppName(), sParams);
    
    std::cout << "DEBUG: Read " << sParams.size() << " config lines" << std::endl;

    STRING_LIST::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
        string line = *p;
        size_t pos = line.find('=');
        if (pos == string::npos) continue;
        
        string param = line.substr(0, pos);
        string value = line.substr(pos + 1);
        param.erase(0, param.find_first_not_of(" \\t"));
        param.erase(param.find_last_not_of(" \\t") + 1);
        value.erase(0, value.find_first_not_of(" \\t"));
        value.erase(value.find_last_not_of(" \\t") + 1);
        
        for (size_t i = 0; i < param.length(); i++) {
            param[i] = tolower(param[i]);
        }
        
        std::cout << "DEBUG: " << param << " = " << value << std::endl;

        if(param == "rtsp_url") m_rtsp_url = value;
        else if(param == "yolo_model_path") m_yolo_model_path = value;
        else if(param == "camera_height") m_camera_height = atof(value.c_str());
        else if(param == "average_boat_height") m_average_boat_height = atof(value.c_str());
        else if(param == "horizontal_fov") m_horizontal_fov = atof(value.c_str());
        else if(param == "vertical_fov") m_vertical_fov = atof(value.c_str());
        else if(param == "distance_calibration_factor") m_distance_calibration_factor = atof(value.c_str());
        else if(param == "confidence_threshold") m_confidence_threshold = atof(value.c_str());
        else if(param == "nms_threshold") m_nms_threshold = atof(value.c_str());
        else if(param == "target_class") m_target_class = atoi(value.c_str());
        else if(param == "exclusion_zone") {
            std::stringstream ss(value);
            string item;
            vector<double> values;
            while (getline(ss, item, ',')) {
                item.erase(0, item.find_first_not_of(" \\t"));
                item.erase(item.find_last_not_of(" \\t") + 1);
                values.push_back(atof(item.c_str()));
            }
            if (values.size() == 4) {
                m_exclusion_x1 = values[0];
                m_exclusion_y1 = values[1];
                m_exclusion_x2 = values[2];
                m_exclusion_y2 = values[3];
            }
        }
    }
    
    std::cout << "DEBUG: Config parsed" << std::endl;

    if (m_rtsp_url.empty()) {
        std::cout << "ERROR: rtsp_url empty" << std::endl;
        reportConfigWarning("rtsp_url not specified");
        return false;
    }
    
    if (m_yolo_model_path.empty()) {
        std::cout << "ERROR: yolo_model_path empty" << std::endl;
        reportConfigWarning("yolo_model_path not specified");
        return false;
    }
    
    try {
        std::cout << "DEBUG: Loading YOLO..." << std::endl;
        reportEvent("Loading YOLO model: " + m_yolo_model_path);
        m_yolo = new yolo::Inference(m_yolo_model_path, cv::Size(640, 640),
                                     m_confidence_threshold, m_nms_threshold);
        std::cout << "DEBUG: YOLO loaded" << std::endl;
        reportEvent("YOLO model loaded successfully");
    }
    catch (const std::exception& e) {
        std::cout << "ERROR: YOLO: " << e.what() << std::endl;
        reportRunWarning("Failed to load YOLO model");
        return false;
    }
    
    std::cout << "DEBUG: Opening camera..." << std::endl;
    reportEvent("Connecting to camera: " + m_rtsp_url);
    m_camera.open(m_rtsp_url);
    m_camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    if (!m_camera.isOpened()) {
        std::cout << "ERROR: Camera failed" << std::endl;
        reportRunWarning("Failed to open camera");
        return false;
    }
    
    std::cout << "DEBUG: Camera opened" << std::endl;
    reportEvent("Camera connected successfully");
    
    m_running = true;
    m_vision_thread = std::thread(&VisionDetector::visionLoop, this);
    
    std::cout << "DEBUG: Thread started" << std::endl;
    reportEvent("Vision processing thread started");
    
    registerVariables();
    
    std::cout << "DEBUG: OnStartUp complete" << std::endl;
    return true;
}'''

content_new = re.sub(old_pattern, new_function, content, flags=re.MULTILINE | re.DOTALL)

with open('VisionDetector.cpp', 'w') as f:
    f.write(content_new)

print("Fixed!")
