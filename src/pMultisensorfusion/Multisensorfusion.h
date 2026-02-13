/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT                                             */
/*    FILE: Multisensorfusion.h                             */
/*    DATE: 2025                                            */
/*                                                          */
/* Multi-sensor fusion: Radar + AIS + Vision (bearing-only)*/
/* Adapted for AppCastingMOOSApp structure                 */
/************************************************************/

#ifndef Multisensorfusion_HEADER
#define Multisensorfusion_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

// Optional MOOSGeodesy support
#ifdef USE_MOOSGEODESY
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#endif

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <array>
#include <cmath>

// Forward declarations
struct Matrix2x2;
struct Matrix4x4;
struct Vec2;
struct Vec4;

//================================================================
// Simple Matrix Operations (no Eigen dependency)
//================================================================

struct Matrix2x2 {
    double data[2][2];
    Matrix2x2();
    double det() const;
    Matrix2x2 inv() const;
    Matrix2x2 operator+(const Matrix2x2& m) const;
    Matrix2x2 operator*(double s) const;
};

struct Matrix4x4 {
    double data[4][4];
    Matrix4x4();
    static Matrix4x4 identity();
    Matrix4x4 operator+(const Matrix4x4& m) const;
    Matrix4x4 operator*(const Matrix4x4& m) const;
    Matrix4x4 operator*(double s) const;
    Matrix4x4 inv() const;
};

struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    double norm() const { return std::sqrt(x*x + y*y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x-v.x, y-v.y); }
    Vec2 operator+(const Vec2& v) const { return Vec2(x+v.x, y+v.y); }
    Vec2 operator*(double s) const { return Vec2(x*s, y*s); }
};

struct Vec4 {
    double data[4];
    Vec4();
    Vec4(double x, double y, double vx, double vy);
    double& operator[](int i) { return data[i]; }
    const double& operator[](int i) const { return data[i]; }
};

//================================================================
// Sensor Measurements
//================================================================

struct RadarDetection {
    double x_m, y_m;
    double vx_ms, vy_ms;
    double range_m, bearing_deg;
    std::string label;
    double timestamp;
    bool has_velocity;
};

struct AISReport {
    std::string mmsi;
    double lat_deg, lon_deg;
    double sog_knots, cog_deg, heading_deg;
    double timestamp;
    bool has_heading;
    double x_m, y_m;  // Computed local
};

struct VisionDetection {
    std::string classname;
    double confidence;
    double bearing_deg;
    double timestamp;
    bool has_range;
    double range_m;
    bool has_bbox;
    int bbox_center_u, bbox_center_v;
    int image_width, image_height;
};

//================================================================
// Classification System
//================================================================

class ClassTracker {
public:
    ClassTracker();
    void updateClass(const std::string& detected_class, double confidence);
    std::string getMostLikelyClass() const;
    double getClassProbability(const std::string& class_name) const;
    void decay(double decay_rate);
    
private:
    std::map<std::string, double> m_class_probs;
};

//================================================================
// IMM Model Types
//================================================================

enum ModelType {
    MODEL_CV = 0,  // Constant Velocity
    MODEL_CT = 1   // Coordinated Turn
};

//================================================================
// Track
//================================================================

class Track {
public:
    Track(int id, double x, double y, double vx, double vy, double time);
    
    void predict(double dt, const std::array<double,2>& process_noise_cv,
                 const std::array<double,2>& process_noise_ct);
    
    void updateRadar(double x_meas, double y_meas,
                     const std::array<double,2>& meas_noise);
    void updateAIS(double x_meas, double y_meas, double vx_ais, double vy_ais,
                   const std::array<double,2>& meas_noise,
                   const std::array<double,2>& vel_noise);
    void updateVisionBearing(double bearing_meas_rad, double ownship_heading_rad,
                            double ownship_x, double ownship_y,
                            double bearing_noise_rad);
    void updateVisionClass(const std::string& detected_class, double confidence);
    
    double mahalanobisDistance(double x_meas, double y_meas) const;
    double angularDistance(double bearing_meas_rad, double ownship_heading_rad,
                          double ownship_x, double ownship_y) const;
    
    void incrementMissCount() { m_consecutive_misses++; }
    void resetMissCount() { m_consecutive_misses = 0; }
    void incrementHitCount() { m_hit_count++; m_scan_count++; }
    void incrementScanCount() { m_scan_count++; }
    
    bool isConfirmed() const { return m_confirmed; }
    void setConfirmed(bool c) { m_confirmed = c; }
    
    int getID() const { return m_id; }
    Vec4 getState() const { return m_state; }
    Matrix4x4 getCovariance() const { return m_P; }
    double getQuality() const { return m_quality; }
    int getHitCount() const { return m_hit_count; }
    int getScanCount() const { return m_scan_count; }
    int getMissCount() const { return m_consecutive_misses; }
    double getLastUpdateTime() const { return m_last_update_time; }
    
    void setMMSI(const std::string& mmsi) { m_mmsi = mmsi; m_has_mmsi = true; }
    bool hasMMSI() const { return m_has_mmsi; }
    std::string getMMSI() const { return m_mmsi; }
    
    void updateQuality(double delta);
    void setLastUpdateTime(double t) { m_last_update_time = t; }
    
    void setLastSource(const std::string& source) { m_last_source = source; }
    std::string getLastSource() const { return m_last_source; }
    bool hasRadar() const { return m_has_radar; }
    bool hasAIS() const { return m_has_ais; }
    bool hasVision() const { return m_has_vision; }
    void setRadar(bool v) { m_has_radar = v; }
    void setAIS(bool v) { m_has_ais = v; }
    void setVision(bool v) { m_has_vision = v; }
    
    std::string getClassification() const { return m_class_tracker.getMostLikelyClass(); }
    double getClassConfidence() const;
    
    bool isVisionOnly() const { return m_has_vision && !m_has_radar && !m_has_ais; }
    
private:
    int m_id;
    Vec4 m_state;
    Matrix4x4 m_P;
    
    std::array<Vec4, 2> m_state_modes;
    std::array<Matrix4x4, 2> m_P_modes;
    std::array<double, 2> m_mode_probs;
    
    bool m_confirmed;
    int m_hit_count;
    int m_scan_count;
    int m_consecutive_misses;
    double m_quality;
    double m_last_update_time;
    
    bool m_has_radar, m_has_ais, m_has_vision;
    std::string m_last_source;
    
    bool m_has_mmsi;
    std::string m_mmsi;
    
    ClassTracker m_class_tracker;
};

//================================================================
// Configuration
//================================================================

struct FusionConfig {
    double radar_cluster_eps_m;
    int radar_cluster_min_pts;
    
    std::array<double,2> radar_noise_xy;
    std::array<double,2> ais_pos_noise;
    std::array<double,2> ais_vel_noise;
    double vision_bearing_noise_deg;
    
    std::array<double,2> process_noise_cv;
    std::array<double,2> process_noise_ct;
    
    double gate_threshold;
    double bearing_gate_threshold;
    
    int confirm_min_hits;
    int confirm_min_scans;
    int delete_max_misses;
    double delete_min_quality;
    double quality_decay_rate;
    double quality_increment;
    double max_coast_time_s;
    
    bool vision_can_initiate_tracks;
    int vision_only_max_scans;
    double class_decay_rate;
    
    double camera_fx;
    double camera_cx;
    double camera_yaw_offset_deg;
    
    double publish_rate_hz;
    bool publish_debug;
    
    FusionConfig();
};

//================================================================
// Main MOOS Application (AppCasting version)
//================================================================

class Multisensorfusion : public AppCastingMOOSApp
{
 public:
   Multisensorfusion();
   ~Multisensorfusion();

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
   FusionConfig m_config;
   
   // Simple geodesy (when MOOSGeodesy not available)
   double m_origin_lat;
   double m_origin_lon;
   bool m_geodesy_initialized;

#ifdef USE_MOOSGEODESY
   CMOOSGeodesy m_geodesy;
#endif
   
   // Ownship state
   double m_nav_x, m_nav_y, m_nav_heading, m_nav_speed;
   double m_nav_lat, m_nav_lon;
   bool m_nav_valid;
   
   // Tracks
   std::map<int, std::shared_ptr<Track>> m_tracks;
   int m_next_track_id;
   
   // Timing
   double m_last_publish_time;
   double m_last_iterate_time;
   
   // Statistics
   int m_total_radar_updates;
   int m_total_ais_reports;
   int m_total_vision_reports;
   int m_radar_associations;
   int m_ais_associations;
   int m_vision_associations;

 private: // Message handlers
   void handleRadarTargets(const std::string& msg, double moos_time);
   void handleAISReport(const std::string& msg, double moos_time);
   void handleVisionReport(const std::string& msg, double moos_time);
   void handleNavUpdate(const std::string& var, double val, double time);
   
   // Message parsers
   std::vector<RadarDetection> parseRadarTargets(const std::string& msg);
   std::vector<AISReport> parseAISReports(const std::string& msg);
   std::vector<VisionDetection> parseVisionReports(const std::string& msg);
   
   // Coordinate transforms
   void convertAISToLocal(std::vector<AISReport>& reports);
   double computeBearingFromPixel(int pixel_u, int image_width);
   
   // Tracking pipeline
   void predictTracks(double current_time);
   void associateRadar(const std::vector<RadarDetection>& detections, double time);
   void associateAIS(const std::vector<AISReport>& reports, double time);
   void associateVision(const std::vector<VisionDetection>& detections, double time);
   void manageTracks();
   
   // Publishing
   void publishContacts(double time);
   void publishDebug();
   std::string generateNodeReport(const Track& track, double time);
   
   // Utility
   double normalizeAngle(double angle_deg);
   double wrapAngle(double angle_rad);
};

#endif
