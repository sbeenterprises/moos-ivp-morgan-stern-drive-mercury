/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT/CSAIL                                       */
/*    FILE: pSensorFusionTracker.h                          */
/*    DATE: December 2024                                   */
/************************************************************/

#ifndef SENSOR_FUSION_TRACKER_HEADER
#define SENSOR_FUSION_TRACKER_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include <queue>
#include <string>

using namespace Eigen;

//=============================================================
// Measurement Structure
//=============================================================
enum SensorType { RADAR, VISION, FUSED };

struct Measurement {
    double timestamp;
    SensorType sensor;
    
    // Position (local frame)
    double x, y;
    
    // Velocity (if available)
    double vx, vy;
    bool has_velocity;
    
    // Polar coordinates (if from radar)
    double range, bearing;
    
    // Classification
    std::string label;
    std::string type;
    double confidence;
    
    // Covariance
    Matrix2d R;  // Measurement noise covariance
};

//=============================================================
// UKF-based Track
//=============================================================
class Track {
public:
    Track(int id, const Measurement& init_meas);
    
    // UKF Operations
    void predict(double dt);
    void update(const Measurement& meas);
    
    // Track management
    void incrementAge() { age_++; misses_ = 0; }
    void incrementMisses() { misses_++; }
    
    // Getters
    int getID() const { return id_; }
    VectorXd getState() const { return x_; }
    MatrixXd getCovariance() const { return P_; }
    double getQuality() const { return quality_; }
    std::string getLabel() const { return label_; }
    std::string getType() const { return type_; }
    
    bool isConfirmed() const { return state_ == CONFIRMED; }
    bool shouldDelete() const { return misses_ > max_misses_; }
    
    // State access
    double getPx() const { return x_(0); }
    double getPy() const { return x_(1); }
    double getVx() const { return x_(5); }
    double getVy() const { return x_(6); }
    double getSpeed() const { return x_(2); }
    double getYaw() const { return x_(3); }

    double getLastUpdateTime() const { return last_update_time_; }
    void   setLastUpdateTime(double t) { last_update_time_ = t; }

    
    bool isFused() const { return has_radar_update_ && has_vision_update_; }

    
private:
    // UKF Implementation
    void generateSigmaPoints();
    void predictSigmaPoints(double dt);
    VectorXd predictMeasurement(const VectorXd& state);
    
    // State
    enum TrackState { TENTATIVE, CONFIRMED, COASTING };
    TrackState state_;
    
    int id_;
    VectorXd x_;     // State: [px, py, v, yaw, yaw_rate, vx, vy]
    MatrixXd P_;     // Covariance
    MatrixXd Q_;     // Process noise
    
    // UKF parameters
    MatrixXd sigma_points_;
    VectorXd weights_mean_;
    VectorXd weights_cov_;
    double lambda_;
    
    // Track quality
    int age_;
    int misses_;
    int max_misses_;
    double quality_;
    
    // Classification
    std::string label_;
    std::string type_;
    SensorType last_sensor_;
    
    // Fusion metadata
    double last_update_time_ = 0.0;
    int radar_updates_;
    int vision_updates_;

     bool has_radar_update_;
    bool has_vision_update_;

};

//=============================================================
// Optimal Hungarian Algorithm (Munkres Algorithm)
//=============================================================
class HungarianAlgorithm {
public:
    std::vector<std::pair<int,int>> solve(const MatrixXd& cost_matrix);
    
private:
    void step1(MatrixXd& cost, int n);
    void step2(MatrixXd& cost, int n, std::vector<int>& row_cover, 
               std::vector<int>& col_cover, std::vector<std::vector<int>>& starred);
    int step3(int n, std::vector<int>& col_cover, 
              std::vector<std::vector<int>>& starred);
    bool step4(MatrixXd& cost, int n, std::vector<int>& row_cover,
               std::vector<int>& col_cover, std::vector<std::vector<int>>& starred,
               std::vector<std::vector<int>>& primed, int& zero_row, int& zero_col);
    void step5(int n, std::vector<int>& row_cover, std::vector<int>& col_cover,
               std::vector<std::vector<int>>& starred, std::vector<std::vector<int>>& primed,
               int zero_row, int zero_col);
    void step6(MatrixXd& cost, int n, std::vector<int>& row_cover,
               std::vector<int>& col_cover);
};

//=============================================================
// Main Application
//=============================================================
class SensorFusionTracker : public AppCastingMOOSApp {
public:
    SensorFusionTracker();
    ~SensorFusionTracker();

protected:
    // MOOS Overrides
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void registerVariables();
    bool buildReport();

private:
    // Configuration
    void loadConfig();
    
    // Message parsing
    void handleRadarTargets(const std::string& msg, double time);
    void handleVisionReport(const std::string& msg, double time);
    
    // Core fusion
    void processBufferedMeasurements();
    void dataAssociation(std::vector<Measurement>& measurements);
    void updateTracks(const std::vector<std::pair<int,int>>& assignments,
                     std::vector<Measurement>& measurements);
    void initiateTracks(const std::vector<int>& unassigned_meas,
                       std::vector<Measurement>& measurements);
    void deleteStaleTracks();
    
    // Hungarian algorithm
    std::vector<std::pair<int,int>> hungarianAssignment(
        const MatrixXd& cost_matrix);
    
    // Publishing
    void publishFusedContacts();
    void publishNodeReports();
    
    // Utilities
    double mahalanobisDistance(const Track& track, const Measurement& meas);
    Matrix2d getMeasurementNoise(SensorType sensor);
    
private:
    // Tracks
    std::map<int, std::shared_ptr<Track>> tracks_;
    int next_track_id_;
    
    // Measurement buffer (for temporal alignment)
    std::queue<Measurement> measurement_buffer_;
    double buffer_time_window_;
    
    // Configuration
    double association_threshold_;
    double track_timeout_;
    double quality_threshold_;
    
    double radar_noise_pos_;
    double radar_noise_vel_;
    double vision_noise_pos_;
    double process_noise_pos_;
    double process_noise_vel_;
    double process_noise_yaw_;
    
    int min_hits_to_confirm_;
    int max_misses_;
    
    // Own ship state
    double nav_x_, nav_y_, nav_heading_;
    
    // Statistics
    unsigned long radar_count_;
    unsigned long vision_count_;
    unsigned long fusion_count_;
    unsigned long tracks_created_;
    unsigned long tracks_deleted_;
};

#endif