/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT/CSAIL                                       */
/*    FILE: pSensorFusionTracker.cpp                        */
/*    DATE: December 2024                                   */
/************************************************************/

#include "pSensorFusionTracker.h"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cctype>

using namespace std;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

//=============================================================
// Constructor
//=============================================================
SensorFusionTracker::SensorFusionTracker()
{
    // Initialize track management
    next_track_id_ = 1;
    
    // Default configuration
    association_threshold_ = 15.0;  // meters (Mahalanobis gate)
    track_timeout_ = 5.0;          // seconds
    quality_threshold_ = 0.5;
    buffer_time_window_ = 0.5;     // 500ms buffer
    
    // Noise parameters
    radar_noise_pos_ = 3.0;        // 3m std dev
    radar_noise_vel_ = 0.5;        // 0.5 m/s std dev
    vision_noise_pos_ = 25.0;      // 25m std dev (less accurate)
    process_noise_pos_ = 0.5;      // 0.5 m/s^2
    process_noise_vel_ = 0.2;      // 0.2 m/s^2
    process_noise_yaw_ = 0.1;      // 0.1 rad/s
    
    // Track lifecycle
    min_hits_to_confirm_ = 3;
    max_misses_ = 5;
    
    // Own ship state
    nav_x_ = 0.0;
    nav_y_ = 0.0;
    nav_heading_ = 0.0;
    
    // Statistics
    radar_count_ = 0;
    vision_count_ = 0;
    fusion_count_ = 0;
    tracks_created_ = 0;
    tracks_deleted_ = 0;
}

SensorFusionTracker::~SensorFusionTracker()
{
}

//=============================================================
// MOOS Callbacks
//=============================================================
bool SensorFusionTracker::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);

    MOOSMSG_LIST::iterator p;
    for(p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();
        double time = msg.GetTime();
        
        if(key == "NAV_X") {
            nav_x_ = msg.GetDouble();
        }
        else if(key == "NAV_Y") {
            nav_y_ = msg.GetDouble();
        }
        else if(key == "NAV_HEADING") {
            nav_heading_ = msg.GetDouble();
        }
        else if(key == "RADAR_TARGETS") {
            handleRadarTargets(msg.GetString(), time);
        }
        else if(key == "VISION_REPORT") {
            handleVisionReport(msg.GetString(), time);
        }
    }

    return true;
}

bool SensorFusionTracker::Iterate()
{
    AppCastingMOOSApp::Iterate();

    processBufferedMeasurements();

    const double current_time = MOOSTime();

    for (auto& pair : tracks_) {
        auto& track = pair.second;

        double dt = current_time - track->getLastUpdateTime();
        if (dt < 0)   dt = 0;    // safety
        if (dt > 5.0) dt = 5.0;  // clamp (optional)

        if (dt > 0.01) {         // at least 10ms
            track->predict(dt);
        }

        track->setLastUpdateTime(current_time);
    }

    deleteStaleTracks();

    publishFusedContacts();
    publishNodeReports();

    AppCastingMOOSApp::PostReport();
    return true;
}


bool SensorFusionTracker::OnConnectToServer()
{
    registerVariables();
    return true;
}

bool SensorFusionTracker::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();
    
    loadConfig();
    
    registerVariables();
    return true;
}

void SensorFusionTracker::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_HEADING", 0);
    Register("RADAR_TARGETS", 0);
    Register("VISION_REPORT", 0);
}

//=============================================================
// Configuration
//=============================================================
void SensorFusionTracker::loadConfig()
{
    STRING_LIST sParams;
    m_MissionReader.GetConfiguration(GetAppName(), sParams);

    STRING_LIST::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
        string line = *p;
        size_t pos = line.find('=');
        if(pos == string::npos) continue;
        
        string param = line.substr(0, pos);
        string value = line.substr(pos + 1);
        
        // Trim
        param.erase(0, param.find_first_not_of(" \t"));
        param.erase(param.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        
        // Convert to lowercase
        for(char& c : param) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        
        if(param == "association_threshold")
            association_threshold_ = atof(value.c_str());
        else if(param == "track_timeout")
            track_timeout_ = atof(value.c_str());
        else if(param == "quality_threshold")
            quality_threshold_ = atof(value.c_str());
        else if(param == "radar_noise_pos")
            radar_noise_pos_ = atof(value.c_str());
        else if(param == "radar_noise_vel")
            radar_noise_vel_ = atof(value.c_str());
        else if(param == "vision_noise_pos")
            vision_noise_pos_ = atof(value.c_str());
        else if(param == "process_noise_pos")
            process_noise_pos_ = atof(value.c_str());
        else if(param == "process_noise_vel")
            process_noise_vel_ = atof(value.c_str());
        else if(param == "min_hits_to_confirm")
            min_hits_to_confirm_ = atoi(value.c_str());
        else if(param == "max_misses")
            max_misses_ = atoi(value.c_str());
    }
    
    reportEvent("Config loaded: assoc_thresh=" + to_string(association_threshold_));
}
//=============================================================
// Message Parsing
//=============================================================
void SensorFusionTracker::handleRadarTargets(const string& msg, double time)
{
    radar_count_++;
    
    // Parse RADAR_TARGETS format (from SPx/iRadarTargets)
    // Expected format: Multiple targets separated by semicolons
    // Each target: "x=10.5,y=20.3,vx=1.2,vy=0.5,range=25.3,bearing=45.2"
    
    istringstream iss(msg);
    string target_str;
    
    while(getline(iss, target_str, ';')) {
        if(target_str.empty()) continue;
        
        Measurement meas;
        meas.timestamp = time;
        meas.sensor = RADAR;
        meas.has_velocity = true;
        meas.label = "radar_" + to_string(radar_count_);
        meas.type = "unknown";
        meas.confidence = 0.9;  // Radar is reliable
        
        // Parse key=value pairs
        istringstream target_stream(target_str);
        string pair;
        
        while(getline(target_stream, pair, ',')) {
            size_t eq_pos = pair.find('=');
            if(eq_pos == string::npos) continue;
            
            string key = pair.substr(0, eq_pos);
            string val = pair.substr(eq_pos + 1);
            
            // Trim
            key.erase(0, key.find_first_not_of(" \t"));
            val.erase(0, val.find_first_not_of(" \t"));
            
            if(key == "x") meas.x = atof(val.c_str());
            else if(key == "y") meas.y = atof(val.c_str());
            else if(key == "vx") meas.vx = atof(val.c_str());
            else if(key == "vy") meas.vy = atof(val.c_str());
            else if(key == "range") meas.range = atof(val.c_str());
            else if(key == "bearing") meas.bearing = atof(val.c_str());
            else if(key == "label") meas.label = val;
        }
        
        // Set measurement noise covariance
        meas.R = Matrix2d::Identity() * radar_noise_pos_ * radar_noise_pos_;
        
        // Add to buffer
        measurement_buffer_.push(meas);
        
        reportEvent("Radar target: x=" + to_string(meas.x) + 
                   ", y=" + to_string(meas.y));
    }
}

void SensorFusionTracker::handleVisionReport(const string& msg, double time)
{
    vision_count_++;
    
    // Parse VISION_REPORT format (from pVisionDetector)
    // Format: "x=10.5,y=20.3,confidence=0.85,bearing=45.2,range=15.3,class=boat,timestamp=123.456"
    
    Measurement meas;
    meas.timestamp = time;
    meas.sensor = VISION;
    meas.has_velocity = false;  // Vision doesn't provide velocity directly
    
    // Parse key=value pairs
    istringstream iss(msg);
    string pair;
    
    while(getline(iss, pair, ',')) {
        size_t eq_pos = pair.find('=');
        if(eq_pos == string::npos) continue;
        
        string key = pair.substr(0, eq_pos);
        string val = pair.substr(eq_pos + 1);
        
        // Trim
        key.erase(0, key.find_first_not_of(" \t"));
        val.erase(0, val.find_first_not_of(" \t"));
        
        if(key == "x") meas.x = atof(val.c_str());
        else if(key == "y") meas.y = atof(val.c_str());
        else if(key == "confidence") meas.confidence = atof(val.c_str());
        else if(key == "bearing") meas.bearing = atof(val.c_str());
        else if(key == "range") meas.range = atof(val.c_str());
        else if(key == "class") meas.type = val;
    }
    
    meas.label = "vision_" + to_string(vision_count_);
    
    // Set measurement noise (vision less accurate in position)
    meas.R = Matrix2d::Identity() * vision_noise_pos_ * vision_noise_pos_;
    
    // Add to buffer
    measurement_buffer_.push(meas);
    
    reportEvent("Vision detection: " + meas.type + " at (" + 
               to_string(meas.x) + "," + to_string(meas.y) + ")");
}

//=============================================================
// Core Fusion Pipeline
//=============================================================
void SensorFusionTracker::processBufferedMeasurements()
{
    if(measurement_buffer_.empty()) return;
    
    double current_time = MOOSTime();
    vector<Measurement> batch;
    
    // Extract measurements within time window
    while(!measurement_buffer_.empty()) {
        Measurement& meas = measurement_buffer_.front();
        
        if(current_time - meas.timestamp > buffer_time_window_) {
            // Too old, discard
            measurement_buffer_.pop();
            continue;
        }
        
        batch.push_back(meas);
        measurement_buffer_.pop();
    }
    
    if(batch.empty()) return;
    
    // Data association
    dataAssociation(batch);
}

void SensorFusionTracker::dataAssociation(vector<Measurement>& measurements)
{
    if(measurements.empty()) return;
    
    int n_tracks = tracks_.size();
    int n_meas = measurements.size();
    
    if(n_tracks == 0) {
        // No tracks, initialize all measurements
        for(int i = 0; i < n_meas; i++) {
            initiateTracks({i}, measurements);
        }
        return;
    }
    
    // Build cost matrix (Mahalanobis distance)
    MatrixXd cost_matrix(n_tracks, n_meas);
    
    int track_idx = 0;
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        for(int meas_idx = 0; meas_idx < n_meas; meas_idx++) {
            double dist = mahalanobisDistance(*track, measurements[meas_idx]);
            
            // Gating: Reject if too far
            if(dist > association_threshold_) {
                cost_matrix(track_idx, meas_idx) = 1e9;  // Large cost
            } else {
                cost_matrix(track_idx, meas_idx) = dist;
            }
        }
        track_idx++;
    }
    
    // Hungarian algorithm
    vector<pair<int,int>> assignments = hungarianAssignment(cost_matrix);
    
    // Find unassigned measurements
    vector<int> assigned_meas(n_meas, 0);
    for(auto& pair : assignments) {
        assigned_meas[pair.second] = 1;
    }
    
    vector<int> unassigned_meas;
    for(int i = 0; i < n_meas; i++) {
        if(!assigned_meas[i]) {
            unassigned_meas.push_back(i);
        }
    }
    
    // Update tracks with assignments
    updateTracks(assignments, measurements);
    
    // Initialize new tracks from unassigned measurements
    if(!unassigned_meas.empty()) {
        initiateTracks(unassigned_meas, measurements);
    }
    
    // Increment misses for unassigned tracks
    vector<int> assigned_tracks(n_tracks, 0);
    for(auto& pair : assignments) {
        assigned_tracks[pair.first] = 1;
    }
    
    track_idx = 0;
    for(auto& pair : tracks_) {
        if(!assigned_tracks[track_idx]) {
            pair.second->incrementMisses();
        }
        track_idx++;
    }
}

void SensorFusionTracker::updateTracks(
    const vector<pair<int,int>>& assignments,
    vector<Measurement>& measurements)
{
    int track_idx = 0;
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        // Find if this track has an assignment
        for(auto& assign : assignments) {
            if(assign.first == track_idx) {
                int meas_idx = assign.second;
                track->update(measurements[meas_idx]);
                track->incrementAge();
                
                reportEvent("Track " + to_string(track->getID()) + 
                           " updated with " + 
                           (measurements[meas_idx].sensor == RADAR ? "radar" : "vision"));
                break;
            }
        }
        track_idx++;
    }
}

void SensorFusionTracker::initiateTracks(
    const vector<int>& unassigned_meas,
    vector<Measurement>& measurements)
{
    for(int idx : unassigned_meas) {
        auto track = make_shared<Track>(next_track_id_++, measurements[idx]);
        tracks_[track->getID()] = track;
        tracks_created_++;
        
        reportEvent("New track " + to_string(track->getID()) + 
                   " initiated from " + measurements[idx].label);
    }
}

void SensorFusionTracker::deleteStaleTracks()
{
    vector<int> to_delete;
    
    for(auto& pair : tracks_) {
        if(pair.second->shouldDelete()) {
            to_delete.push_back(pair.first);
        }
    }
    
    for(int id : to_delete) {
        reportEvent("Deleting stale track " + to_string(id));
        tracks_.erase(id);
        tracks_deleted_++;
    }
}

//=============================================================
// Utilities
//=============================================================
double SensorFusionTracker::mahalanobisDistance(
    const Track& track, 
    const Measurement& meas)
{
    // Innovation (measurement residual)
    Vector2d z(meas.x, meas.y);
    Vector2d z_pred(track.getPx(), track.getPy());
    Vector2d innovation = z - z_pred;
    
    // Innovation covariance
    Matrix2d P_pos = track.getCovariance().block<2,2>(0,0);
    Matrix2d S = P_pos + meas.R;
    
    // Mahalanobis distance
    double dist = sqrt(innovation.transpose() * S.inverse() * innovation);
    
    return dist;
}

Matrix2d SensorFusionTracker::getMeasurementNoise(SensorType sensor)
{
    if(sensor == RADAR) {
        return Matrix2d::Identity() * radar_noise_pos_ * radar_noise_pos_;
    } else {
        return Matrix2d::Identity() * vision_noise_pos_ * vision_noise_pos_;
    }
}
//=============================================================
// Track Class Implementation
//=============================================================

Track::Track(int id, const Measurement& init_meas)
    : id_(id)
    , state_(TENTATIVE)
    , age_(1)
    , misses_(0)
    , max_misses_(5)
    , quality_(0.3)
    , last_update_time_(init_meas.timestamp)
    , radar_updates_(0)
    , vision_updates_(0)
{
    // Initialize state vector [px, py, v, yaw, yaw_rate, vx, vy]
    x_ = VectorXd(7);
    x_ << init_meas.x, init_meas.y, 0.0, 0.0, 0.0, 
          init_meas.has_velocity ? init_meas.vx : 0.0,
          init_meas.has_velocity ? init_meas.vy : 0.0;
    
    // Initialize covariance
    P_ = MatrixXd::Identity(7, 7);
    P_(0,0) = 10.0;   // px uncertainty
    P_(1,1) = 10.0;   // py uncertainty
    P_(2,2) = 5.0;    // v uncertainty
    P_(3,3) = 1.0;    // yaw uncertainty
    P_(4,4) = 0.5;    // yaw_rate uncertainty
    P_(5,5) = 2.0;    // vx uncertainty
    P_(6,6) = 2.0;    // vy uncertainty
    
    // Process noise
    Q_ = MatrixXd::Identity(7, 7);
    Q_(0,0) = 0.5;    // position noise
    Q_(1,1) = 0.5;
    Q_(2,2) = 0.2;    // velocity noise
    Q_(3,3) = 0.1;    // yaw noise
    Q_(4,4) = 0.05;   // yaw rate noise
    Q_(5,5) = 0.2;
    Q_(6,6) = 0.2;
    
    // UKF parameters (for 7D state)
    double alpha = 0.001;
    double beta = 2.0;
    double kappa = 0.0;
    lambda_ = alpha * alpha * (7 + kappa) - 7;
    
    // Weights
    int n_sigma = 2 * 7 + 1;
    weights_mean_ = VectorXd(n_sigma);
    weights_cov_ = VectorXd(n_sigma);
    
    weights_mean_(0) = lambda_ / (7 + lambda_);
    weights_cov_(0) = lambda_ / (7 + lambda_) + (1 - alpha*alpha + beta);
    
    for(int i = 1; i < n_sigma; i++) {
        weights_mean_(i) = 0.5 / (7 + lambda_);
        weights_cov_(i) = 0.5 / (7 + lambda_);
    }
    
    // Classification
    label_ = init_meas.label;
    type_ = init_meas.type;
    last_sensor_ = init_meas.sensor;
    
    if(init_meas.sensor == RADAR) radar_updates_++;
    else vision_updates_++;
}

void Track::predict(double dt)
{
    // Generate sigma points
    generateSigmaPoints();
    
    // Predict sigma points through motion model
    predictSigmaPoints(dt);
    
    // Recover mean and covariance
    x_.setZero();
    for(int i = 0; i < sigma_points_.cols(); i++) {
        x_ += weights_mean_(i) * sigma_points_.col(i);
    }
    
    P_.setZero();
    for(int i = 0; i < sigma_points_.cols(); i++) {
        VectorXd diff = sigma_points_.col(i) - x_;
        P_ += weights_cov_(i) * diff * diff.transpose();
    }
    
    P_ += Q_ * dt;  // Add process noise

    // Advance track time so Iterate() does not re-predict the same interval repeatedly
    last_update_time_ += dt;
}

void Track::update(const Measurement& meas)
{
    // Measurement vector
    Vector2d z(meas.x, meas.y);
    
    // Predicted measurement
    Vector2d z_pred(x_(0), x_(1));
    
    // Innovation
    Vector2d y = z - z_pred;
    
    // Measurement Jacobian (identity for position)
    MatrixXd H = MatrixXd::Zero(2, 7);
    H(0,0) = 1.0;
    H(1,1) = 1.0;
    
    // Innovation covariance
    Matrix2d S = H * P_ * H.transpose() + meas.R;
    
    // Kalman gain
    MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state
    x_ += K * y;
    
    // Update covariance
    MatrixXd I = MatrixXd::Identity(7, 7);
    P_ = (I - K * H) * P_;
    
    // Update velocity from state if measurement has velocity
    if(meas.has_velocity) {
        x_(5) = meas.vx;
        x_(6) = meas.vy;
        x_(2) = sqrt(meas.vx*meas.vx + meas.vy*meas.vy);
        x_(3) = atan2(meas.vy, meas.vx);
    }
    
    // Update quality
    double sensor_bonus = (meas.sensor == RADAR) ? 1.5 : 0.8;
    quality_ = min(1.0, quality_ + 0.1 * sensor_bonus);
    
    // Update classification
    if(meas.sensor == VISION && !meas.type.empty()) {
        type_ = meas.type;
    }
    
    // Track state management
    if(age_ >= 3 && quality_ > 0.5) {
        state_ = CONFIRMED;
    }
    
    // Update metadata
    last_update_time_ = meas.timestamp;
    last_sensor_ = meas.sensor;
    
    if(meas.sensor == RADAR) radar_updates_++;
    else vision_updates_++;
}

void Track::generateSigmaPoints()
{
    int n = x_.size();
    int n_sigma = 2 * n + 1;
    
    sigma_points_ = MatrixXd(n, n_sigma);
    
    // Cholesky decomposition
    MatrixXd A = ((n + lambda_) * P_).llt().matrixL();
    
    // First sigma point is the mean
    sigma_points_.col(0) = x_;
    
    // Generate remaining sigma points
    for(int i = 0; i < n; i++) {
        sigma_points_.col(i + 1) = x_ + A.col(i);
        sigma_points_.col(i + 1 + n) = x_ - A.col(i);
    }
}

void Track::predictSigmaPoints(double dt)
{
    // CTRV motion model for each sigma point
    for(int i = 0; i < sigma_points_.cols(); i++) {
        double px = sigma_points_(0, i);
        double py = sigma_points_(1, i);
        double v = sigma_points_(2, i);
        double yaw = sigma_points_(3, i);
        double yawd = sigma_points_(4, i);
        
        double px_p, py_p;
        
        // Avoid division by zero
        if(fabs(yawd) > 0.001) {
            px_p = px + v/yawd * (sin(yaw + yawd*dt) - sin(yaw));
            py_p = py + v/yawd * (cos(yaw) - cos(yaw + yawd*dt));
        } else {
            px_p = px + v*dt*cos(yaw);
            py_p = py + v*dt*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;
        
        // Update vx, vy
        double vx_p = v_p * cos(yaw_p);
        double vy_p = v_p * sin(yaw_p);
        
        // Write back
        sigma_points_(0, i) = px_p;
        sigma_points_(1, i) = py_p;
        sigma_points_(2, i) = v_p;
        sigma_points_(3, i) = yaw_p;
        sigma_points_(4, i) = yawd_p;
        sigma_points_(5, i) = vx_p;
        sigma_points_(6, i) = vy_p;
    }
}
//=============================================================
// Hungarian Algorithm for Assignment
//=============================================================
vector<pair<int,int>> SensorFusionTracker::hungarianAssignment(
    const MatrixXd& cost_matrix)
{
    // Simple implementation of Hungarian algorithm
    // For production, use library like munkres-cpp
    
    int n_rows = cost_matrix.rows();
    int n_cols = cost_matrix.cols();
    
    if(n_rows == 0 || n_cols == 0) {
        return vector<pair<int,int>>();
    }
    
    // For simplicity, use greedy nearest neighbor
    // TODO: Replace with full Hungarian for optimal assignment
    vector<pair<int,int>> assignments;
    vector<bool> assigned_rows(n_rows, false);
    vector<bool> assigned_cols(n_cols, false);
    
    // Greedy assignment: iterate and pick minimum cost
    for(int iter = 0; iter < min(n_rows, n_cols); iter++) {
        double min_cost = 1e9;
        int best_row = -1;
        int best_col = -1;
        
        for(int i = 0; i < n_rows; i++) {
            if(assigned_rows[i]) continue;
            
            for(int j = 0; j < n_cols; j++) {
                if(assigned_cols[j]) continue;
                
                if(cost_matrix(i, j) < min_cost) {
                    min_cost = cost_matrix(i, j);
                    best_row = i;
                    best_col = j;
                }
            }
        }
        
        // Check if valid assignment found
        if(best_row >= 0 && min_cost < 1e8) {
            assignments.push_back({best_row, best_col});
            assigned_rows[best_row] = true;
            assigned_cols[best_col] = true;
        } else {
            break;  // No more valid assignments
        }
    }
    
    return assignments;
}

//=============================================================
// Publishing
//=============================================================
void SensorFusionTracker::publishFusedContacts()
{
    if(tracks_.empty()) return;
    
    // Build FUSED_CONTACTS message
    // Format: Multiple contacts separated by semicolons
    ostringstream oss;
    bool first = true;
    
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        // Only publish confirmed tracks above quality threshold
        if(!track->isConfirmed() || track->getQuality() < quality_threshold_) {
            continue;
        }
        
        if(!first) oss << ";";
        first = false;
        
        oss << fixed << setprecision(2);
        oss << "label=" << track->getLabel() << ",";
        oss << "id=" << track->getID() << ",";
        oss << "x=" << track->getPx() << ",";
        oss << "y=" << track->getPy() << ",";
        oss << "vx=" << track->getVx() << ",";
        oss << "vy=" << track->getVy() << ",";
        oss << "speed=" << track->getSpeed() << ",";
        oss << "heading=" << (track->getYaw() * 180.0 / kPi) << ",";
        oss << "type=" << track->getType() << ",";
        oss << "quality=" << track->getQuality();
        
        fusion_count_++;
    }
    
    if(!first) {  // At least one track published
        Notify("FUSED_CONTACTS", oss.str());
    }
}

void SensorFusionTracker::publishNodeReports()
{
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        // Only publish confirmed tracks
        if(!track->isConfirmed() || track->getQuality() < quality_threshold_) {
            continue;
        }
        
        // Build NODE_REPORT for pMarineViewer
        ostringstream oss;
        oss << fixed << setprecision(2);
        oss << "NAME=track_" << track->getID() << ",";
        oss << "TYPE=" << (track->getType().empty() ? "ship" : track->getType()) << ",";
        oss << "X=" << track->getPx() << ",";
        oss << "Y=" << track->getPy() << ",";
        oss << "SPD=" << (track->getSpeed() * 1.94384) << ",";  // m/s to knots
        oss << "HDG=" << (track->getYaw() * 180.0 / kPi) << ",";
        oss << "YAW=" << (track->getYaw() * 180.0 / kPi) << ",";
        oss << "TIME=" << setprecision(3) << MOOSTime() << ",";
        oss << "DEPTH=0.0,";
        oss << "LENGTH=10,";
        oss << "MODE=DRIVE";
        
        Notify("NODE_REPORT", oss.str());
    }
}

//=============================================================
// AppCasting
//=============================================================
bool SensorFusionTracker::buildReport()
{
    m_msgs << "========================================" << endl;
    m_msgs << "pSensorFusionTracker Status" << endl;
    m_msgs << "========================================" << endl;
    m_msgs << endl;
    
    // Configuration
    m_msgs << "Configuration:" << endl;
    m_msgs << "  Association threshold: " << association_threshold_ << " m" << endl;
    m_msgs << "  Track timeout:         " << track_timeout_ << " s" << endl;
    m_msgs << "  Quality threshold:     " << quality_threshold_ << endl;
    m_msgs << "  Min hits to confirm:   " << min_hits_to_confirm_ << endl;
    m_msgs << "  Max misses:            " << max_misses_ << endl;
    m_msgs << endl;
    
    // Statistics
    m_msgs << "Statistics:" << endl;
    m_msgs << "  Radar measurements:    " << radar_count_ << endl;
    m_msgs << "  Vision measurements:   " << vision_count_ << endl;
    m_msgs << "  Fused contacts:        " << fusion_count_ << endl;
    m_msgs << "  Tracks created:        " << tracks_created_ << endl;
    m_msgs << "  Tracks deleted:        " << tracks_deleted_ << endl;
    m_msgs << "  Active tracks:         " << tracks_.size() << endl;
    m_msgs << endl;
    
    // Active tracks
    m_msgs << "Active Tracks:" << endl;
    m_msgs << "ID   | State   | Quality | Pos (x,y)      | Vel (vx,vy)   | Type     | R/V" << endl;
    m_msgs << "-----|---------|---------|----------------|---------------|----------|-----" << endl;
    
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        m_msgs << setw(4) << track->getID() << " | ";
        m_msgs << (track->isConfirmed() ? "CONFIRM" : "TENTATV") << " | ";
        m_msgs << fixed << setprecision(2) << setw(7) << track->getQuality() << " | ";
        m_msgs << "(" << setw(6) << track->getPx() << "," << setw(6) << track->getPy() << ") | ";
        m_msgs << "(" << setw(5) << track->getVx() << "," << setw(5) << track->getVy() << ") | ";
        m_msgs << setw(8) << track->getType() << " | ";
        m_msgs << "R/V" << endl;  // Could show radar_updates/vision_updates
    }
    
    return true;
}

//=============================================================
// Main
//=============================================================
int main(int argc, char *argv[])
{
    SensorFusionTracker app;
    app.Run(argv[0], argc, argv);
    return 0;
}