/************************************************************/
/*    NAME: Abdul Aziz                                      */
/*    ORGN: MIT                                             */
/*    FILE: Multisensorfusion.cpp                           */
/*    DATE: 2025                                            */
/*                                                          */
/* Multi-sensor fusion: Radar + AIS + Vision (bearing-only)*/
/* Adapted for AppCastingMOOSApp structure                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Multisensorfusion.h"

using namespace std;

//================================================================
// Matrix Helper Implementations
//================================================================

Matrix2x2::Matrix2x2() {
    data[0][0]=1; data[0][1]=0;
    data[1][0]=0; data[1][1]=1;
}

double Matrix2x2::det() const {
    return data[0][0]*data[1][1] - data[0][1]*data[1][0];
}

Matrix2x2 Matrix2x2::inv() const {
    Matrix2x2 result;
    double d = det();
    if(fabs(d) < 1e-10) return *this;
    result.data[0][0] = data[1][1] / d;
    result.data[0][1] = -data[0][1] / d;
    result.data[1][0] = -data[1][0] / d;
    result.data[1][1] = data[0][0] / d;
    return result;
}

Matrix2x2 Matrix2x2::operator+(const Matrix2x2& m) const {
    Matrix2x2 result;
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++)
            result.data[i][j] = data[i][j] + m.data[i][j];
    return result;
}

Matrix2x2 Matrix2x2::operator*(double s) const {
    Matrix2x2 result;
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++)
            result.data[i][j] = data[i][j] * s;
    return result;
}

Matrix4x4::Matrix4x4() {
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            data[i][j] = 0;
}

Matrix4x4 Matrix4x4::identity() {
    Matrix4x4 m;
    for(int i=0; i<4; i++) m.data[i][i] = 1;
    return m;
}

Matrix4x4 Matrix4x4::operator+(const Matrix4x4& m) const {
    Matrix4x4 result;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            result.data[i][j] = data[i][j] + m.data[i][j];
    return result;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& m) const {
    Matrix4x4 result;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            for(int k=0; k<4; k++)
                result.data[i][j] += data[i][k] * m.data[k][j];
    return result;
}

Matrix4x4 Matrix4x4::operator*(double s) const {
    Matrix4x4 result;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            result.data[i][j] = data[i][j] * s;
    return result;
}

Matrix4x4 Matrix4x4::inv() const {
    Matrix4x4 result;
    // Simplified block-diagonal inversion
    Matrix2x2 block_pos, block_vel;
    block_pos.data[0][0] = data[0][0]; block_pos.data[0][1] = data[0][1];
    block_pos.data[1][0] = data[1][0]; block_pos.data[1][1] = data[1][1];
    block_vel.data[0][0] = data[2][2]; block_vel.data[0][1] = data[2][3];
    block_vel.data[1][0] = data[3][2]; block_vel.data[1][1] = data[3][3];
    
    Matrix2x2 inv_pos = block_pos.inv();
    Matrix2x2 inv_vel = block_vel.inv();
    
    result.data[0][0] = inv_pos.data[0][0]; result.data[0][1] = inv_pos.data[0][1];
    result.data[1][0] = inv_pos.data[1][0]; result.data[1][1] = inv_pos.data[1][1];
    result.data[2][2] = inv_vel.data[0][0]; result.data[2][3] = inv_vel.data[0][1];
    result.data[3][2] = inv_vel.data[1][0]; result.data[3][3] = inv_vel.data[1][1];
    
    return result;
}

Vec4::Vec4() {
    data[0]=0; data[1]=0; data[2]=0; data[3]=0;
}

Vec4::Vec4(double x, double y, double vx, double vy) {
    data[0]=x; data[1]=y; data[2]=vx; data[3]=vy;
}

//================================================================
// ClassTracker Implementation
//================================================================

ClassTracker::ClassTracker() {
    m_class_probs["unknown"] = 1.0;
}

void ClassTracker::updateClass(const string& detected_class, double confidence) {
    double likelihood = confidence;
    
    if(m_class_probs.find(detected_class) == m_class_probs.end()) {
        m_class_probs[detected_class] = 0.01;
    }
    m_class_probs[detected_class] *= likelihood;
    
    // Normalize
    double sum = 0.0;
    for(auto& kv : m_class_probs) sum += kv.second;
    if(sum > 0) {
        for(auto& kv : m_class_probs) kv.second /= sum;
    }
}

string ClassTracker::getMostLikelyClass() const {
    double max_prob = 0;
    string best_class = "unknown";
    for(const auto& kv : m_class_probs) {
        if(kv.second > max_prob) {
            max_prob = kv.second;
            best_class = kv.first;
        }
    }
    return best_class;
}

double ClassTracker::getClassProbability(const string& class_name) const {
    auto it = m_class_probs.find(class_name);
    return (it != m_class_probs.end()) ? it->second : 0.0;
}

void ClassTracker::decay(double decay_rate) {
    for(auto& kv : m_class_probs) {
        kv.second *= (1.0 - decay_rate);
    }
    for(auto& kv : m_class_probs) {
        kv.second += decay_rate / m_class_probs.size();
    }
}

//================================================================
// Track Implementation
//================================================================

Track::Track(int id, double x, double y, double vx, double vy, double time)
    : m_id(id), m_state(x, y, vx, vy), m_confirmed(false),
      m_hit_count(1), m_scan_count(1), m_consecutive_misses(0),
      m_quality(0.5), m_last_update_time(time),
      m_has_radar(false), m_has_ais(false), m_has_vision(false),
      m_last_source("unknown"),
      m_has_mmsi(false), m_mmsi("")
{
    m_P = Matrix4x4::identity();
    m_P.data[0][0] = 100.0;
    m_P.data[1][1] = 100.0;
    m_P.data[2][2] = 25.0;
    m_P.data[3][3] = 25.0;
    
    m_mode_probs[MODEL_CV] = 0.5;
    m_mode_probs[MODEL_CT] = 0.5;
    m_state_modes[MODEL_CV] = m_state;
    m_state_modes[MODEL_CT] = m_state;
    m_P_modes[MODEL_CV] = m_P;
    m_P_modes[MODEL_CT] = m_P;
}

void Track::predict(double dt, const array<double,2>& process_noise_cv,
                    const array<double,2>& process_noise_ct)
{
    if(dt <= 0 || dt > 10.0) return;
    
    Matrix4x4 F = Matrix4x4::identity();
    F.data[0][2] = dt;
    F.data[1][3] = dt;
    
    Matrix4x4 Q_cv = Matrix4x4::identity();
    Q_cv.data[0][0] = process_noise_cv[0] * process_noise_cv[0] * dt*dt*dt / 3.0;
    Q_cv.data[1][1] = process_noise_cv[1] * process_noise_cv[1] * dt*dt*dt / 3.0;
    Q_cv.data[2][2] = process_noise_cv[0] * process_noise_cv[0] * dt;
    Q_cv.data[3][3] = process_noise_cv[1] * process_noise_cv[1] * dt;
    Q_cv.data[0][2] = process_noise_cv[0] * process_noise_cv[0] * dt*dt / 2.0;
    Q_cv.data[2][0] = Q_cv.data[0][2];
    Q_cv.data[1][3] = process_noise_cv[1] * process_noise_cv[1] * dt*dt / 2.0;
    Q_cv.data[3][1] = Q_cv.data[1][3];
    
    Matrix4x4 Q_ct = Q_cv * 4.0;
    
    for(int m = 0; m < 2; m++) {
        Vec4 x = m_state_modes[m];
        Matrix4x4 P = m_P_modes[m];
        
        Vec4 x_pred;
        for(int i=0; i<4; i++) {
            x_pred[i] = 0;
            for(int j=0; j<4; j++)
                x_pred[i] += F.data[i][j] * x[j];
        }
        
        Matrix4x4 FP;
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                for(int k=0; k<4; k++)
                    FP.data[i][j] += F.data[i][k] * P.data[k][j];
        
        Matrix4x4 P_pred;
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                for(int k=0; k<4; k++)
                    P_pred.data[i][j] += FP.data[i][k] * F.data[j][k];
        
        Matrix4x4 Q = (m == MODEL_CV) ? Q_cv : Q_ct;
        P_pred = P_pred + Q;
        
        m_state_modes[m] = x_pred;
        m_P_modes[m] = P_pred;
    }
    
    m_state[0] = m_mode_probs[0] * m_state_modes[0][0] + m_mode_probs[1] * m_state_modes[1][0];
    m_state[1] = m_mode_probs[0] * m_state_modes[0][1] + m_mode_probs[1] * m_state_modes[1][1];
    m_state[2] = m_mode_probs[0] * m_state_modes[0][2] + m_mode_probs[1] * m_state_modes[1][2];
    m_state[3] = m_mode_probs[0] * m_state_modes[0][3] + m_mode_probs[1] * m_state_modes[1][3];
    
    m_P = m_P_modes[0] * m_mode_probs[0] + m_P_modes[1] * m_mode_probs[1];
}

void Track::updateRadar(double x_meas, double y_meas,
                        const array<double,2>& meas_noise)
{
    double innov_x = x_meas - m_state[0];
    double innov_y = y_meas - m_state[1];
    
    double S_xx = m_P.data[0][0] + meas_noise[0]*meas_noise[0];
    double S_yy = m_P.data[1][1] + meas_noise[1]*meas_noise[1];
    double S_xy = m_P.data[0][1];
    
    double det_S = S_xx * S_yy - S_xy * S_xy;
    if(fabs(det_S) < 1e-6) return;
    
    double inv_S[2][2];
    inv_S[0][0] = S_yy / det_S;
    inv_S[0][1] = -S_xy / det_S;
    inv_S[1][0] = -S_xy / det_S;
    inv_S[1][1] = S_xx / det_S;
    
    double K[4][2];
    for(int i=0; i<4; i++) {
        K[i][0] = m_P.data[i][0] * inv_S[0][0] + m_P.data[i][1] * inv_S[1][0];
        K[i][1] = m_P.data[i][0] * inv_S[0][1] + m_P.data[i][1] * inv_S[1][1];
    }
    
    m_state[0] += K[0][0] * innov_x + K[0][1] * innov_y;
    m_state[1] += K[1][0] * innov_x + K[1][1] * innov_y;
    m_state[2] += K[2][0] * innov_x + K[2][1] * innov_y;
    m_state[3] += K[3][0] * innov_x + K[3][1] * innov_y;
    
    Matrix4x4 I = Matrix4x4::identity();
    for(int i=0; i<4; i++) {
        I.data[i][0] -= K[i][0];
        I.data[i][1] -= K[i][1];
    }
    
    Matrix4x4 P_new;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            for(int k=0; k<4; k++)
                P_new.data[i][j] += I.data[i][k] * m_P.data[k][j];
    
    m_P = P_new;
    
    m_state_modes[MODEL_CV] = m_state;
    m_state_modes[MODEL_CT] = m_state;
    m_P_modes[MODEL_CV] = m_P;
    m_P_modes[MODEL_CT] = m_P;
}

void Track::updateAIS(double x_meas, double y_meas, double vx_ais, double vy_ais,
                      const array<double,2>& meas_noise,
                      const array<double,2>& vel_noise)
{
    Vec4 z_meas(x_meas, y_meas, vx_ais, vy_ais);
    Vec4 innov;
    for(int i=0; i<4; i++)
        innov[i] = z_meas[i] - m_state[i];
    
    Matrix4x4 R;
    R.data[0][0] = meas_noise[0] * meas_noise[0];
    R.data[1][1] = meas_noise[1] * meas_noise[1];
    R.data[2][2] = vel_noise[0] * vel_noise[0];
    R.data[3][3] = vel_noise[1] * vel_noise[1];
    
    Matrix4x4 S = m_P + R;
    Matrix4x4 inv_S = S.inv();
    Matrix4x4 K = m_P * inv_S;
    
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            m_state[i] += K.data[i][j] * innov[j];
    
    Matrix4x4 I = Matrix4x4::identity();
    Matrix4x4 I_K;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            I_K.data[i][j] = I.data[i][j] - K.data[i][j];
    
    m_P = I_K * m_P;
    
    m_state_modes[MODEL_CV] = m_state;
    m_state_modes[MODEL_CT] = m_state;
    m_P_modes[MODEL_CV] = m_P;
    m_P_modes[MODEL_CT] = m_P;
}

void Track::updateVisionBearing(double bearing_meas_rad, double ownship_heading_rad,
                                double ownship_x, double ownship_y,
                                double bearing_noise_rad)
{
    // Bearing-only EKF update
    double dx = m_state[0] - ownship_x;
    double dy = m_state[1] - ownship_y;
    
    double bearing_pred_rad = atan2(dx, dy);
    
    double innov = bearing_meas_rad - bearing_pred_rad;
    while(innov > M_PI) innov -= 2*M_PI;
    while(innov < -M_PI) innov += 2*M_PI;
    
    // Jacobian: H = [dh/dx, dh/dy, 0, 0]
    double range_sq = dx*dx + dy*dy;
    if(range_sq < 1.0) range_sq = 1.0;
    
    double H[4];
    H[0] = -dy / range_sq;
    H[1] = dx / range_sq;
    H[2] = 0;
    H[3] = 0;
    
    // Innovation covariance
    double S = 0;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            S += H[i] * m_P.data[i][j] * H[j];
    S += bearing_noise_rad * bearing_noise_rad;
    
    if(fabs(S) < 1e-10) return;
    
    // Kalman gain
    double K[4];
    for(int i=0; i<4; i++) {
        K[i] = 0;
        for(int j=0; j<4; j++)
            K[i] += m_P.data[i][j] * H[j];
        K[i] /= S;
    }
    
    // State update
    for(int i=0; i<4; i++)
        m_state[i] += K[i] * innov;
    
    // Covariance update
    Matrix4x4 I = Matrix4x4::identity();
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            I.data[i][j] -= K[i] * H[j];
    
    Matrix4x4 P_new;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            for(int k=0; k<4; k++)
                P_new.data[i][j] += I.data[i][k] * m_P.data[k][j];
    
    m_P = P_new;
    
    m_state_modes[MODEL_CV] = m_state;
    m_state_modes[MODEL_CT] = m_state;
    m_P_modes[MODEL_CV] = m_P;
    m_P_modes[MODEL_CT] = m_P;
}

void Track::updateVisionClass(const string& detected_class, double confidence) {
    m_class_tracker.updateClass(detected_class, confidence);
}

double Track::mahalanobisDistance(double x_meas, double y_meas) const {
    double dx = x_meas - m_state[0];
    double dy = y_meas - m_state[1];
    
    double S_xx = m_P.data[0][0];
    double S_yy = m_P.data[1][1];
    double S_xy = m_P.data[0][1];
    
    double det_S = S_xx * S_yy - S_xy * S_xy;
    if(fabs(det_S) < 1e-10) return 1e10;
    
    double inv_S_xx = S_yy / det_S;
    double inv_S_yy = S_xx / det_S;
    double inv_S_xy = -S_xy / det_S;
    
    double dist_sq = dx*dx*inv_S_xx + 2.0*dx*dy*inv_S_xy + dy*dy*inv_S_yy;
    return sqrt(fabs(dist_sq));
}

double Track::angularDistance(double bearing_meas_rad, double ownship_heading_rad,
                              double ownship_x, double ownship_y) const {
    double dx = m_state[0] - ownship_x;
    double dy = m_state[1] - ownship_y;
    double bearing_pred_rad = atan2(dx, dy);
    
    double diff = bearing_meas_rad - bearing_pred_rad;
    while(diff > M_PI) diff -= 2*M_PI;
    while(diff < -M_PI) diff += 2*M_PI;
    
    return fabs(diff);
}

void Track::updateQuality(double delta) {
    m_quality += delta;
    if(m_quality > 1.0) m_quality = 1.0;
    if(m_quality < 0.0) m_quality = 0.0;
}

double Track::getClassConfidence() const {
    return m_class_tracker.getClassProbability(m_class_tracker.getMostLikelyClass());
}

//================================================================
// FusionConfig Implementation
//================================================================

FusionConfig::FusionConfig() {
    radar_cluster_eps_m = 20.0;
    radar_cluster_min_pts = 1;
    
    radar_noise_xy = {5.0, 5.0};
    ais_pos_noise = {10.0, 10.0};
    ais_vel_noise = {0.5, 0.5};
    vision_bearing_noise_deg = 2.0;
    
    process_noise_cv = {0.5, 0.5};
    process_noise_ct = {2.0, 2.0};
    
    gate_threshold = 9.0;
    bearing_gate_threshold = 10.0;
    
    confirm_min_hits = 3;
    confirm_min_scans = 5;
    delete_max_misses = 5;
    delete_min_quality = 0.1;
    quality_decay_rate = 0.05;
    quality_increment = 0.15;
    max_coast_time_s = 5.0;
    
    vision_can_initiate_tracks = false;
    vision_only_max_scans = 10;
    class_decay_rate = 0.01;
    
    camera_fx = 1000.0;
    camera_cx = 640.0;
    camera_yaw_offset_deg = 0.0;
    
    publish_rate_hz = 2.0;
    publish_debug = false;
}

//================================================================
// Multisensorfusion Implementation
//================================================================

Multisensorfusion::Multisensorfusion()
{
    m_geodesy_initialized = false;
    m_origin_lat = 42.3601;  // Default: Boston
    m_origin_lon = -71.0589;
    
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_heading = 0;
    m_nav_speed = 0;
    m_nav_lat = 0;
    m_nav_lon = 0;
    m_nav_valid = false;
    
    m_next_track_id = 1;
    m_last_publish_time = 0;
    m_last_iterate_time = 0;
    
    m_total_radar_updates = 0;
    m_total_ais_reports = 0;
    m_total_vision_reports = 0;
    m_radar_associations = 0;
    m_ais_associations = 0;
    m_vision_associations = 0;
}

Multisensorfusion::~Multisensorfusion()
{
}

bool Multisensorfusion::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);

    MOOSMSG_LIST::iterator p;
    for(p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key   = msg.GetKey();
        double mtime = msg.GetTime();

        if(key == "RADAR_TARGETS") {
            handleRadarTargets(msg.GetString(), mtime);
        }
        else if(key == "AIS_REPORT") {
            handleAISReport(msg.GetString(), mtime);
        }
        else if(key == "VISION_REPORT") {
            handleVisionReport(msg.GetString(), mtime);
        }
        else if(key == "NAV_X" || key == "NAV_Y" || 
                key == "NAV_HEADING" || key == "NAV_SPEED") {
            handleNavUpdate(key, msg.GetDouble(), mtime);
        }
        else if(key == "NAV_LAT") {
            m_nav_lat = msg.GetDouble();
        }
        else if(key == "NAV_LON") {
            m_nav_lon = msg.GetDouble();
        }
        else if(key != "APPCAST_REQ") {
            reportRunWarning("Unhandled Mail: " + key);
        }
    }

    return true;
}

bool Multisensorfusion::Iterate()
{
    AppCastingMOOSApp::Iterate();
    
    double current_time = MOOSTime();
    
    if(m_last_iterate_time > 0) {
        double dt = current_time - m_last_iterate_time;
        if(dt > 0.01) {
            predictTracks(current_time);
            manageTracks();
        }
    }
    
    if(current_time - m_last_publish_time > 1.0 / m_config.publish_rate_hz) {
        publishContacts(current_time);
        if(m_config.publish_debug)
            publishDebug();
        m_last_publish_time = current_time;
    }
    
    m_last_iterate_time = current_time;
    
    AppCastingMOOSApp::PostReport();
    return true;
}

bool Multisensorfusion::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();

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
        
        if(param == "radar_cluster_eps_m") {
            m_config.radar_cluster_eps_m = atof(value.c_str());
            handled = true;
        }
        else if(param == "radar_noise_x") {
            m_config.radar_noise_xy[0] = atof(value.c_str());
            handled = true;
        }
        else if(param == "radar_noise_y") {
            m_config.radar_noise_xy[1] = atof(value.c_str());
            handled = true;
        }
        else if(param == "ais_pos_noise") {
            m_config.ais_pos_noise[0] = m_config.ais_pos_noise[1] = atof(value.c_str());
            handled = true;
        }
        else if(param == "ais_vel_noise") {
            m_config.ais_vel_noise[0] = m_config.ais_vel_noise[1] = atof(value.c_str());
            handled = true;
        }
        else if(param == "vision_bearing_noise_deg") {
            m_config.vision_bearing_noise_deg = atof(value.c_str());
            handled = true;
        }
        else if(param == "process_noise_cv") {
            m_config.process_noise_cv[0] = m_config.process_noise_cv[1] = atof(value.c_str());
            handled = true;
        }
        else if(param == "gate_threshold") {
            m_config.gate_threshold = atof(value.c_str());
            handled = true;
        }
        else if(param == "bearing_gate_threshold") {
            m_config.bearing_gate_threshold = atof(value.c_str());
            handled = true;
        }
        else if(param == "confirm_min_hits") {
            m_config.confirm_min_hits = atoi(value.c_str());
            handled = true;
        }
        else if(param == "delete_max_misses") {
            m_config.delete_max_misses = atoi(value.c_str());
            handled = true;
        }
        else if(param == "vision_can_initiate_tracks") {
            m_config.vision_can_initiate_tracks = (tolower(value) == "true");
            handled = true;
        }
        else if(param == "camera_fx") {
            m_config.camera_fx = atof(value.c_str());
            handled = true;
        }
        else if(param == "camera_cx") {
            m_config.camera_cx = atof(value.c_str());
            handled = true;
        }
        else if(param == "camera_yaw_offset_deg") {
            m_config.camera_yaw_offset_deg = atof(value.c_str());
            handled = true;
        }
        else if(param == "publish_rate_hz") {
            m_config.publish_rate_hz = atof(value.c_str());
            handled = true;
        }
        else if(param == "publish_debug") {
            m_config.publish_debug = (tolower(value) == "true");
            handled = true;
        }
        else if(param == "lat_origin") {
            m_origin_lat = atof(value.c_str());
            handled = true;
        }
        else if(param == "lon_origin") {
            m_origin_lon = atof(value.c_str());
            handled = true;
        }
        
        // Initialize geodesy if both lat and lon are set
        if(param == "lat_origin" || param == "lon_origin") {
#ifdef USE_MOOSGEODESY
            m_geodesy.Initialise(m_origin_lat, m_origin_lon);
#endif
            m_geodesy_initialized = true;
        }

        if(!handled)
            reportUnhandledConfigWarning(orig);
    }

    registerVariables();    
    return true;
}

bool Multisensorfusion::OnConnectToServer()
{
    registerVariables();
    return true;
}

void Multisensorfusion::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();
    Register("RADAR_TARGETS", 0);
    Register("AIS_REPORT", 0);
    Register("VISION_REPORT", 0);
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_LAT", 0);
    Register("NAV_LON", 0);
    Register("NAV_HEADING", 0);
    Register("NAV_SPEED", 0);
}

bool Multisensorfusion::buildReport() 
{
    m_msgs << "============================================" << endl;
    m_msgs << "pMultisensorfusion - Multi-Sensor Fusion  " << endl;
    m_msgs << "============================================" << endl;
    
    m_msgs << "Configuration:" << endl;
    m_msgs << "  Vision bearing-only: YES (design choice)" << endl;
    m_msgs << "  Geodesy initialized: " << (m_geodesy_initialized ? "YES" : "NO") << endl;
    m_msgs << endl;
    
    m_msgs << "Sensor Updates:" << endl;
    m_msgs << "  Radar:  " << m_total_radar_updates << endl;
    m_msgs << "  AIS:    " << m_total_ais_reports << endl;
    m_msgs << "  Vision: " << m_total_vision_reports << endl;
    m_msgs << endl;
    
    m_msgs << "Associations:" << endl;
    m_msgs << "  Radar:  " << m_radar_associations << endl;
    m_msgs << "  AIS:    " << m_ais_associations << endl;
    m_msgs << "  Vision: " << m_vision_associations << endl;
    m_msgs << endl;
    
    m_msgs << "Tracks: " << m_tracks.size() << " total, ";
    int confirmed = 0;
    for(auto& kv : m_tracks) {
        if(kv.second->isConfirmed()) confirmed++;
    }
    m_msgs << confirmed << " confirmed" << endl;
    
    if(confirmed > 0) {
        m_msgs << endl;
        ACTable actab(6);
        actab << "ID | Position | Speed | Sources | Class | MMSI";
        actab.addHeaderLines();
        
        for(auto& kv : m_tracks) {
            auto track = kv.second;
            if(!track->isConfirmed()) continue;
            
            Vec4 state = track->getState();
            double speed = sqrt(state[2]*state[2] + state[3]*state[3]);
            
            string sources;
            if(track->hasRadar()) sources += "R";
            if(track->hasAIS()) sources += "A";
            if(track->hasVision()) sources += "V";
            
            string pos = "(" + intToString((int)state[0]) + "," + 
                        intToString((int)state[1]) + ")";
            string spd = doubleToString(speed, 1) + " m/s";
            string cls = track->getClassification();
            string mmsi = track->hasMMSI() ? track->getMMSI() : "-";
            
            actab << track->getID() << pos << spd << sources << cls << mmsi;
        }
        m_msgs << actab.getFormattedString() << endl;
    }
    
    return true;
}

//================================================================
// Message Handlers
//================================================================

void Multisensorfusion::handleRadarTargets(const string& msg, double moos_time) {
    if(!m_nav_valid) return;
    
    vector<RadarDetection> detections = parseRadarTargets(msg);
    if(detections.empty()) return;
    
    associateRadar(detections, moos_time);
    m_total_radar_updates++;
}

void Multisensorfusion::handleAISReport(const string& msg, double moos_time) {
    vector<AISReport> reports = parseAISReports(msg);
    if(reports.empty()) return;
    
    convertAISToLocal(reports);
    associateAIS(reports, moos_time);
    m_total_ais_reports++;
}

void Multisensorfusion::handleVisionReport(const string& msg, double moos_time) {
    if(!m_nav_valid) return;
    
    vector<VisionDetection> detections = parseVisionReports(msg);
    if(detections.empty()) return;
    
    associateVision(detections, moos_time);
    m_total_vision_reports++;
}

void Multisensorfusion::handleNavUpdate(const string& var, double val, double time) {
    if(var == "NAV_X") m_nav_x = val;
    else if(var == "NAV_Y") m_nav_y = val;
    else if(var == "NAV_HEADING") m_nav_heading = val;
    else if(var == "NAV_SPEED") m_nav_speed = val;
    
    m_nav_valid = true;
}

//================================================================
// Message Parsers
//================================================================

vector<RadarDetection> Multisensorfusion::parseRadarTargets(const string& msg) {
    vector<RadarDetection> detections;
    
    stringstream ss(msg);
    string target_str;
    
    while(getline(ss, target_str, ';')) {
        if(target_str.empty()) continue;
        
        RadarDetection det;
        det.x_m = 0;
        det.y_m = 0;
        det.vx_ms = 0;
        det.vy_ms = 0;
        det.range_m = 0;
        det.bearing_deg = 0;
        det.timestamp = 0;
        det.has_velocity = false;
        
        stringstream target_ss(target_str);
        string pair;
        while(getline(target_ss, pair, ',')) {
            size_t eq_pos = pair.find('=');
            if(eq_pos == string::npos) continue;
            
            string key = pair.substr(0, eq_pos);
            string value = pair.substr(eq_pos + 1);
            
            if(key == "x") det.x_m = atof(value.c_str());
            else if(key == "y") det.y_m = atof(value.c_str());
            else if(key == "vx") {
                det.vx_ms = atof(value.c_str());
                det.has_velocity = true;
            }
            else if(key == "vy") {
                det.vy_ms = atof(value.c_str());
                det.has_velocity = true;
            }
            else if(key == "range") det.range_m = atof(value.c_str());
            else if(key == "bearing") det.bearing_deg = atof(value.c_str());
            else if(key == "label") det.label = value;
        }
        
        det.timestamp = MOOSTime();
        if(det.range_m > 0 || (det.x_m != 0 || det.y_m != 0))
            detections.push_back(det);
    }
    
    return detections;
}

vector<AISReport> Multisensorfusion::parseAISReports(const string& msg) {
    vector<AISReport> reports;
    
    stringstream ss(msg);
    string report_str;
    
    while(getline(ss, report_str, '#')) {
        if(report_str.empty()) continue;
        
        AISReport report;
        report.mmsi = "";
        report.lat_deg = 0;
        report.lon_deg = 0;
        report.sog_knots = 0;
        report.cog_deg = 0;
        report.heading_deg = 0;
        report.timestamp = 0;
        report.has_heading = false;
        
        stringstream report_ss(report_str);
        string pair;
        while(getline(report_ss, pair, ',')) {
            size_t eq_pos = pair.find('=');
            if(eq_pos == string::npos) continue;
            
            string key = pair.substr(0, eq_pos);
            string value = pair.substr(eq_pos + 1);
            
            if(key == "mmsi") report.mmsi = value;
            else if(key == "lat") report.lat_deg = atof(value.c_str());
            else if(key == "lon") report.lon_deg = atof(value.c_str());
            else if(key == "sog") report.sog_knots = atof(value.c_str());
            else if(key == "cog") report.cog_deg = atof(value.c_str());
            else if(key == "heading") {
                report.heading_deg = atof(value.c_str());
                report.has_heading = true;
            }
            else if(key == "time") report.timestamp = atof(value.c_str());
        }
        
        if(!report.mmsi.empty())
            reports.push_back(report);
    }
    
    return reports;
}

vector<VisionDetection> Multisensorfusion::parseVisionReports(const string& msg) {
    vector<VisionDetection> detections;
    
    stringstream ss(msg);
    string det_str;
    
    while(getline(ss, det_str, ';')) {
        if(det_str.empty()) continue;
        
        VisionDetection det;
        det.classname = "unknown";
        det.confidence = 0;
        det.bearing_deg = 0;
        det.timestamp = 0;
        det.has_range = false;
        det.range_m = 0;
        det.has_bbox = false;
        det.bbox_center_u = 0;
        det.image_width = 1280;
        
        bool has_bearing = false;
        
        stringstream det_ss(det_str);
        string pair;
        while(getline(det_ss, pair, ',')) {
            size_t eq_pos = pair.find('=');
            if(eq_pos == string::npos) continue;
            
            string key = pair.substr(0, eq_pos);
            string value = pair.substr(eq_pos + 1);
            
            if(key == "class" || key == "classname") det.classname = value;
            else if(key == "confidence") det.confidence = atof(value.c_str());
            else if(key == "bearing" || key == "bearing_deg") {
                det.bearing_deg = atof(value.c_str());
                has_bearing = true;
            }
            else if(key == "pixel_u" || key == "bbox_center_u") {
                det.bbox_center_u = atoi(value.c_str());
                det.has_bbox = true;
            }
            else if(key == "img_width" || key == "image_width") {
                det.image_width = atoi(value.c_str());
            }
            else if(key == "time") det.timestamp = atof(value.c_str());
        }
        
        if(!has_bearing && det.has_bbox) {
            det.bearing_deg = computeBearingFromPixel(det.bbox_center_u, det.image_width);
        }
        
        det.timestamp = (det.timestamp > 0) ? det.timestamp : MOOSTime();
        detections.push_back(det);
    }
    
    return detections;
}

//================================================================
// Coordinate Transforms
//================================================================

void Multisensorfusion::convertAISToLocal(vector<AISReport>& reports) {
    if(!m_geodesy_initialized) return;
    
    for(auto& report : reports) {
        double x, y;
        
#ifdef USE_MOOSGEODESY
        // Use MOOSGeodesy if available
        m_geodesy.LatLong2LocalUTM(report.lat_deg, report.lon_deg, y, x);
        report.x_m = x;
        report.y_m = y;
#else
        // Simple tangent plane approximation (good for <50km)
        const double earth_radius = 6378137.0;  // WGS84 equatorial radius
        double lat_rad = report.lat_deg * M_PI / 180.0;
        double lon_rad = report.lon_deg * M_PI / 180.0;
        double origin_lat_rad = m_origin_lat * M_PI / 180.0;
        double origin_lon_rad = m_origin_lon * M_PI / 180.0;
        
        double dlat = lat_rad - origin_lat_rad;
        double dlon = lon_rad - origin_lon_rad;
        
        // Local coordinates
        report.x_m = earth_radius * dlon * cos(origin_lat_rad);
        report.y_m = earth_radius * dlat;
#endif
    }
}

double Multisensorfusion::computeBearingFromPixel(int pixel_u, int image_width) {
    double u_centered = pixel_u - m_config.camera_cx;
    double bearing_camera_rad = atan(u_centered / m_config.camera_fx);
    double bearing_ownship_rad = bearing_camera_rad + m_config.camera_yaw_offset_deg * M_PI / 180.0;
    return bearing_ownship_rad * 180.0 / M_PI;
}

//================================================================
// Tracking Pipeline
//================================================================

void Multisensorfusion::predictTracks(double current_time) {
    for(auto& kv : m_tracks) {
        auto track = kv.second;
        double dt = current_time - track->getLastUpdateTime();
        if(dt > 0 && dt < 10.0) {
            track->predict(dt, m_config.process_noise_cv, m_config.process_noise_ct);
            track->setLastUpdateTime(current_time);
        }
    }
}

void Multisensorfusion::associateRadar(const vector<RadarDetection>& detections, double time) {
    vector<bool> det_assigned(detections.size(), false);
    map<int, bool> track_updated;
    
    vector<pair<double, pair<int, int>>> associations;
    
    for(auto& kv : m_tracks) {
        auto track = kv.second;
        for(size_t i = 0; i < detections.size(); i++) {
            double dist = track->mahalanobisDistance(detections[i].x_m, detections[i].y_m);
            if(dist < m_config.gate_threshold) {
                associations.push_back(make_pair(dist, make_pair(track->getID(), i)));
            }
        }
    }
    
    sort(associations.begin(), associations.end());
    
    for(auto& assoc : associations) {
        int track_id = assoc.second.first;
        int det_idx = assoc.second.second;
        
        if(det_assigned[det_idx] || track_updated[track_id])
            continue;
        
        auto track = m_tracks[track_id];
        track->updateRadar(detections[det_idx].x_m, detections[det_idx].y_m,
                          m_config.radar_noise_xy);
        track->resetMissCount();
        track->incrementHitCount();
        track->updateQuality(m_config.quality_increment);
        track->setRadar(true);
        track->setLastSource("radar");
        
        det_assigned[det_idx] = true;
        track_updated[track_id] = true;
        m_radar_associations++;
    }
    
    for(auto& kv : m_tracks) {
        if(!track_updated[kv.first]) {
            kv.second->incrementMissCount();
            kv.second->incrementScanCount();
            kv.second->updateQuality(-m_config.quality_decay_rate);
        }
    }
    
    for(size_t i = 0; i < detections.size(); i++) {
        if(!det_assigned[i]) {
            double vx = detections[i].has_velocity ? detections[i].vx_ms : 0.0;
            double vy = detections[i].has_velocity ? detections[i].vy_ms : 0.0;
            
            auto new_track = make_shared<Track>(m_next_track_id++,
                                                detections[i].x_m, detections[i].y_m,
                                                vx, vy, time);
            new_track->setRadar(true);
            new_track->setLastSource("radar");
            m_tracks[new_track->getID()] = new_track;
        }
    }
}

void Multisensorfusion::associateAIS(const vector<AISReport>& reports, double time) {
    for(auto& ais : reports) {
        double sog_ms = ais.sog_knots * 0.514444;
        double cog_rad = ais.cog_deg * M_PI / 180.0;
        double vx_ais = sog_ms * sin(cog_rad);
        double vy_ais = sog_ms * cos(cog_rad);
        
        double best_dist = 1e10;
        shared_ptr<Track> best_track = nullptr;
        
        for(auto& kv : m_tracks) {
            auto track = kv.second;
            double dist = track->mahalanobisDistance(ais.x_m, ais.y_m);
            
            if(dist < m_config.gate_threshold * 2.0 && dist < best_dist) {
                best_dist = dist;
                best_track = track;
            }
        }
        
        if(best_track) {
            best_track->updateAIS(ais.x_m, ais.y_m, vx_ais, vy_ais,
                                 m_config.ais_pos_noise, m_config.ais_vel_noise);
            best_track->setMMSI(ais.mmsi);
            best_track->resetMissCount();
            best_track->updateQuality(m_config.quality_increment * 1.5);
            best_track->setAIS(true);
            best_track->setLastSource("ais");
            m_ais_associations++;
        } else {
            auto new_track = make_shared<Track>(m_next_track_id++,
                                                ais.x_m, ais.y_m,
                                                vx_ais, vy_ais, time);
            new_track->setMMSI(ais.mmsi);
            new_track->setConfirmed(true);
            new_track->setAIS(true);
            new_track->setLastSource("ais");
            m_tracks[new_track->getID()] = new_track;
        }
    }
}

void Multisensorfusion::associateVision(const vector<VisionDetection>& detections, double time) {
    for(auto& det : detections) {
        double bearing_meas_rad = (m_nav_heading + det.bearing_deg) * M_PI / 180.0;
        double bearing_noise_rad = m_config.vision_bearing_noise_deg * M_PI / 180.0;
        
        vector<pair<double, shared_ptr<Track>>> candidates;
        
        for(auto& kv : m_tracks) {
            auto track = kv.second;
            double ang_dist = track->angularDistance(bearing_meas_rad, 
                                                     m_nav_heading * M_PI / 180.0,
                                                     m_nav_x, m_nav_y);
            
            double ang_gate = m_config.bearing_gate_threshold * M_PI / 180.0;
            if(ang_dist < ang_gate) {
                candidates.push_back(make_pair(ang_dist, track));
            }
        }
        
        if(!candidates.empty()) {
            sort(candidates.begin(), candidates.end());
            auto track = candidates[0].second;
            
            track->updateVisionBearing(bearing_meas_rad,
                                      m_nav_heading * M_PI / 180.0,
                                      m_nav_x, m_nav_y,
                                      bearing_noise_rad);
            track->updateVisionClass(det.classname, det.confidence);
            track->setVision(true);
            track->setLastSource("vision");
            m_vision_associations++;
        }
    }
}

void Multisensorfusion::manageTracks() {
    vector<int> tracks_to_delete;
    
    for(auto& kv : m_tracks) {
        auto track = kv.second;
        
        if(!track->isConfirmed()) {
            if(track->getHitCount() >= m_config.confirm_min_hits &&
               track->getScanCount() >= m_config.confirm_min_scans) {
                track->setConfirmed(true);
            }
        }
        
        if(track->isVisionOnly() && 
           track->getScanCount() > m_config.vision_only_max_scans) {
            tracks_to_delete.push_back(kv.first);
            continue;
        }
        
        if(track->getMissCount() > m_config.delete_max_misses ||
           track->getQuality() < m_config.delete_min_quality) {
            tracks_to_delete.push_back(kv.first);
        }
    }
    
    for(int id : tracks_to_delete) {
        m_tracks.erase(id);
    }
}

//================================================================
// Publishing
//================================================================

void Multisensorfusion::publishContacts(double time) {
    for(auto& kv : m_tracks) {
        auto track = kv.second;
        
        if(!track->isConfirmed()) continue;
        if(track->isVisionOnly()) continue;
        
        string node_report = generateNodeReport(*track, time);
        Notify("NODE_REPORT", node_report);
    }
}

void Multisensorfusion::publishDebug() {
    stringstream ss;
    
    // Count confirmed tracks
    int confirmed = 0;
    for(const auto& kv : m_tracks) {
        if(kv.second->isConfirmed()) confirmed++;
    }
    
    ss << "tracks=" << m_tracks.size()
       << ",confirmed=" << confirmed
       << ",radar=" << m_total_radar_updates
       << ",ais=" << m_total_ais_reports
       << ",vision=" << m_total_vision_reports
       << ",r_assoc=" << m_radar_associations
       << ",a_assoc=" << m_ais_associations
       << ",v_assoc=" << m_vision_associations;
    
    Notify("FUSION_DEBUG", ss.str());
}

string Multisensorfusion::generateNodeReport(const Track& track, double time) {
    Vec4 state = track.getState();
    double x = state[0];
    double y = state[1];
    double vx = state[2];
    double vy = state[3];
    
    double speed = sqrt(vx*vx + vy*vy);
    double heading = atan2(vx, vy) * 180.0 / M_PI;
    if(heading < 0) heading += 360.0;
    
    stringstream ss;
    
    if(track.hasMMSI()) {
        ss << "NAME=MMSI_" << track.getMMSI();
    } else {
        ss << "NAME=RADAR_" << track.getID();
    }
    
    ss << ",X=" << doubleToStringX(x, 2);
    ss << ",Y=" << doubleToStringX(y, 2);
    ss << ",SPD=" << doubleToStringX(speed, 2);
    ss << ",HDG=" << doubleToStringX(heading, 1);
    ss << ",TIME=" << doubleToStringX(time, 2);
    
    string class_name = track.getClassification();
    if(class_name != "unknown") {
        ss << ",TYPE=" << class_name;
    } else {
        ss << ",TYPE=KAYAK";
    }
    
    string sources;
    if(track.hasRadar()) sources += "R";
    if(track.hasAIS()) sources += "A";
    if(track.hasVision()) sources += "V";
    ss << ",SOURCE=" << sources;
    
    if(track.hasMMSI()) {
        ss << ",MMSI=" << track.getMMSI();
    }
    
    return ss.str();
}

//================================================================
// Utility Functions
//================================================================

double Multisensorfusion::normalizeAngle(double angle_deg) {
    while(angle_deg > 360.0) angle_deg -= 360.0;
    while(angle_deg < 0.0) angle_deg += 360.0;
    return angle_deg;
}

double Multisensorfusion::wrapAngle(double angle_rad) {
    while(angle_rad > M_PI) angle_rad -= 2*M_PI;
    while(angle_rad < -M_PI) angle_rad += 2*M_PI;
    return angle_rad;
}
