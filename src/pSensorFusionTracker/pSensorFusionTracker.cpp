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

using namespace std;

// Constructor
SensorFusionTracker::SensorFusionTracker()
{
    next_track_id_ = 1;
    association_threshold_ = 15.0;
    track_timeout_ = 5.0;
    quality_threshold_ = 0.5;
    buffer_time_window_ = 0.5;
    radar_noise_pos_ = 3.0;
    radar_noise_vel_ = 0.5;
    vision_noise_pos_ = 25.0;
    process_noise_pos_ = 0.5;
    process_noise_vel_ = 0.2;
    process_noise_yaw_ = 0.1;
    min_hits_to_confirm_ = 3;
    max_misses_ = 5;
    nav_x_ = 0.0;
    nav_y_ = 0.0;
    nav_heading_ = 0.0;
    radar_count_ = 0;
    vision_count_ = 0;
    fusion_count_ = 0;
    tracks_created_ = 0;
    tracks_deleted_ = 0;
}

SensorFusionTracker::~SensorFusionTracker() {}

bool SensorFusionTracker::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);
    MOOSMSG_LIST::iterator p;
    for(p=NewMail.begin(); p!=NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();
        double time = msg.GetTime();
        if(key == "NAV_X") nav_x_ = msg.GetDouble();
        else if(key == "NAV_Y") nav_y_ = msg.GetDouble();
        else if(key == "NAV_HEADING") nav_heading_ = msg.GetDouble();
        else if(key == "RADAR_TARGETS") handleRadarTargets(msg.GetString(), time);
        else if(key == "VISION_REPORT") handleVisionReport(msg.GetString(), time);
    }
    return true;
}

bool SensorFusionTracker::Iterate()
{
    AppCastingMOOSApp::Iterate();
    processBufferedMeasurements();
    double current_time = MOOSTime();
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        double dt = current_time - track->getLastUpdateTime();
        if(dt > 0.01 && dt < 10.0) {
            track->predict(dt);
            track->setLastUpdateTime(current_time);
        }
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
        param.erase(0, param.find_first_not_of(" \t"));
        param.erase(param.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        for(char& c : param) c = tolower(c);
        if(param == "association_threshold") association_threshold_ = atof(value.c_str());
        else if(param == "track_timeout") track_timeout_ = atof(value.c_str());
        else if(param == "quality_threshold") quality_threshold_ = atof(value.c_str());
        else if(param == "radar_noise_pos") radar_noise_pos_ = atof(value.c_str());
        else if(param == "vision_noise_pos") vision_noise_pos_ = atof(value.c_str());
        else if(param == "min_hits_to_confirm") min_hits_to_confirm_ = atoi(value.c_str());
        else if(param == "max_misses") max_misses_ = atoi(value.c_str());
    }
    reportEvent("Config loaded");
}

void SensorFusionTracker::handleRadarTargets(const string& msg, double time)
{
    radar_count_++;
    istringstream iss(msg);
    string target_str;
    while(getline(iss, target_str, ';')) {
        if(target_str.empty()) continue;
        Measurement meas;
        meas.timestamp = time;
        meas.sensor = RADAR;
        meas.has_velocity = true;
        meas.confidence = 0.9;
        istringstream target_stream(target_str);
        string pair;
        while(getline(target_stream, pair, ',')) {
            size_t eq_pos = pair.find('=');
            if(eq_pos == string::npos) continue;
            string key = pair.substr(0, eq_pos);
            string val = pair.substr(eq_pos + 1);
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
        meas.R = Matrix2d::Identity() * radar_noise_pos_ * radar_noise_pos_;
        measurement_buffer_.push(meas);
    }
}

void SensorFusionTracker::handleVisionReport(const string& msg, double time)
{
    vision_count_++;
    Measurement meas;
    meas.timestamp = time;
    meas.sensor = VISION;
    meas.has_velocity = false;
    istringstream iss(msg);
    string pair;
    while(getline(iss, pair, ',')) {
        size_t eq_pos = pair.find('=');
        if(eq_pos == string::npos) continue;
        string key = pair.substr(0, eq_pos);
        string val = pair.substr(eq_pos + 1);
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
    meas.R = Matrix2d::Identity() * vision_noise_pos_ * vision_noise_pos_;
    measurement_buffer_.push(meas);
}

void SensorFusionTracker::processBufferedMeasurements()
{
    if(measurement_buffer_.empty()) return;
    double current_time = MOOSTime();
    vector<Measurement> batch;
    while(!measurement_buffer_.empty()) {
        Measurement meas = measurement_buffer_.front();
        if(current_time - meas.timestamp > buffer_time_window_) {
            measurement_buffer_.pop();
            continue;
        }
        batch.push_back(meas);
        measurement_buffer_.pop();
    }
    if(!batch.empty()) dataAssociation(batch);
}

void SensorFusionTracker::dataAssociation(vector<Measurement>& measurements)
{
    if(measurements.empty()) return;
    int n_tracks = tracks_.size();
    int n_meas = measurements.size();
    if(n_tracks == 0) {
        for(size_t i = 0; i < measurements.size(); i++) {
            initiateTracks({(int)i}, measurements);
        }
        return;
    }
    MatrixXd cost_matrix(n_tracks, n_meas);
    int track_idx = 0;
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        for(int meas_idx = 0; meas_idx < n_meas; meas_idx++) {
            double dist = mahalanobisDistance(*track, measurements[meas_idx]);
            cost_matrix(track_idx, meas_idx) = (dist > association_threshold_) ? 1e9 : dist;
        }
        track_idx++;
    }
    vector<pair<int,int>> assignments = hungarianAssignment(cost_matrix);
    vector<int> assigned_meas(n_meas, 0);
    for(auto& pair : assignments) assigned_meas[pair.second] = 1;
    vector<int> unassigned_meas;
    for(int i = 0; i < n_meas; i++) {
        if(!assigned_meas[i]) unassigned_meas.push_back(i);
    }
    updateTracks(assignments, measurements);
    if(!unassigned_meas.empty()) initiateTracks(unassigned_meas, measurements);
    vector<int> assigned_tracks(n_tracks, 0);
    for(auto& pair : assignments) assigned_tracks[pair.first] = 1;
    track_idx = 0;
    for(auto& pair : tracks_) {
        if(!assigned_tracks[track_idx]) pair.second->incrementMisses();
        track_idx++;
    }
}

void SensorFusionTracker::updateTracks(const vector<pair<int,int>>& assignments, vector<Measurement>& measurements)
{
    int track_idx = 0;
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        for(auto& assign : assignments) {
            if(assign.first == track_idx) {
                track->update(measurements[assign.second]);
                track->incrementAge();
                break;
            }
        }
        track_idx++;
    }
}

void SensorFusionTracker::initiateTracks(const vector<int>& unassigned_meas, vector<Measurement>& measurements)
{
    for(int idx : unassigned_meas) {
        auto track = make_shared<Track>(next_track_id_++, measurements[idx]);
        tracks_[track->getID()] = track;
        tracks_created_++;
    }
}

void SensorFusionTracker::deleteStaleTracks()
{
    vector<int> to_delete;
    for(auto& pair : tracks_) {
        if(pair.second->shouldDelete()) to_delete.push_back(pair.first);
    }
    for(int id : to_delete) {
        tracks_.erase(id);
        tracks_deleted_++;
    }
}

double SensorFusionTracker::mahalanobisDistance(const Track& track, const Measurement& meas)
{
    Vector2d z(meas.x, meas.y);
    Vector2d z_pred(track.getPx(), track.getPy());
    Vector2d innovation = z - z_pred;
    Matrix2d P_pos = track.getCovariance().block<2,2>(0,0);
    Matrix2d S = P_pos + meas.R;
    return sqrt(innovation.transpose() * S.inverse() * innovation);
}

Matrix2d SensorFusionTracker::getMeasurementNoise(SensorType sensor)
{
    double noise = (sensor == RADAR) ? radar_noise_pos_ : vision_noise_pos_;
    return Matrix2d::Identity() * noise * noise;
}

vector<pair<int,int>> SensorFusionTracker::hungarianAssignment(const MatrixXd& cost_matrix)
{
    HungarianAlgorithm hungarian;
    return hungarian.solve(cost_matrix);
}

void SensorFusionTracker::publishFusedContacts()
{
    if(tracks_.empty()) return;
    ostringstream oss;
    bool first = true;
    int fused_count = 0;
    
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        // ONLY publish if:
        // 1. Track is confirmed
        // 2. Quality above threshold
        // 3. Has BOTH radar AND vision updates (FUSED)
        if(!track->isConfirmed() || 
           track->getQuality() < quality_threshold_ ||
           !track->isFused()) {
            continue;
        }
        
        if(!first) oss << ";";
        first = false;
        
        oss << fixed << setprecision(2);
        oss << "id=" << track->getID() << ",";
        oss << "x=" << track->getPx() << ",";
        oss << "y=" << track->getPy() << ",";
        oss << "vx=" << track->getVx() << ",";
        oss << "vy=" << track->getVy() << ",";
        oss << "type=" << track->getType() << ",";
        oss << "quality=" << track->getQuality() << ",";
        oss << "source=FUSED";  // Indicate it's fused
        
        fused_count++;
    }
    
    if(!first) {
        Notify("FUSED_CONTACTS", oss.str());
        fusion_count_ = fused_count;
    }
}

void SensorFusionTracker::publishNodeReports()
{
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        
        // ONLY publish fused tracks
        if(!track->isConfirmed() || 
           track->getQuality() < quality_threshold_ ||
           !track->isFused()) {
            continue;
        }
        
        ostringstream oss;
        oss << fixed << setprecision(2);
        oss << "NAME=fused_" << track->getID() << ",";
        oss << "TYPE=" << (track->getType().empty() ? "ship" : track->getType()) << ",";
        oss << "X=" << track->getPx() << ",";
        oss << "Y=" << track->getPy() << ",";
        oss << "SPD=" << (track->getSpeed() * 1.94384) << ",";
        oss << "HDG=" << (track->getYaw() * 180.0 / M_PI) << ",";
        oss << "TIME=" << setprecision(3) << MOOSTime();
        
        Notify("NODE_REPORT_FUSED", oss.str());
    }
}

bool SensorFusionTracker::buildReport()
{
    m_msgs << "========================================"  << endl;
    m_msgs << "pSensorFusionTracker"  << endl;
    m_msgs << "========================================"  << endl;
    m_msgs << "Radar measurements:    " << radar_count_ << endl;
    m_msgs << "Vision measurements:   " << vision_count_ << endl;
    m_msgs << "Active tracks:         " << tracks_.size() << endl;
    m_msgs << "Fused tracks:          " << fusion_count_ << endl;
    m_msgs << "Tracks created:        " << tracks_created_ << endl;
    m_msgs << "Tracks deleted:        " << tracks_deleted_ << endl;
    m_msgs << endl;
    
    // Show track details
    m_msgs << "Track Details:" << endl;
    m_msgs << "ID | Fused | Quality | Radar | Vision | Type" << endl;
    m_msgs << "---|-------|---------|-------|--------|------" << endl;
    
    for(auto& pair : tracks_) {
        auto& track = pair.second;
        m_msgs << setw(2) << track->getID() << " | ";
        m_msgs << (track->isFused() ? "YES" : "NO ") << "   | ";
        m_msgs << fixed << setprecision(2) << track->getQuality() << "    | ";
        m_msgs << setw(5) << "R" << "  | ";
        m_msgs << setw(6) << "V" << "  | ";
        m_msgs << track->getType() << endl;
    }
    
    return true;
}

Track::Track(int id, const Measurement& init_meas)
    : id_(id), state_(TENTATIVE), age_(1), misses_(0), max_misses_(5),
      quality_(0.3), last_update_time_(init_meas.timestamp),
      radar_updates_(0), vision_updates_(0)
{
    x_ = VectorXd(7);
    x_ << init_meas.x, init_meas.y, 0.0, 0.0, 0.0,
          init_meas.has_velocity ? init_meas.vx : 0.0,
          init_meas.has_velocity ? init_meas.vy : 0.0;
    P_ = MatrixXd::Identity(7, 7) * 10.0;
    Q_ = MatrixXd::Identity(7, 7) * 0.5;
    label_ = init_meas.label;
    type_ = init_meas.type;
    last_sensor_ = init_meas.sensor;
    
    // Initialize fusion flags
    has_radar_update_ = (init_meas.sensor == RADAR);
    has_vision_update_ = (init_meas.sensor == VISION);
}

void Track::predict(double dt)
{
    x_(0) += x_(5) * dt;
    x_(1) += x_(6) * dt;
    P_ += Q_ * dt;
}

void Track::update(const Measurement& meas)
{
    Vector2d z(meas.x, meas.y);
    Vector2d z_pred(x_(0), x_(1));
    Vector2d y = z - z_pred;
    MatrixXd H = MatrixXd::Zero(2, 7);
    H(0,0) = 1.0;
    H(1,1) = 1.0;
    Matrix2d S = H * P_ * H.transpose() + meas.R;
    MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    P_ = (MatrixXd::Identity(7, 7) - K * H) * P_;
    
    if(meas.has_velocity) {
        x_(5) = meas.vx;
        x_(6) = meas.vy;
    }
    
    // Track which sensors have updated this track
    if(meas.sensor == RADAR) {
        has_radar_update_ = true;
        radar_updates_++;
    } else if(meas.sensor == VISION) {
        has_vision_update_ = true;
        vision_updates_++;
        // Update type from vision if available
        if(!meas.type.empty()) {
            type_ = meas.type;
        }
    }
    
    // Higher quality for fused tracks
    if(has_radar_update_ && has_vision_update_) {
        quality_ = min(1.0, quality_ + 0.15);  // Faster quality increase for fused
    } else {
        quality_ = min(1.0, quality_ + 0.1);
    }
    
    if(age_ >= 3) state_ = CONFIRMED;
    last_update_time_ = meas.timestamp;
}

void Track::generateSigmaPoints() {}
void Track::predictSigmaPoints(double dt) {}
VectorXd Track::predictMeasurement(const VectorXd& state) { return VectorXd(2); }

vector<pair<int,int>> HungarianAlgorithm::solve(const MatrixXd& cost_matrix)
{
    int n_rows = cost_matrix.rows();
    int n_cols = cost_matrix.cols();
    if(n_rows == 0 || n_cols == 0) return vector<pair<int,int>>();
    int n = max(n_rows, n_cols);
    MatrixXd cost = MatrixXd::Constant(n, n, 1e9);
    cost.block(0, 0, n_rows, n_cols) = cost_matrix;
    vector<int> row_cover(n, 0);
    vector<int> col_cover(n, 0);
    vector<vector<int>> starred(n, vector<int>(n, 0));
    vector<vector<int>> primed(n, vector<int>(n, 0));
    step1(cost, n);
    step2(cost, n, row_cover, col_cover, starred);
    fill(row_cover.begin(), row_cover.end(), 0);
    fill(col_cover.begin(), col_cover.end(), 0);
    int step = 3;
    bool done = false;
    while(!done && step < 100) {
        if(step == 3) step = step3(n, col_cover, starred);
        else if(step == 4) {
            int zero_row, zero_col;
            if(step4(cost, n, row_cover, col_cover, starred, primed, zero_row, zero_col)) step = 6;
            else { step5(n, row_cover, col_cover, starred, primed, zero_row, zero_col); step = 3; }
        }
        else if(step == 6) { step6(cost, n, row_cover, col_cover); step = 4; }
        else if(step == 7) done = true;
    }
    vector<pair<int,int>> result;
    for(int i = 0; i < n_rows; i++) {
        for(int j = 0; j < n_cols; j++) {
            if(starred[i][j] && cost_matrix(i,j) < 1e8) result.push_back({i, j});
        }
    }
    return result;
}

void HungarianAlgorithm::step1(MatrixXd& cost, int n)
{
    for(int i = 0; i < n; i++) {
        double min_val = cost.row(i).minCoeff();
        if(min_val < 1e8) cost.row(i).array() -= min_val;
    }
    for(int j = 0; j < n; j++) {
        double min_val = cost.col(j).minCoeff();
        if(min_val < 1e8) cost.col(j).array() -= min_val;
    }
}

void HungarianAlgorithm::step2(MatrixXd& cost, int n, vector<int>& row_cover, vector<int>& col_cover, vector<vector<int>>& starred)
{
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            if(cost(i,j) == 0 && !row_cover[i] && !col_cover[j]) {
                starred[i][j] = 1;
                row_cover[i] = 1;
                col_cover[j] = 1;
            }
        }
    }
    fill(row_cover.begin(), row_cover.end(), 0);
    fill(col_cover.begin(), col_cover.end(), 0);
}

int HungarianAlgorithm::step3(int n, vector<int>& col_cover, vector<vector<int>>& starred)
{
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            if(starred[i][j]) col_cover[j] = 1;
        }
    }
    int count = 0;
    for(int j = 0; j < n; j++) if(col_cover[j]) count++;
    return (count >= n) ? 7 : 4;
}

bool HungarianAlgorithm::step4(MatrixXd& cost, int n, vector<int>& row_cover, vector<int>& col_cover, vector<vector<int>>& starred, vector<vector<int>>& primed, int& zero_row, int& zero_col)
{
    for(int i = 0; i < n; i++) {
        if(row_cover[i]) continue;
        for(int j = 0; j < n; j++) {
            if(col_cover[j]) continue;
            if(cost(i,j) == 0) {
                primed[i][j] = 1;
                int star_col = -1;
                for(int k = 0; k < n; k++) {
                    if(starred[i][k]) { star_col = k; break; }
                }
                if(star_col >= 0) { row_cover[i] = 1; col_cover[star_col] = 0; return true; }
                else { zero_row = i; zero_col = j; return false; }
            }
        }
    }
    return true;
}

void HungarianAlgorithm::step5(int n, vector<int>& row_cover, vector<int>& col_cover, vector<vector<int>>& starred, vector<vector<int>>& primed, int zero_row, int zero_col)
{
    vector<pair<int,int>> path;
    path.push_back({zero_row, zero_col});
    bool done = false;
    while(!done) {
        int star_row = -1;
        for(int i = 0; i < n; i++) {
            if(starred[i][path.back().second]) { star_row = i; break; }
        }
        if(star_row >= 0) {
            path.push_back({star_row, path.back().second});
            for(int j = 0; j < n; j++) {
                if(primed[star_row][j]) { path.push_back({star_row, j}); break; }
            }
        } else done = true;
    }
    for(size_t k = 0; k < path.size(); k++) {
        if(k % 2 == 0) starred[path[k].first][path[k].second] = 1;
        else starred[path[k].first][path[k].second] = 0;
    }
    fill(row_cover.begin(), row_cover.end(), 0);
    fill(col_cover.begin(), col_cover.end(), 0);
    for(int i = 0; i < n; i++) fill(primed[i].begin(), primed[i].end(), 0);
}

void HungarianAlgorithm::step6(MatrixXd& cost, int n, vector<int>& row_cover, vector<int>& col_cover)
{
    double min_val = 1e9;
    for(int i = 0; i < n; i++) {
        if(row_cover[i]) continue;
        for(int j = 0; j < n; j++) {
            if(col_cover[j]) continue;
            min_val = min(min_val, cost(i,j));
        }
    }
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            if(row_cover[i]) cost(i,j) += min_val;
            if(!col_cover[j]) cost(i,j) -= min_val;
        }
    }
}

int main(int argc, char *argv[])
{
    SensorFusionTracker app;
    app.Run(argv[0], argc, argv);
    return 0;
}
