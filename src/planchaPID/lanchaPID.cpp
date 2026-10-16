/************************************************************/
/*    NAME: DL                                            */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: lanchaPID.cpp                                        */
/*    DATE: September, 2023                          */
/************************************************************/

#include "MBUtils.h"
#include "ACTable.h"
#include "lanchaPID.h"
#include "AutoTuning.h"
#include "CanProfile.h"
//#include "PIDParameterReader.h"
#include <cmath>
#include <string>
#include <vector>
#include <iterator>

using namespace std;

//Variáveis globais
double nav_heading;  // Rumo atual do navio
double desired_heading;  // Rumo desejado do navio
double nav_speed;  // Velocidade atual do navio
double desired_speed;  // Velocidade desejada do navio
double nav_cog; // COG do navio
double nav_speed_filt = 0;  // filtered speed actually used by the code
bool   nav_speed_init = false;
double speed_alpha = 0.1;          //  nav_speed filter coefficient 10 % new sample, 90 % history
double heading_alpha     = 0.15;   // 15 % new sample, 85 % history
double nav_heading_filt  = 0.0;    // filtered heading (deg)
bool   nav_heading_init  = false;  // first-sample flag


double desired_thrust;  // Saída do controle de velocidade
double desired_rudder;  // Saída do controle de rumo

// Manual speed gains: only used in service mode (normal operation is gain
// scheduled). Seeded with the mid-band schedule values so enabling service mode
// without touching the UI sliders keeps the boat driving sensibly.
double speed_kp = 0.16; //kp da veloc
double speed_ki = 0.003; //ki da veloc
double speed_kd = 0.0; //kd da veloc
int thrust_map = 1;  // set to 1 to use thrust_map speed control

double heading_kp = 0.3; //kp do heading - Modified value for USV
double heading_ki = 0.0; //ki do heading
double heading_kd = 0.06; //kd do heading

// Service mode: when true the gain-scheduling tables below are bypassed and the
// kp/ki/kd pushed from the UI (HEADING_K* / SPEED_K*) are applied directly, so
// the boat can be calibrated live without rebuilding/reconfiguring the backend.
bool service_mode = false;

// Speed-based rudder angle limit (configurable via .moos)
double rudder_limit_max        = 20.0;
double rudder_limit_min        = 8.0;
double rudder_limit_speed_low  = 5.0;
double rudder_limit_speed_high = 18.0;
double rudder_limit_active     = 20.0;

double dt = 0.0;
double output = 0;
std::string deploy; //deploy
std::string moos_manual_overide;
std::string moos_manual_override;
std::string stationing; // verify station keep
std::string trailing; // verify follow-a-contact (BHV_Trail)

std::string constant_heading; //Variável para verificar se o rumo constante é verdadeiro
double setpoint_heading; //Variável em que eu coloco o setpoint de rumo

//Check when autonomous control is active
std::string waypoint; //Variável para verificar se o controle autônomo é verdadeiro
std::string return_var; //
std::string region; // Variável para verificar se o OpRegion está ativo

//Cria os objetos para o controle PID com valores iniciais de kp, ki e kd lidos do arquivo
//PIDController speedController("/home/dueiras/vsnt/moos-ivp-vsnt/src/planchaPID/speed_pid_parameters.txt");
//PIDController headingController("/home/dueiras/vsnt/moos-ivp-vsnt/src/planchaPID/heading_pid_parameters.txt");
// Boat-specific autopilot settings (speed feed-forward, speed gains, rudder
// and thrust limits). Read from the boat profile (auto_* keys) in OnStartUp;
// the defaults are the values that used to be hardcoded here.
canprofile::AutoTuning g_tuning;
std::string g_tuning_source = "built-in defaults";

PIDController speedController(speed_kp, speed_ki, speed_kd);
PIDController headingController(heading_kp, heading_ki, heading_kd);

//---------------------------------------------------------
// Constructor()

PIDController::PIDController(double kp, double ki, double kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

  upper_bound = 20;
  lower_bound = -20;
  integral = 0;
  prevError = 0;
  eff_kp = 0;
  eff_ki = 0;
  eff_kd = 0;
  prevMeas = 0;
  service_mode = false;
}

// CalculateSpeed() – gain-scheduled PID + feed-forward
// thrust output is in the same 0–40 % scale you use elsewhere
double PIDController::CalculateSpeed(double desired, double current, double dt)
{
    // ------- 1. Feed-forward (profile auto_speed_ff) ------
    // Thrust that holds the desired speed, from the boat profile.
    double ff = g_tuning.feedForward(desired);

    // ------- 2. PID feedback (gain-scheduled) -------------
    double error      = desired - current;
    double derivative = (error - prevError)/dt;
    prevError         = error;

    // Service mode: apply the gains pushed from the UI as-is so the operator
    // sees exactly the effect of the values being calibrated. The feed-forward
    // map above is kept - only the feedback term is under manual control.
    if(service_mode) {
        eff_kp = kp;
        eff_ki = ki;
        eff_kd = kd;
    }
    // gain scheduling on the *desired* speed (not the measured one):
    // the gains stay constant throughout a setpoint, so they no longer
    // jump as the boat accelerates across the 2 kt / 6 kt thresholds.
    else {
        // profile auto_speed_gains: kp/ki by desired speed
        g_tuning.gains(desired, eff_kp, eff_ki);
        eff_kd = 0.0;
    }

    // Integral with *bounded authority* anti-wind-up. The old code clamped
    // the raw integral to +/-100, letting the I-term alone command up to
    // eff_ki*100 (40..120 %!) of thrust and stay pinned high while the hull
    // lag caught up -> overshoot. Now we cap the integral so its
    // *contribution* never exceeds +/- I_AUTHORITY percent of thrust.
    const double I_AUTHORITY = g_tuning.i_authority;   // max thrust (%) the I-term may add
    integral += error * dt;
    if(eff_ki > 1e-6){
        double i_max = I_AUTHORITY / eff_ki;
        if(integral >  i_max) integral =  i_max;
        if(integral < -i_max) integral = -i_max;
    }

    double fb = eff_kp*error + eff_ki*integral + eff_kd*derivative;

    // ------- 3. Combine and clamp -------------------------
    output = ff + fb;

    // Back-calculation anti-wind-up: when the output saturates, pull the
    // integral back so it does not keep accumulating authority the actuator
    // cannot deliver (removes the long "stuck high" phase of the cycle).
    double clamped = output;
    if(clamped > 100.0)    clamped = 100.0;   // max thrust 100 %
    else if(clamped < 0.0) clamped = 0.0;     // no negative thrust
    if(clamped != output && eff_ki > 1e-6)
        integral += (clamped - output) / eff_ki;
    output = clamped;
    //upper_bound = 40;                             // for AppCasting table
    //lower_bound = 0;

    return output;
}


//---------------------------------------------------------
// Calculate()

double PIDController::Calculate(double desired, double current, double dt) {
    
  
    double error = desired - current;

    //Passando para a escala de -180 a 180
    // Isso não é o ideal para o controlador de velocidade
    // Porém hoje os valores não chegam a -180 ou 180
    if (error < -180){
      error += 360;
    } else if (error > 180){
      error -= 360;
    }

    integral += error*dt; //Multiplico pelo período dt de tempo

    double dmeas = current - prevMeas;
    if (dmeas >  180) dmeas -= 360;
    else if (dmeas < -180) dmeas += 360;
    double derivative = -dmeas / dt;
    prevMeas = current;

    // Service mode: the gains pushed from the UI (HEADING_KP/KI/KD) are applied
    // directly, bypassing the speed-based schedule below, so the operator can
    // calibrate the boat live and read the result on the rudder.
    if (service_mode) {
      eff_kp = kp;
      eff_ki = ki;
      eff_kd = kd;
    }
    else {
      // profile auto_heading_gains: kp/ki/kd by boat speed (NAV_SPEED)
      g_tuning.headingGains(nav_speed, eff_kp, eff_ki, eff_kd);
    }

    // Anti-wind-up. With an I-term from the profile, cap its contribution
    // at auto_heading_i_authority_pct of rudder; it trims out a steady
    // pull or current. Otherwise keep the old clamp to the output bounds.
    if (!service_mode && eff_ki > 1e-6 && g_tuning.heading_i_authority > 0) {
      double i_max = g_tuning.heading_i_authority / eff_ki;
      if (integral >  i_max) integral =  i_max;
      if (integral < -i_max) integral = -i_max;
    }
    else if (integral > upper_bound) {
      integral = upper_bound;
    }
    else if (integral < lower_bound) {
      integral = lower_bound;
    }

    output = eff_kp * error + eff_ki * integral + eff_kd * derivative;

    // Limits the output value
    if (output > upper_bound)
      output = upper_bound;
    else if (output < lower_bound)
      output = lower_bound;

    prevError = error;

    return output;
} // Calculate()

//---------------------------------------------------------
// setKP()

void PIDController::setKP(double KP) {
  kp = KP;
} // setKP()

//---------------------------------------------------------
// setKI()

void PIDController::setKI(double KI) {
  ki = KI;
  integral = 0;
} // setKI()

//---------------------------------------------------------
// setKD()

void PIDController::setKD(double KD) {
  kd = KD;
} // setKD()

//---------------------------------------------------------
// setServiceMode()
//   Switches between the built-in gain schedule and the kp/ki/kd pushed from
//   the UI. The integral is dropped on every transition so the controller does
//   not carry accumulated authority from one gain set into the other.

void PIDController::setServiceMode(bool enabled) {
  if(service_mode != enabled)
    integral = 0;
  service_mode = enabled;
} // setServiceMode()

//---------------------------------------------------------
// getServiceMode()

bool PIDController::getServiceMode() const {
  return service_mode;
} // getServiceMode()

//---------------------------------------------------------
// setUpper()

void PIDController::setUpper(double Upper) {
  upper_bound = Upper;
} // setUpper()

//---------------------------------------------------------
// setLower()

void PIDController::setLower(double Lower) {
  lower_bound = Lower;
} // setKD()

//---------------------------------------------------------
// getKP()

double PIDController::getKP() {
  return kp;
} // getKP()

//---------------------------------------------------------
// getKI()

double PIDController::getKI() {
  return ki;
} // getKI()

//---------------------------------------------------------
// getIterm()

double PIDController::getIterm() {
  return integral;
} // getIterm()

//---------------------------------------------------------
// getKD()

double PIDController::getKD() {
  return kd;
} // getKD()

// Gain Schedulling functions

double PIDController::getEffKP() { return eff_kp; }
double PIDController::getEffKI() { return eff_ki; }
double PIDController::getEffKD() { return eff_kd; }

//---------------------------------------------------------
// resetIntegral()

void PIDController::resetIntegral() {
  integral = 0.0;
} // resetIntegral()

//---------------------------------------------------------
// Constructor()

lanchaPID::lanchaPID()
{
}

//---------------------------------------------------------
// Destructor

lanchaPID::~lanchaPID()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool lanchaPID::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_HEADING") {
      double raw = msg.GetDouble();        // incoming 0–360 °

      if(!nav_heading_init) {              // first sample
          nav_heading_filt = raw;
          nav_heading_init = true;
      } else {
          // --- identical EMA structure to NAV_SPEED ---
          // 1) shortest signed difference (handles 0/360 wrap)
          double diff = raw - nav_heading_filt;
          if(diff > 180.0)       diff -= 360.0;
          else if(diff < -180.0) diff += 360.0;

          // 2) integrate a fraction of that difference
          nav_heading_filt += heading_alpha * diff;

          // 3) wrap result back to 0–360 for downstream code
          if(nav_heading_filt < 0.0)        nav_heading_filt += 360.0;
          else if(nav_heading_filt >= 360.) nav_heading_filt -= 360.0;
      }

      nav_heading = nav_heading_filt;       // PID sees filtered value
      }

     else if(key == "DESIRED_HEADING") 
       desired_heading = msg.GetDouble();
     else if(key == "NAV_SPEED"){
       double raw = msg.GetDouble();
       if(!nav_speed_init) {          // Filter to nav_speed
        nav_speed_filt  = raw;
        nav_speed_init  = true;
        } else {
            nav_speed_filt  = speed_alpha * raw +
                              (1.0 - speed_alpha) * nav_speed_filt;
        }

        nav_speed = nav_speed_filt;
        }
     else if(key == "DESIRED_SPEED") 
       desired_speed = msg.GetDouble();  
     else if(key == "SPEED_KP") 
       speedController.setKP(msg.GetDouble());
     else if(key == "SPEED_KI") 
       speedController.setKI(msg.GetDouble());
     else if(key == "SPEED_KD") 
       speedController.setKD(msg.GetDouble());
     else if(key == "HEADING_KP") 
       headingController.setKP(msg.GetDouble());
     else if(key == "HEADING_KI") 
       headingController.setKI(msg.GetDouble());
     else if(key == "HEADING_KD")
       headingController.setKD(msg.GetDouble());
     else if(key == "SERVICE_MODE") {
       service_mode = (msg.GetString() == "true");
       headingController.setServiceMode(service_mode);
       speedController.setServiceMode(service_mode);
       }
     else if(key == "DEPLOY") {
       deploy = msg.GetString(); 
       // Reset PID error  
       headingController.resetIntegral();
       speedController.resetIntegral();
       }
     else if(key == "MOOS_MANUAL_OVERRIDE") 
       moos_manual_overide = msg.GetString();  
     else if(key == "CONSTANT_HEADING") 
       constant_heading = msg.GetString();  
     else if(key == "SETPOINT_HEADING") 
       setpoint_heading = msg.GetDouble();  
      else if(key == "WAYPOINT")
        waypoint = msg.GetString();
      else if(key == "RETURN")
        return_var = msg.GetString();
      else if(key == "NAV_COG")
        nav_cog = msg.GetDouble();
      else if(key == "STATIONING")
        stationing = msg.GetString();
      else if(key == "TRAILING")
        trailing = msg.GetString();
      else if(key == "REGION")
        region = msg.GetString();
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool lanchaPID::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: speedRudderLimit()
//   Max rudder magnitude (deg) allowed for a given speed.

static double speedRudderLimit(double speed)
{
  if(speed <= rudder_limit_speed_low)
    return rudder_limit_max;
  if(speed >= rudder_limit_speed_high)
    return rudder_limit_min;

  double w = (speed - rudder_limit_speed_low) /
             (rudder_limit_speed_high - rudder_limit_speed_low);
  return rudder_limit_max + w * (rudder_limit_min - rudder_limit_max);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool lanchaPID::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Loop de controle

  // (The speed loop is CalculateSpeed() below. A leftover call here ran the
  // heading algorithm on the speed controller, clamping its integral to
  // +/-20 every tick, so the speed I-term could add only ~0.1 % thrust.)

  // Calcule o controle de rumo
  //Caso o rumo constante esteja ativado o setpoint vai ser esse rumo
  if (constant_heading == "true"){
    desired_rudder = headingController.Calculate(setpoint_heading, nav_heading,dt);
  } else {
    desired_rudder = headingController.Calculate(desired_heading, nav_heading,dt);
  }

  // Speed-based rudder limit (tightens HEADING_UPPER/LOWER_BOUND, never relaxes)
  rudder_limit_active = speedRudderLimit(nav_speed);
  if(desired_rudder >  rudder_limit_active) desired_rudder =  rudder_limit_active;
  if(desired_rudder < -rudder_limit_active) desired_rudder = -rudder_limit_active;

  //Speed controller - New function
  desired_thrust = speedController.CalculateSpeed(desired_speed, nav_speed,dt);

  // Highest thrust the autopilot may ask for (profile auto_thrust_max_pct)
  if(desired_thrust > g_tuning.thrust_max)
    desired_thrust = g_tuning.thrust_max;

  // Imprima as saídas dos controladores

  //Publico DESIRED_RUDDER e DESIRED_THRUST
  //Just publish when waypoint behavior is true and manual override is false
  //BHV_Waypoint is active, or StationKeep is active, or OpRegion is active
  // TRAILING is the follow-a-contact mode (BHV_Trail). Without it here the helm
  // computes a pursuit solution that never reaches the actuators, because
  // engaging Follow deliberately clears WAYPOINT to stop the mission fighting it.
  // Never while SS UI has manual control (MOOS_MANUAL_OVERRIDE=true): the
  // operator's rudder/throttle must not be overwritten. iCAN also ignores
  // planchaPID while in manual, so this is belt and braces.
  bool manual_control = (moos_manual_overide == "true");
  if (!manual_control &&
      ((waypoint == "true" && moos_manual_overide == "false") ||
       (trailing == "true" && moos_manual_overide == "false") ||
       stationing == "true" || region == "true")) {
    Notify("DESIRED_THRUST", desired_thrust);
    Notify("DESIRED_RUDDER", desired_rudder);
  }
  else {
    // Not driving the boat: do not let the I-terms build up on errors
    // the autopilot is not acting on (manual control, no active mode).
    headingController.resetIntegral();
    speedController.resetIntegral();
  }

  // Report the gains actually being applied this cycle so the UI can show the
  // operator what the controller is running on (the manual values in service
  // mode, the scheduled ones otherwise).
  string pid_gains = "service_mode=";
  pid_gains += (service_mode ? "true" : "false");
  pid_gains += ",heading_kp=" + doubleToString(headingController.getEffKP(), 4);
  pid_gains += ",heading_ki=" + doubleToString(headingController.getEffKI(), 4);
  pid_gains += ",heading_kd=" + doubleToString(headingController.getEffKD(), 4);
  pid_gains += ",speed_kp="   + doubleToString(speedController.getEffKP(), 4);
  pid_gains += ",speed_ki="   + doubleToString(speedController.getEffKI(), 4);
  pid_gains += ",speed_kd="   + doubleToString(speedController.getEffKD(), 4);
  Notify("PID_GAINS", pid_gains);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool lanchaPID::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  string profile_file;
  vector<string> moos_tuning;   // tuning keys also set in this .moos block

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));

    // Boat profile with the auto_* tuning (string, so before stod below)
    if(param == "profile_file") {
      profile_file = line;
      continue;
    }
    if(param.find("heading_") == 0 || param.find("rudder_limit_") == 0)
      moos_tuning.push_back(param);

    // SERVICE_MODE is the only boolean parameter - handle it before the
    // numeric conversion below, which would throw on "true"/"false".
    if(param == "service_mode") {
      service_mode = (tolower(line) == "true");
      headingController.setServiceMode(service_mode);
      speedController.setServiceMode(service_mode);
      continue;
    }

    double value = std::stod(line);

    bool handled = false;
    if(param == "apptick") {
      dt = 1/value;
      handled = true;
    }
    else if(param == "heading_kp") {
      headingController.setKP(value);
      handled = true;
    }
    else if(param == "heading_ki") {
      headingController.setKI(value);
      handled = true;
    }
    else if(param == "heading_kd") {
      headingController.setKD(value);
      handled = true;
    }
    // Manual speed gains: applied only while service mode is active.
    else if(param == "speed_kp") {
      speedController.setKP(value);
      handled = true;
    }
    else if(param == "speed_ki") {
      speedController.setKI(value);
      handled = true;
    }
    else if(param == "speed_kd") {
      speedController.setKD(value);
      handled = true;
    }
    else if(param == "heading_upper_bound") {
      headingController.setUpper(value);
      handled = true;
    }
    else if(param == "heading_lower_bound") {
      headingController.setLower(value);
      handled = true;
    }
    else if(param == "rudder_limit_max") {
      rudder_limit_max = value;
      handled = true;
    }
    else if(param == "rudder_limit_min") {
      rudder_limit_min = value;
      handled = true;
    }
    else if(param == "rudder_limit_speed_low") {
      rudder_limit_speed_low = value;
      handled = true;
    }
    else if(param == "rudder_limit_speed_high") {
      rudder_limit_speed_high = value;
      handled = true;
    }
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // No profile_file here: use the one iCAN loads, so one line in the
  // mission selects the boat for both apps.
  if(profile_file.empty()) {
    STRING_LIST ican;
    if(m_MissionReader.GetConfiguration("iCAN", ican)) {
      for(STRING_LIST::iterator q=ican.begin(); q!=ican.end(); q++) {
        string line = *q;
        if(tolower(biteStringX(line, '=')) == "profile_file")
          profile_file = line;
      }
    }
  }

  if(profile_file.empty())
    reportConfigWarning("No profile_file: using built-in autopilot tuning");
  else {
    canprofile::Profile prof;
    string err;
    if(!prof.load(profile_file)) {
      reportConfigWarning("Profile " + profile_file +
                          " is invalid: using built-in autopilot tuning");
      for(size_t i=0; i<prof.errors().size(); i++)
        reportConfigWarning("  " + prof.errors()[i]);
    }
    else if(!g_tuning.configure(prof, &err))
      reportConfigWarning("Profile " + profile_file + ": " + err);
    else {
      g_tuning_source = profile_file;
      // The profile is the one source of these values on a real boat
      for(size_t i=0; i<moos_tuning.size(); i++)
        reportConfigWarning(toupper(moos_tuning[i]) +
                            " in the .moos file is ignored: set it in " + profile_file);
    }
  }

  // With a profile, its heading gains and rudder limits replace the .moos
  // values. Without one, the .moos values (or the built-in ones) stay.
  if(g_tuning_source != "built-in defaults") {
    headingController.setUpper(g_tuning.heading_limit);
    headingController.setLower(-g_tuning.heading_limit);
    rudder_limit_max        = g_tuning.rudder_max;
    rudder_limit_min        = g_tuning.rudder_min;
    rudder_limit_speed_low  = g_tuning.rudder_speed_low;
    rudder_limit_speed_high = g_tuning.rudder_speed_high;
  }

  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void lanchaPID::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("DESIRED_SPEED", 0);
  Register("SPEED_KP", 0);
  Register("SPEED_KI", 0);
  Register("SPEED_KD", 0);
  Register("HEADING_KP", 0);
  Register("HEADING_KI", 0);
  Register("HEADING_KD", 0);
  Register("DEPLOY", 0);
  Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  Register("CONSTANT_HEADING",0);
  Register("SETPOINT_HEADING",0);
  Register("WAYPOINT",0);
  Register("RETURN",0);
  Register("STATIONING",0);
  Register("TRAILING",0);
  Register("REGION",0);
  Register("SERVICE_MODE",0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool lanchaPID::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab1(4);
  actab1 << "DESIRED_THRUST | DESIRED_RUDDER | CONSTANT_HEADING | SETPOINT_HEADING ";
  actab1.addHeaderLines();
  actab1 << desired_thrust << desired_rudder << constant_heading << setpoint_heading;
  m_msgs << actab1.getFormattedString();
  m_msgs << endl;
  m_msgs << "---------------------------------------------------" << endl;

  m_msgs << "Tuning: " << g_tuning_source << endl;
  m_msgs << "  rudder limit " << rudder_limit_max << "% (<= " << rudder_limit_speed_low
         << ") .. " << rudder_limit_min << "% (>= " << rudder_limit_speed_high
         << ")   thrust max " << g_tuning.thrust_max << "%   speed I-term max "
         << g_tuning.i_authority << "%" << endl;
  m_msgs << "  speed feed-forward: " << g_tuning.feedForward(desired_speed)
         << "% for desired speed " << desired_speed << endl;
  m_msgs << "---------------------------------------------------" << endl;

  m_msgs << "SERVICE MODE (manual gains): "
         << (service_mode ? "ON" : "off") << endl;
  m_msgs << "---------------------------------------------------" << endl;

  ACTable actab2(4);
  actab2 << "LOOP | EFF_KP | EFF_KI | EFF_KD";
  actab2.addHeaderLines();
  actab2 << "heading"
        << headingController.getEffKP()
        << headingController.getEffKI()
        << headingController.getEffKD();
  actab2 << "speed"
        << speedController.getEffKP()
        << speedController.getEffKI()
        << speedController.getEffKD();
  m_msgs << actab2.getFormattedString();
  m_msgs << endl;
  m_msgs << "---------------------------------------------------" << endl;

  ACTable actab3(2);
  actab3 << "NAV_SPEED (kt) | RUDDER_LIMIT (deg)";
  actab3.addHeaderLines();
  actab3 << nav_speed << rudder_limit_active;
  m_msgs << actab3.getFormattedString();
  m_msgs << endl;

  return(true);
}
